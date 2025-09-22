#![no_std]
#![no_main]
mod adc;
mod as5047p;
mod bldc_driver;
mod can_message;
mod clock;
mod drv8316;
mod pwm;

use core::ops::Sub;

use crate::{
    adc::AdcSelector,
    as5047p::As5047P,
    bldc_driver::PwmDriver,
    can_message::{Command, MotorStatus, StatusMessage},
    drv8316::Drv8316,
};

use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    Config, bind_interrupts,
    can::{self, Can},
    gpio, i2c as stm32_i2c,
    mode::Async,
    peripherals::{self, ADC1, TIM1},
    time::mhz,
    timer::AdvancedInstance4Channel,
};
use embassy_stm32::{
    adc as stm32_adc,
    peripherals::{PA8, PA9, PA10, PB13, PB14, PB15},
    spi as stm32_spi,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer, WithTimeout};

use embedded_can::Id;
use foc::{
    angle_input::AngleInput,
    controller::{Direction, FocController, RunMode},
    pwm_output::DutyCycle3Phase,
    units::{Radian, RadianPerSecond, Second},
};
use foc::{
    controller::{clarke_transform, park_transform},
    pwm_output::PwmOutput,
};
use libm::atan2f;

static COMMAND_CHANNEL: Channel<ThreadModeRawMutex, Command, 3> = Channel::new();
static STATUS_CHANNEL: Channel<ThreadModeRawMutex, MotorStatus, 64> = Channel::new();

type SpiMutex = embassy_sync::mutex::Mutex<
    embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    Option<stm32_spi::Spi<'static, Async>>,
>;

#[derive(Clone, Copy)]
struct PhaseCurrent {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

impl PhaseCurrent {
    pub fn new(a: f32, b: f32, c: f32) -> Self {
        Self { a, b, c }
    }
}

impl Sub<PhaseCurrent> for PhaseCurrent {
    type Output = Self;
    fn sub(self, c: PhaseCurrent) -> Self::Output {
        PhaseCurrent::new(self.a - c.a, self.b - c.b, self.c - c.c)
    }
}

static SPI: SpiMutex = SpiMutex::new(None);

async fn get_average_adc<T>(
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    n: usize,
) -> PhaseCurrent
where
    T: stm32_adc::Instance + AdcSelector,
{
    let mut ia_avg = 0.0;
    let mut ib_avg = 0.0;
    let mut ic_avg = 0.0;
    for _ in 0..n {
        let (ia_raw, ib_raw, ic_raw) = p_adc.read();
        let (ia, ib, ic) = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
        ia_avg += ia;
        ib_avg += ib;
        ic_avg += ic;
        Timer::after_millis(1).await;
    }
    ia_avg /= n as f32;
    ib_avg /= n as f32;
    ic_avg /= n as f32;
    PhaseCurrent::new(ia_avg, ib_avg, ic_avg)
}

fn zero_phase() -> DutyCycle3Phase {
    DutyCycle3Phase {
        t1: 0.0,
        t2: 0.0,
        t3: 0.0,
    }
}

async fn get_current_offset<'a, T, TIM>(
    p_adc: &mut adc::DummyAdc<T>,
    driver: &mut PwmDriver<'a, TIM>,
    csa_gain: drv8316::CsaGain,
) -> PhaseCurrent
where
    T: stm32_adc::Instance + AdcSelector,
    TIM: AdvancedInstance4Channel,
{
    driver.run(zero_phase());
    Timer::after_millis(100).await;
    get_average_adc(p_adc, csa_gain, 100).await
}

async fn get_phase_mapping<'a, T, TIM>(
    p_adc: &mut adc::DummyAdc<T>,
    driver: &mut PwmDriver<'a, TIM>,
    csa_gain: drv8316::CsaGain,
    offset: PhaseCurrent,
    align_ratio: f32,
) -> Option<(u8, u8, u8)>
where
    T: stm32_adc::Instance + AdcSelector,
    TIM: AdvancedInstance4Channel,
{
    let mut phase = zero_phase();
    for i in 0..100 {
        phase.t1 = align_ratio * (i as f32) / 100.0;
        driver.run(phase);
        Timer::after_millis(1).await;
    }
    Timer::after_millis(100).await;
    let c = get_average_adc(p_adc, csa_gain, 20).await - offset;
    let phase_1 = max_index(c.a, c.b, c.c)?;

    phase = zero_phase();
    for i in 0..100 {
        phase.t2 = align_ratio * (i as f32) / 100.0;
        driver.run(phase);
        Timer::after_millis(1).await;
    }
    Timer::after_millis(100).await;
    let c = get_average_adc(p_adc, csa_gain, 20).await - offset;
    let phase_2 = max_index(c.a, c.b, c.c)?;
    if phase_2 == phase_1 {
        return None;
    }

    phase = zero_phase();
    for i in 0..100 {
        phase.t3 = align_ratio * (i as f32) / 100.0;
        driver.run(phase);
        Timer::after_millis(1).await;
    }
    driver.run(phase);
    Timer::after_millis(100).await;
    let c = get_average_adc(p_adc, csa_gain, 20).await - offset;
    let phase_3 = max_index(c.a, c.b, c.c)?;
    if phase_3 == phase_1 || phase_3 == phase_2 {
        return None;
    }

    Some((phase_1, phase_2, phase_3))
}

fn max_index(v1: f32, v2: f32, v3: f32) -> Option<u8> {
    if v1 > v2 && v1 > v3 {
        Some(0)
    } else if v2 > v1 && v2 > v3 {
        Some(1)
    } else if v3 > v1 && v3 > v2 {
        Some(2)
    } else {
        None
    }
}

async fn align_current<'a, T, TIM>(
    p_adc: &mut adc::DummyAdc<T>,
    driver: &mut PwmDriver<'a, TIM>,
    csa_gain: drv8316::CsaGain,
    current_offset: PhaseCurrent,
    sensor: &mut As5047P<'a>,
    foc: &FocController,
) -> Option<f32>
where
    T: stm32_adc::Instance + AdcSelector,
    TIM: AdvancedInstance4Channel,
{
    // assumption: mechanical angle and voltage (electrical) angle are aligned
    let a_duty = 0.2;

    // start with zero voltage
    let mut duty = zero_phase();
    duty.t1 = a_duty;
    driver.run(duty);
    Timer::after_millis(100).await;

    // set voltage phase to zero
    // measure current phase n_measure times and take average
    let n_max_try = 100;
    let n_measure = 100;
    let mut n_success = 0;
    let mut avg_angle = 0.0;
    for _ in 0..n_max_try {
        let (ia_raw, ib_raw, ic_raw) = p_adc.read();
        let (ia, ib, ic) = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
        let (ia, ib, ic) = foc.map_currents(
            ia - current_offset.a,
            ib - current_offset.b,
            ic - current_offset.c,
        );
        if let Ok(reading) = sensor.read_async().await {
            let angle = reading.angle;
            let e_angle = foc.to_electrical_angle(angle);

            let (i_alpha, i_beta) = clarke_transform(ia, ib, ic);
            let (i_q, i_d) = park_transform(i_alpha, i_beta, e_angle);
            let i_angle = atan2f(i_q, i_d);
            avg_angle += i_angle - e_angle.angle;
            n_success += 1;

            if n_success >= n_measure {
                break;
            }
        }
        Timer::after_millis(1).await;
    }

    match n_success {
        0 => None,
        _ => Some(avg_angle / (n_success as f32)),
    }
}

#[allow(dead_code)]
#[embassy_executor::task]
async fn foc_sensorless_task(
    p_spi: &'static SpiMutex,
    cs_sensor_pin: gpio::Output<'static>,
    cs_drv_pin: gpio::Output<'static>,
    mut drvoff_pin: gpio::Output<'static>,
    pwm_timer: pwm::Pwm6Timer<'static, TIM1, PA8, PA9, PA10, PB13, PB14, PB15>,
) {
    drvoff_pin.set_high();

    let mut driver = PwmDriver::new(pwm_timer.timer);
    let mut sensor = As5047P::new(p_spi, cs_sensor_pin);
    let mut gate_driver = Drv8316::new(p_spi, cs_drv_pin);
    Timer::after_millis(1).await; // ready time of gate driver

    gate_driver.unlock_registers().await.unwrap();
    let config = drv8316::BuckRegulatorConfig {
        enable: true,
        voltage: drv8316::BuckVoltage::V5_0,
        current_limit: drv8316::BuckCurrentLimit::Limit200mA,
    };
    gate_driver.configure_buck_regulator(config).await.unwrap();
    gate_driver.lock_registers().await.unwrap();
    Timer::after_millis(1).await; // wait for register value update

    let psu_voltage = 16.0;
    let mut foc = FocController::new(
        foc::controller::MotorSetup {
            pole_pair_count: 11,
            phase_resistance: 9.0,
            max_velocity: 50.0,
        },
        psu_voltage,
        foc::pid::PID {
            p: 0.5,
            i: 1000.0,
            d: 0.0,
        },
        foc::pid::PID {
            p: 16.0,
            i: 0.0,
            d: 0.0,
        },
        foc::pid::PID {
            p: 0.2,
            i: 12.0,
            d: 0.0,
        },
        12.0,
        1000.0,
    );
    foc.disable_current_sensing();

    driver.stop();
    drvoff_pin.set_low();

    let align_voltage: f32 = 3.0;

    match sensor.initialize().await {
        Ok(_) => info!("Sensor initialized"),
        Err(e) => error!("Sensor initialization failed, {:?}", e),
    }

    let mut read_sensor = {
        let s = &mut sensor;
        async || s.read_async().await.unwrap()
    };
    let set_motor = |d: DutyCycle3Phase| driver.run(d);
    let wait_seconds =
        async |s: Second| Timer::after(Duration::from_millis((s.0 * 1e3) as u64)).await;

    match foc
        .align_sensor(align_voltage, &mut read_sensor, set_motor, wait_seconds)
        .await
    {
        Ok(_) => {
            info!(
                "Sensor aligned {} {}",
                foc.bias_angle.angle,
                foc.sensor_direction == Direction::Clockwise
            );
        }
        Err(_) => {
            error!("Sensor align failed");
            return;
        }
    };

    foc.set_run_mode(RunMode::Impedance);
    foc.set_target_angle(Radian::new(0.0));
    foc.set_target_velocity(RadianPerSecond(0.0));
    foc.set_target_torque(0.0);
    foc.set_spring(4.0);
    foc.set_damping(0.0);

    foc.enable();

    let command_channel = COMMAND_CHANNEL.receiver();
    let status_channel = STATUS_CHANNEL.sender();

    let mut last_logged_at = Instant::from_secs(0);
    let mut count: usize = 0;

    let mut monitor_period: u8 = 100;
    let mut monitor_tick: u8 = 0;

    loop {
        count += 1;

        if let Ok(command) = command_channel.try_receive() {
            match command {
                Command::Enable => foc.enable(),
                Command::Stop => foc.stop(),
                Command::SetRunMode(m) => foc.set_run_mode(m),
                Command::SetAngle(p) => foc.set_target_angle(Radian::new(p)),
                Command::SetVelocity(v) => foc.set_target_velocity(RadianPerSecond(v)),
                Command::SetTorque(t) => foc.set_target_torque(t),
                Command::SetSpeedLimit(v) => foc.set_velocity_limit(RadianPerSecond(v)),
                Command::SetCurrentLimit(i) => foc.set_current_limit(i),
                Command::SetTorqueLimit(t) => foc.set_torque_limit(t),

                Command::SetCurrentKp(kp) => foc.set_current_kp(kp),
                Command::SetCurrentKi(ki) => foc.set_current_ki(ki),

                Command::SetVelocityKp(kp) => foc.set_velocity_kp(kp),
                Command::SetVelocityKi(ki) => foc.set_velocity_ki(ki),
                Command::SetVelocityGain(tf) => foc.set_velocity_gain(tf),
                Command::SetAngleKp(kp) => foc.set_angle_kp(kp),
                Command::SetSpring(k) => foc.set_spring(k),
                Command::SetDamping(b) => foc.set_damping(b),

                Command::SetMonitorInterval(period) => monitor_period = period,
                _ => (),
            }
        }
        match sensor.read_async().await {
            Ok(reading) => match foc.get_duty_cycle(&reading, 0.0, 0.0, 0.0) {
                Ok(duty) => {
                    driver.run(duty);

                    if let Some(state) = foc.state {
                        monitor_tick += 1;
                        if monitor_tick >= monitor_period {
                            let motor_status = MotorStatus {
                                raw_angle: f32_to_u16(state.angle.angle, -100.0, 100.0),
                                raw_velocity: f32_to_u16(state.filtered_velocity.0, -100.0, 100.0),
                                raw_torque: f32_to_u16(state.i_q, -10.0, 10.0),
                                raw_temperature: 0x00,
                            };
                            status_channel.send(motor_status).await;
                            monitor_tick = 0;
                        }
                    }

                    let now = Instant::now();
                    if let Some(log_after) = last_logged_at.checked_add(Duration::from_millis(100))
                        && now > log_after
                    {
                        let second_since_last_log = (now - last_logged_at).as_micros() as f32 / 1e6;

                        last_logged_at = now;

                        if let Some(state) = foc.state {
                            info!(
                                "target: a = {}, v = {}, i_ref = {}",
                                state.target_angle, state.target_velocity, state.i_ref,
                            );

                            info!(
                                "a = {}, v = {}, iq = {}, v_q = {}",
                                reading.angle.angle,
                                state.filtered_velocity.0,
                                state.i_q,
                                state.v_q,
                            );
                        }

                        info!("loop = {} Hz", count as f32 / second_since_last_log);
                        count = 0;
                    };
                }
                Err(e) => match e {
                    foc::pwm::FocError::AlignError => error!("AlignError"),
                    foc::pwm::FocError::CalculationError => error!("CalculationError"),
                    foc::pwm::FocError::InvalidParameters => error!("InvalidParam"),
                },
            },
            Err(e) => error!("Failed to read from sensor: {:?}", e),
        }

        Timer::after(Duration::from_micros(1)).await;
    }
}

#[embassy_executor::task]
async fn foc_task(
    p_spi: &'static SpiMutex,
    cs_sensor_pin: gpio::Output<'static>,
    cs_drv_pin: gpio::Output<'static>,
    mut drvoff_pin: gpio::Output<'static>,
    pwm_timer: pwm::Pwm6Timer<'static, TIM1, PA8, PA9, PA10, PB13, PB14, PB15>,
    mut p_adc: adc::DummyAdc<ADC1>,
) {
    drvoff_pin.set_high();

    let mut driver = PwmDriver::new(pwm_timer.timer);
    let mut sensor = As5047P::new(p_spi, cs_sensor_pin);
    let mut gate_driver = Drv8316::new(p_spi, cs_drv_pin);
    Timer::after_millis(1).await; // ready time of gate driver

    let csa_gain = drv8316::CsaGain::Gain0_3V;

    gate_driver.unlock_registers().await.unwrap();
    gate_driver.set_csa_gain(csa_gain).await.unwrap();
    let config = drv8316::BuckRegulatorConfig {
        enable: true,
        voltage: drv8316::BuckVoltage::V5_0,
        current_limit: drv8316::BuckCurrentLimit::Limit200mA,
    };
    gate_driver.configure_buck_regulator(config).await.unwrap();
    gate_driver.lock_registers().await.unwrap();
    Timer::after_millis(1).await; // wait for register value update

    let psu_voltage = 16.0;
    let mut foc = FocController::new(
        foc::controller::MotorSetup {
            pole_pair_count: 11,
            phase_resistance: 9.0,
            max_velocity: 50.0,
        },
        psu_voltage,
        foc::pid::PID {
            p: 0.5,
            i: 1000.0,
            d: 0.0,
        },
        foc::pid::PID {
            p: 2.0,
            i: 0.0,
            d: 0.0,
        },
        foc::pid::PID {
            p: 0.04,
            i: 0.1,
            d: 0.0,
        },
        12.0,
        1000.0,
    );

    driver.stop();
    drvoff_pin.set_low();

    let align_voltage: f32 = 3.0;
    let current_offset = get_current_offset(&mut p_adc, &mut driver, csa_gain).await;
    if let Some(m) = get_phase_mapping(
        &mut p_adc,
        &mut driver,
        csa_gain,
        current_offset,
        align_voltage / foc.motor.phase_resistance,
    )
    .await
    {
        info!("phase mapping = {} {} {}", m.0, m.1, m.2);
        foc.current_mapping = m;
    } else {
        error!("Current phase mapping failed");
        driver.stop();
        drvoff_pin.set_high();
        return;
    }

    match sensor.initialize().await {
        Ok(_) => info!("Sensor initialized"),
        Err(e) => error!("Sensor initialization failed, {:?}", e),
    }

    let mut read_sensor = {
        let s = &mut sensor;
        async || s.read_async().await.unwrap()
    };
    let set_motor = |d: DutyCycle3Phase| driver.run(d);
    let wait_seconds =
        async |s: Second| Timer::after(Duration::from_millis((s.0 * 1e3) as u64)).await;

    match foc
        .align_sensor(align_voltage, &mut read_sensor, set_motor, wait_seconds)
        .await
    {
        Ok(_) => {
            info!(
                "Sensor aligned {} {}",
                foc.bias_angle.angle,
                foc.sensor_direction == Direction::Clockwise
            );
        }
        Err(_) => {
            error!("Sensor align failed");
            return;
        }
    };

    if let Some(offset) = align_current(
        &mut p_adc,
        &mut driver,
        csa_gain,
        current_offset,
        &mut sensor,
        &foc,
    )
    .await
    {
        foc.set_current_phase_bias(offset);
        info!("current phase bias = {}", offset);
    }

    /*
    foc.set_run_mode(RunMode::Impedance);
    foc.set_target_angle(Radian::new(0.0));
    foc.set_target_velocity(RadianPerSecond(0.0));
    foc.set_target_torque(0.0);
    foc.set_spring(4.0);
    foc.set_damping(0.0);

    foc.set_run_mode(RunMode::Angle);
    foc.set_target_angle(Radian::new(0.0));

    foc.set_run_mode(RunMode::Velocity);
    foc.set_target_velocity(RadianPerSecond(40.0));
    */
    foc.set_run_mode(RunMode::Torque);
    foc.set_target_torque(0.5);

    foc.enable();

    let command_channel = COMMAND_CHANNEL.receiver();
    let status_channel = STATUS_CHANNEL.sender();

    let mut last_logged_at = Instant::from_secs(0);
    let mut count: usize = 0;

    let mut monitor_period: u8 = 100;
    let mut monitor_tick: u8 = 0;

    loop {
        count += 1;

        if let Ok(command) = command_channel.try_receive() {
            match command {
                Command::Enable => foc.enable(),
                Command::Stop => foc.stop(),
                Command::SetRunMode(m) => foc.set_run_mode(m),
                Command::SetAngle(p) => foc.set_target_angle(Radian::new(p)),
                Command::SetVelocity(v) => foc.set_target_velocity(RadianPerSecond(v)),
                Command::SetTorque(t) => foc.set_target_torque(t),
                Command::SetSpeedLimit(v) => foc.set_velocity_limit(RadianPerSecond(v)),
                Command::SetCurrentLimit(i) => foc.set_current_limit(i),
                Command::SetTorqueLimit(t) => foc.set_torque_limit(t),

                Command::SetCurrentKp(kp) => foc.set_current_kp(kp),
                Command::SetCurrentKi(ki) => foc.set_current_ki(ki),

                Command::SetVelocityKp(kp) => foc.set_velocity_kp(kp),
                Command::SetVelocityKi(ki) => foc.set_velocity_ki(ki),
                Command::SetVelocityGain(tf) => foc.set_velocity_gain(tf),
                Command::SetAngleKp(kp) => foc.set_angle_kp(kp),
                Command::SetSpring(k) => foc.set_spring(k),
                Command::SetDamping(b) => foc.set_damping(b),

                Command::SetMonitorInterval(period) => monitor_period = period,
                _ => (),
            }
        }
        match sensor.read_async().await {
            Ok(reading) => {
                let (ia_raw, ib_raw, ic_raw) = p_adc.read();
                let (ia, ib, ic) = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);

                match foc.get_duty_cycle(
                    &reading,
                    ia - current_offset.a,
                    ib - current_offset.b,
                    ic - current_offset.c,
                ) {
                    Ok(duty) => {
                        driver.run(duty);

                        if let Some(state) = foc.state {
                            monitor_tick += 1;
                            if monitor_tick >= monitor_period {
                                let motor_status = MotorStatus {
                                    raw_angle: f32_to_u16(state.angle.angle, -100.0, 100.0),
                                    raw_velocity: f32_to_u16(
                                        state.filtered_velocity.0,
                                        -100.0,
                                        100.0,
                                    ),
                                    raw_torque: f32_to_u16(state.i_q, -10.0, 10.0),
                                    raw_temperature: 0x00,
                                };
                                status_channel.send(motor_status).await;
                                monitor_tick = 0;
                            }
                        }

                        let now = Instant::now();
                        if let Some(log_after) =
                            last_logged_at.checked_add(Duration::from_millis(100))
                            && now > log_after
                        {
                            let second_since_last_log =
                                (now - last_logged_at).as_micros() as f32 / 1e6;

                            last_logged_at = now;

                            if let Some(state) = foc.state {
                                info!(
                                    "target: a = {}, v = {}, i_ref = {}",
                                    state.target_angle, state.target_velocity, state.i_ref,
                                );

                                info!(
                                    "a = {}, v = {}, iq = {}, v_q = {}",
                                    reading.angle.angle,
                                    state.filtered_velocity.0,
                                    state.i_q,
                                    state.v_q,
                                );
                            }

                            info!("loop = {} Hz", count as f32 / second_since_last_log);
                            count = 0;
                        };
                    }
                    Err(e) => match e {
                        foc::pwm::FocError::AlignError => error!("AlignError"),
                        foc::pwm::FocError::CalculationError => error!("CalculationError"),
                        foc::pwm::FocError::InvalidParameters => error!("InvalidParam"),
                    },
                }
            }
            Err(e) => error!("Failed to read from sensor: {:?}", e),
        }

        Timer::after(Duration::from_micros(1)).await;
    }
}

#[embassy_executor::task]
async fn can_task(p_can: Can<'static>) {
    let (mut can_tx, mut can_rx, _properties) = p_can.split();
    let mut can_id: u8 = 0x0F;
    let command_sender = COMMAND_CHANNEL.sender();
    let status_receiver = STATUS_CHANNEL.receiver();
    let mut status: Option<MotorStatus> = None;
    let mut host_id: Option<u8> = Some(0);

    loop {
        loop {
            if let Ok(s) = status_receiver.try_receive()
                && let Some(h) = host_id
            {
                status = Some(s);
                let message = StatusMessage {
                    motor_can_id: can_id,
                    host_can_id: h,
                    motor_status: s,
                };
                if let Ok(frame) = message.try_into() {
                    can_tx.write(&frame).await;
                }
            } else {
                break;
            }
        }
        match can_rx.read().with_timeout(Duration::from_millis(1)).await {
            Ok(Ok(m)) => {
                if let Ok(message) = can_message::CommandMessage::try_from(m.frame) {
                    if message.motor_can_id != can_id {
                        continue;
                    }
                    // TODO: send request to the main FOC task
                    match message.command {
                        can_message::Command::SetCanId(new_can_id) => {
                            can_id = new_can_id;
                        }
                        can_message::Command::RequestStatus(host_can_id) => {
                            host_id = Some(host_can_id);
                            if let Some(s) = status {
                                let message = StatusMessage {
                                    motor_can_id: can_id,
                                    host_can_id,
                                    motor_status: s,
                                };
                                if let Ok(frame) = message.try_into() {
                                    match can_tx
                                        .write(&frame)
                                        .with_timeout(Duration::from_millis(50))
                                        .await
                                    {
                                        Ok(Some(f)) => {
                                            warn!("CAN tx error: {:?}", f);
                                        }
                                        Ok(None) => (),
                                        Err(e) => {
                                            warn!("CAN tx timeout: {:?}", e);
                                        }
                                    }
                                }
                            }
                        }
                        cmd => command_sender.send(cmd).await,
                    };
                } else {
                    let id = match m.frame.header().id() {
                        Id::Standard(x) => x.as_raw() as u32,
                        Id::Extended(x) => x.as_raw(),
                    };
                    warn!("parse failed: {}#{}", id, m.frame.data());
                }
            }
            _ => Timer::after(Duration::from_micros(1)).await,
        }
    }
}

bind_interrupts!(
    struct Irqs {
        I2C1_EV => stm32_i2c::EventInterruptHandler<peripherals::I2C1>;
        I2C1_ER => stm32_i2c::ErrorInterruptHandler<peripherals::I2C1>;
        FDCAN1_IT1 => can::IT1InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
        FDCAN1_IT0 => can::IT0InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    clock::set_clock(&mut config);
    let p = embassy_stm32::init(config);
    clock::print_clock_info(&p.RCC);

    let mut spi_config = stm32_spi::Config::default();
    spi_config.miso_pull = gpio::Pull::Up;
    spi_config.mode = stm32_spi::MODE_1;
    spi_config.bit_order = stm32_spi::BitOrder::MsbFirst;
    spi_config.frequency = mhz(1);
    spi_config.gpio_speed = gpio::Speed::VeryHigh;
    let cs_out = gpio::Output::new(p.PB6, gpio::Level::High, gpio::Speed::VeryHigh);

    {
        let p_spi = stm32_spi::Spi::new(
            p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH4, spi_config,
        );
        *(SPI.lock().await) = Some(p_spi);
    }

    let mut can_conf = can::CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    can_conf.set_bitrate(1_000_000);
    can_conf.properties().set_extended_filter(
        can::filter::ExtendedFilterSlot::_0,
        can::filter::ExtendedFilter::accept_all_into_fifo1(),
    );
    let p_can = can_conf.into_normal_mode();

    let drvoff_pin = gpio::Output::new(p.PB3, gpio::Level::High, gpio::Speed::Medium);
    let cs_drv = gpio::Output::new(p.PB5, gpio::Level::High, gpio::Speed::VeryHigh);

    let mut p_adc = adc::DummyAdc::new(p.ADC1, p.PA0, p.PA1, p.PC2);
    p_adc.calibrate();
    p_adc.initialize(1, 2, 8);

    let mut timer = pwm::Pwm6Timer::new(p.TIM1, p.PA8, p.PA9, p.PA10, p.PB13, p.PB14, p.PB15);
    timer.initialize((1 << 12) - 1);
    info!("Timer frequency: {}", timer.get_frequency());

    unwrap!(spawner.spawn(foc_task(&SPI, cs_out, cs_drv, drvoff_pin, timer, p_adc)));
    //unwrap!(spawner.spawn(foc_sensorless_task(&SPI, cs_out, cs_drv, drvoff_pin, timer)));
    unwrap!(spawner.spawn(can_task(p_can)));
}

fn f32_to_u16(x: f32, x_min: f32, x_max: f32) -> u16 {
    let x = x.min(x_max).max(x_min);
    let span = x_max - x_min;
    ((x - x_min) / span * (u16::MAX as f32)) as u16
}
