#![no_std]
#![no_main]
mod adc;
mod as5047p;
mod bldc_driver;
mod can;
mod clock;
mod cordic;
mod current_tuning;
mod drv8316;
mod gm2804;
mod gm3506;
mod pwm;

use foc::current::PhaseCurrent;

use crate::{
    adc::AdcSelector,
    as5047p::As5047P,
    bldc_driver::PwmDriver,
    can::{convert_response_message, parse_command_frame},
    cordic::{initialize_cordic, sincos},
    current_tuning::calculate_current_pi,
    drv8316::Drv8316,
};

use crate::gm3506 as motor;

use can_message::message::{
    Command, FeedbackType, MotorCurrent, MotorStatus, ParameterIndex, ParameterValue, ResponseBody,
    ResponseMessage,
};
use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    Config, bind_interrupts,
    can::{self as stm32_can, CanRx, CanTx},
    gpio, i2c as stm32_i2c,
    interrupt::{self, InterruptExt},
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
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, Sender},
};
use embassy_time::{Duration, Instant, Timer, WithTimeout};

use embedded_can::Id;
use foc::{
    angle_input::AngleInput,
    controller::{FocController, FocState, OutputLimit, RunMode},
    pwm_output::DutyCycle3Phase,
};
use foc::{
    controller::{clarke_transform, park_transform},
    pwm_output::PwmOutput,
};
use libm::atan2f;

static COMMAND_CHANNEL: Channel<ThreadModeRawMutex, Command, 3> = Channel::new();
static RESPONSE_CHANNEL: Channel<ThreadModeRawMutex, ResponseBody, 64> = Channel::new();

type SpiMutex = embassy_sync::mutex::Mutex<
    embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    Option<stm32_spi::Spi<'static, Async>>,
>;

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
        let measured = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
        ia_avg += measured.a;
        ib_avg += measured.b;
        ic_avg += measured.c;
        Timer::after_millis(1).await;
    }
    ia_avg /= n as f32;
    ib_avg /= n as f32;
    ic_avg /= n as f32;
    PhaseCurrent::new(ia_avg, ib_avg, ic_avg)
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
    driver.run(DutyCycle3Phase::zero());
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
    let mut phase = DutyCycle3Phase::zero();
    for i in 0..100 {
        phase.t1 = align_ratio * (i as f32) / 100.0;
        driver.run(phase);
        Timer::after_millis(1).await;
    }
    Timer::after_millis(100).await;
    let c = get_average_adc(p_adc, csa_gain, 20).await - offset;
    let phase_1 = max_index(c.a, c.b, c.c)?;

    phase = DutyCycle3Phase::zero();
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

    phase = DutyCycle3Phase::zero();
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

async fn align_current<'a, T, TIM, Fsincos>(
    p_adc: &mut adc::DummyAdc<T>,
    driver: &mut PwmDriver<'a, TIM>,
    csa_gain: drv8316::CsaGain,
    sensor: &mut As5047P<'a>,
    foc: &FocController<Fsincos>,
) -> Option<f32>
where
    T: stm32_adc::Instance + AdcSelector,
    TIM: AdvancedInstance4Channel,
    Fsincos: Fn(f32) -> (f32, f32),
{
    // assumption: mechanical angle and voltage (electrical) angle are aligned
    let a_duty = 0.2;

    // start with zero voltage
    let mut duty = DutyCycle3Phase::zero();
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
        let measured = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
        let mapped = foc.normalize_current(measured);
        if let Ok(reading) = sensor.read_async().await {
            let angle = reading.angle;
            let e_angle = foc.to_electrical_angle(angle);

            let (i_alpha, i_beta) = clarke_transform(mapped.a, mapped.b, mapped.c);
            let (i_q, i_d) = park_transform(i_alpha, i_beta, e_angle, sincos);
            let i_angle = atan2f(i_q, i_d);
            avg_angle += i_angle - e_angle;
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

fn build_motor_status(state: FocState) -> MotorStatus {
    MotorStatus {
        angle: state.angle,
        velocity: state.filtered_velocity,
        torque: state.i_q,
        temperature: 0.0,
    }
}

#[embassy_executor::task]
async fn foc_task(
    use_current_sensing: bool,
    p_spi: &'static SpiMutex,
    cs_sensor_pin: gpio::Output<'static>,
    cs_drv_pin: gpio::Output<'static>,
    mut drvoff_pin: gpio::Output<'static>,
    pwm_timer: pwm::Pwm6Timer<'static, TIM1, PA8, PA9, PA10, PB13, PB14, PB15>,
    mut p_adc: adc::DummyAdc<ADC1>,
) {
    let csa_gain = drv8316::CsaGain::Gain0_3V;
    let slew_rate = drv8316::SlewRate::Rate200V;
    let psu_voltage = 16.0;
    let align_voltage: f32 = 3.0;

    drvoff_pin.set_high();
    let mut driver = PwmDriver::new(pwm_timer.timer);
    let mut sensor = As5047P::new(p_spi, cs_sensor_pin);
    let mut gate_driver = Drv8316::new(p_spi, cs_drv_pin, drvoff_pin);
    gate_driver.initialize(csa_gain, slew_rate).await;

    let mut foc = {
        if use_current_sensing {
            FocController::new(
                motor::SETUP,
                psu_voltage,
                motor::CURRENT_PID,
                motor::ANGLE_PID_CS,
                motor::VELOCITY_PID_CS,
                OutputLimit {
                    max_value: 16.0,
                    ramp: 1000.0,
                },
                sincos,
            )
        } else {
            FocController::new(
                motor::SETUP,
                psu_voltage,
                motor::CURRENT_PID,
                motor::ANGLE_PID_NOCS,
                motor::VELOCITY_PID_NOCS,
                OutputLimit {
                    max_value: 16.0,
                    ramp: 1000.0,
                },
                sincos,
            )
        }
    };

    if !use_current_sensing {
        foc.disable_current_sensing();
    } else {
        foc.enable_current_sensing();
    }

    gate_driver.turn_on();
    if use_current_sensing {
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
            gate_driver.turn_off();
            return;
        }
        foc.set_current_offset(current_offset);
    }

    if let Err(e) = sensor.initialize().await {
        error!("Sensor initialization failed, {:?}", e);
        gate_driver.turn_off();
        return;
    }

    let mut read_sensor = {
        let s = &mut sensor;
        async || s.read_async().await.map_err(|e| error!("{:?}", e)).unwrap()
    };
    let set_motor = |d: DutyCycle3Phase| driver.run(d);
    let wait_seconds = async |s: f32| Timer::after(Duration::from_millis((s * 1e3) as u64)).await;

    if foc
        .align_sensor(align_voltage, &mut read_sensor, set_motor, wait_seconds)
        .await
        .is_err()
    {
        error!("Sensor align failed");
        gate_driver.turn_off();
        return;
    }

    if use_current_sensing
        && let Some(offset) =
            align_current(&mut p_adc, &mut driver, csa_gain, &mut sensor, &foc).await
    {
        foc.set_current_phase_bias(offset);
        info!("current phase bias = {}", offset);

        let impedance = current_tuning::find_motor_impedance(
            &mut foc,
            &mut driver,
            &mut p_adc,
            csa_gain,
            &mut sensor,
        )
        .await;
        let current_gain = calculate_current_pi(impedance, 500.0);
        foc.set_current_kp(current_gain.p);
        foc.set_current_ki(current_gain.i);
    }

    foc.set_run_mode(RunMode::Torque);
    foc.set_target_torque(0.0);
    foc.enable();

    let command_receiver = COMMAND_CHANNEL.receiver();
    let response_sender = RESPONSE_CHANNEL.sender();

    let mut feedback_type = FeedbackType::Status;
    let mut feedback_period: u8 = 10;
    let mut feedback_tick: u8 = 0;
    let mut check_count: usize = 0;
    let mut last_logged_at = Instant::now();
    let mut phase_current = PhaseCurrent::new(0.0, 0.0, 0.0);

    loop {
        if let Ok(command) = command_receiver.try_receive() {
            match command {
                Command::SetFeedbackInterval(period) => feedback_period = period,
                Command::SetFeedbackType(typ) => feedback_type = typ,
                _ => {
                    let _ = handle_command(&command, &mut foc, &response_sender)
                        .with_timeout(Duration::from_micros(5))
                        .await;
                }
            }
        }
        match sensor.read_async().await {
            Ok(reading) => {
                if use_current_sensing {
                    let (ia_raw, ib_raw, ic_raw) = p_adc.read();
                    phase_current = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
                }

                match foc.get_duty_cycle(&reading, phase_current) {
                    Ok(duty) => {
                        driver.run(duty);

                        feedback_tick += 1;
                        if feedback_tick >= feedback_period {
                            match feedback_type {
                                FeedbackType::Status => {
                                    let motor_status = build_motor_status(foc.state);
                                    let _ = response_sender
                                        .send(ResponseBody::MotorStatus(motor_status))
                                        .with_timeout(Duration::from_micros(1))
                                        .await;
                                }
                                FeedbackType::Current => {
                                    let motor_current = MotorCurrent {
                                        i_q: foc.state.i_q,
                                        i_d: foc.state.i_d,
                                    };
                                    let _ = response_sender
                                        .send(ResponseBody::MotorCurrent(motor_current))
                                        .with_timeout(Duration::from_micros(1))
                                        .await;
                                }
                            };
                            feedback_tick = 0;
                        }

                        check_count += 1;
                        if check_count.is_multiple_of(5_000) {
                            let now = Instant::now();
                            if let Some(dt) = now.checked_duration_since(last_logged_at) {
                                log_state(&foc.state, &dt, check_count);
                                check_count = 0;
                                last_logged_at = now;
                            }
                        }
                    }
                    Err(e) => match e {
                        foc::pwm::FocError::AlignError => error!("AlignError"),
                        foc::pwm::FocError::CalculationError => error!("CalculationError"),
                        foc::pwm::FocError::InvalidParameters => error!("InvalidParam"),
                    },
                }
            }
            Err(as5047p::Error::CommandFrame) => {
                if sensor.read_and_reset_error_flag().await.is_err() {
                    error!("CommandFrame error correction failed");
                }
            }
            Err(as5047p::Error::Parity) => {
                if sensor.read_and_reset_error_flag().await.is_err() {
                    error!("Parity error correction failed");
                }
            }
            Err(e) => error!("Failed to read from sensor: {:?}", e),
        }
    }
}

fn log_state(state: &FocState, dt: &Duration, check_count: usize) {
    let dt_seconds = (dt.as_micros() as f32) / 1e6;
    let freq = (check_count as f32) / dt_seconds;
    info!(
        "MEASURED: a={}, v={}, t={}, {} Hz",
        state.angle, state.filtered_velocity, state.v_q, freq
    );
}

#[embassy_executor::task]
async fn can_tx_task(mut can_tx: CanTx<'static>) {
    let can_id: u8 = 0x0F;
    let response_receiver = RESPONSE_CHANNEL.receiver();
    let host_id: Option<u8> = Some(0);

    loop {
        let r = response_receiver.receive().await;
        if let Some(h) = host_id {
            let message = ResponseMessage::new(can_id, h, r);
            if let Ok(frame) = convert_response_message(message) {
                can_tx.write(&frame).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn can_rx_task(mut can_rx: CanRx<'static>) {
    let mut can_id: u8 = 0x0F;
    let command_sender = COMMAND_CHANNEL.sender();

    loop {
        if let Ok(m) = can_rx.read().await {
            if let Ok(message) = parse_command_frame(m.frame) {
                if message.motor_can_id != can_id {
                    continue;
                }
                match message.command {
                    Command::SetCanId(new_can_id) => {
                        can_id = new_can_id;
                    }
                    cmd => {
                        if command_sender.try_send(cmd).is_err() {
                            warn!("failed to send cmd");
                        }
                    }
                };
            } else {
                let id = match m.frame.header().id() {
                    Id::Standard(x) => x.as_raw() as u32,
                    Id::Extended(x) => x.as_raw(),
                };
                warn!("parse failed: {}#{}", id, m.frame.data());
            }
        }
    }
}

async fn handle_command<Fsincos: Fn(f32) -> (f32, f32)>(
    command: &Command,
    foc: &mut FocController<Fsincos>,
    response_sender: &Sender<'static, ThreadModeRawMutex, ResponseBody, 64>,
) {
    match command {
        Command::Enable => foc.enable(),
        Command::Stop => foc.stop(),

        Command::SetParameter(ParameterValue::RunMode(m)) => {
            info!("received runmode {}", *m as u8);
            foc.set_run_mode(decode_run_mode(m));
        }
        Command::SetParameter(ParameterValue::AngleRef(p)) => foc.set_target_angle(*p),
        Command::SetParameter(ParameterValue::SpeedRef(v)) => foc.set_target_velocity(*v),
        Command::SetParameter(ParameterValue::IqRef(t)) => foc.set_target_torque(*t),
        Command::SetParameter(ParameterValue::SpeedLimit(v)) => foc.set_velocity_limit(*v),
        Command::SetParameter(ParameterValue::CurrentLimit(i)) => foc.set_current_limit(*i),
        Command::SetParameter(ParameterValue::TorqueLimit(t)) => foc.set_torque_limit(*t),

        Command::SetParameter(ParameterValue::CurrentKp(kp)) => foc.set_current_kp(*kp),
        Command::SetParameter(ParameterValue::CurrentKi(ki)) => foc.set_current_ki(*ki),

        Command::SetParameter(ParameterValue::SpeedKp(kp)) => foc.set_velocity_kp(*kp),
        Command::SetParameter(ParameterValue::SpeedKi(ki)) => foc.set_velocity_ki(*ki),
        Command::SetParameter(ParameterValue::Iq(f)) => foc.set_current_filter(*f),
        Command::SetParameter(ParameterValue::AngleKp(kp)) => foc.set_angle_kp(*kp),

        Command::SetParameter(ParameterValue::Spring(k)) => foc.set_spring(*k),
        Command::SetParameter(ParameterValue::Damping(b)) => foc.set_damping(*b),
        Command::SetParameter(ParameterValue::VqRef(v)) => {
            info!("received target Vq {}", v);
            foc.set_target_voltage(*v);
        }

        Command::GetParameter(p) => {
            info!("parameter requested {}", *p as u16);
            let pv = match p {
                ParameterIndex::RunMode => ParameterValue::RunMode(encode_run_mode(&foc.mode)),
                ParameterIndex::IqRef => ParameterValue::IqRef(foc.state.i_ref),
                ParameterIndex::Angle => ParameterValue::Angle(foc.state.angle),
                ParameterIndex::AngleRef => ParameterValue::AngleRef(foc.target.angle),
                ParameterIndex::Speed => ParameterValue::Speed(foc.state.filtered_velocity),
                ParameterIndex::SpeedRef => ParameterValue::SpeedRef(foc.target.velocity),
                ParameterIndex::SpeedLimit => {
                    ParameterValue::SpeedLimit(foc.velocity_limit.unwrap_or(0.0))
                }
                ParameterIndex::TorqueLimit => {
                    ParameterValue::TorqueLimit(foc.torque_limit.unwrap_or(0.0))
                }
                ParameterIndex::Iq => ParameterValue::Iq(foc.state.i_q),

                ParameterIndex::AngleKp => ParameterValue::AngleKp(foc.angle_pid.gains.p),
                ParameterIndex::SpeedKp => ParameterValue::SpeedKp(foc.velocity_pid.gains.p),
                ParameterIndex::SpeedKi => ParameterValue::SpeedKi(foc.velocity_pid.gains.i),

                ParameterIndex::CurrentKp => ParameterValue::CurrentKp(foc.current_q_pid.gains.p),
                ParameterIndex::CurrentKi => ParameterValue::CurrentKi(foc.current_q_pid.gains.i),
                ParameterIndex::CurrentFilter => {
                    ParameterValue::CurrentFilter(foc.current_q_filter.time_constant)
                }
                ParameterIndex::CurrentLimit => {
                    ParameterValue::CurrentLimit(foc.current_limit.unwrap_or(0.0))
                }
                ParameterIndex::Spring => ParameterValue::Spring(foc.target.spring),
                ParameterIndex::Damping => ParameterValue::Damping(foc.target.damping),
                ParameterIndex::VqRef => ParameterValue::Damping(foc.target.voltage),
            };
            let body = ResponseBody::ParameterValue(pv);
            response_sender.send(body).await;
        }
        _ => (),
    }
}

fn decode_run_mode(m: &can_message::message::RunMode) -> foc::controller::RunMode {
    match m {
        can_message::message::RunMode::Impedance => foc::controller::RunMode::Impedance,
        can_message::message::RunMode::Angle => foc::controller::RunMode::Angle,
        can_message::message::RunMode::Velocity => foc::controller::RunMode::Velocity,
        can_message::message::RunMode::Torque => foc::controller::RunMode::Torque,
        can_message::message::RunMode::Voltage => foc::controller::RunMode::Voltage,
    }
}

fn encode_run_mode(m: &foc::controller::RunMode) -> can_message::message::RunMode {
    match m {
        foc::controller::RunMode::Impedance => can_message::message::RunMode::Impedance,
        foc::controller::RunMode::Angle => can_message::message::RunMode::Angle,
        foc::controller::RunMode::Velocity => can_message::message::RunMode::Velocity,
        foc::controller::RunMode::Torque => can_message::message::RunMode::Torque,
        foc::controller::RunMode::Voltage => can_message::message::RunMode::Voltage,
    }
}

bind_interrupts!(
    struct Irqs {
        I2C1_EV => stm32_i2c::EventInterruptHandler<peripherals::I2C1>;
        I2C1_ER => stm32_i2c::ErrorInterruptHandler<peripherals::I2C1>;
        FDCAN1_IT1 => stm32_can::IT1InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
        FDCAN1_IT0 => stm32_can::IT0InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    clock::set_clock(&mut config);
    initialize_cordic();
    let p = embassy_stm32::init(config);
    clock::print_clock_info(&p.RCC);

    embassy_stm32::interrupt::TIM1_UP_TIM16.set_priority(interrupt::Priority::P0);
    embassy_stm32::interrupt::ADC1_2.set_priority(interrupt::Priority::P1);
    embassy_stm32::interrupt::FDCAN1_IT0.set_priority(interrupt::Priority::P7);
    embassy_stm32::interrupt::FDCAN1_IT1.set_priority(interrupt::Priority::P7);

    let mut spi_config = stm32_spi::Config::default();
    spi_config.miso_pull = gpio::Pull::Up;
    spi_config.mode = stm32_spi::MODE_1;
    spi_config.bit_order = stm32_spi::BitOrder::MsbFirst;
    spi_config.frequency = mhz(10);
    spi_config.gpio_speed = gpio::Speed::VeryHigh;
    let cs_out = gpio::Output::new(p.PB6, gpio::Level::High, gpio::Speed::VeryHigh);

    {
        let p_spi = stm32_spi::Spi::new(
            p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH4, spi_config,
        );
        *(SPI.lock().await) = Some(p_spi);
    }

    let mut can_conf = stm32_can::CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    can_conf.set_bitrate(1_000_000);
    can_conf.properties().set_extended_filter(
        stm32_can::filter::ExtendedFilterSlot::_0,
        stm32_can::filter::ExtendedFilter::accept_all_into_fifo1(),
    );
    let p_can = can_conf.into_normal_mode();
    let (can_tx, can_rx, _properties) = p_can.split();

    let drvoff_pin = gpio::Output::new(p.PB12, gpio::Level::High, gpio::Speed::Medium);
    let cs_drv = gpio::Output::new(p.PB5, gpio::Level::High, gpio::Speed::VeryHigh);

    let mut p_adc = adc::DummyAdc::new(p.ADC1, p.PA0, p.PA1, p.PC2);
    p_adc.calibrate();
    p_adc.initialize(1, 2, 8);

    let mut timer = pwm::Pwm6Timer::new(p.TIM1, p.PA8, p.PA9, p.PA10, p.PB13, p.PB14, p.PB15);
    timer.initialize((1 << 12) - 1);
    info!("Timer frequency: {}", timer.get_frequency());

    unwrap!(spawner.spawn(foc_task(
        true, &SPI, cs_out, cs_drv, drvoff_pin, timer, p_adc
    )));

    unwrap!(spawner.spawn(can_rx_task(can_rx)));
    unwrap!(spawner.spawn(can_tx_task(can_tx)));
}
