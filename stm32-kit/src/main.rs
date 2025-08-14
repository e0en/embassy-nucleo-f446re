#![no_std]
#![no_main]
mod adc;
mod as5047p;
mod bldc_driver;
mod can_message;
mod clock;
mod pwm;

use crate::{
    as5047p::As5047P,
    bldc_driver::PwmDriver,
    can_message::{Command, MotorStatus, StatusMessage},
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
};
use embassy_stm32::{
    peripherals::{PA8, PA9, PA10, PB13, PB14, PB15},
    spi as stm32_spi,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};

use embedded_can::Id;
use foc::pwm_output::PwmOutput;
use foc::{
    angle_input::AngleInput,
    controller::{Direction, FocController, RunMode},
    pwm_output::DutyCycle3Phase,
    units::{Radian, RadianPerSecond, Second},
};

static COMMAND_CHANNEL: Channel<ThreadModeRawMutex, Command, 1> = Channel::new();
static STATUS_CHANNEL: Channel<ThreadModeRawMutex, MotorStatus, 1> = Channel::new();

type SpiMutex = embassy_sync::mutex::Mutex<
    embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    Option<stm32_spi::Spi<'static, Async>>,
>;

static SPI: SpiMutex = SpiMutex::new(None);

#[embassy_executor::task]
async fn foc_task(
    p_spi: &'static SpiMutex,
    cs_pin: gpio::Output<'static>,
    pwm_timer: pwm::Pwm6Timer<'static, TIM1, PA8, PA9, PA10, PB13, PB14, PB15>,
    _p_adc: adc::DummyAdc<ADC1>,
) {
    let mut driver = PwmDriver::new(pwm_timer.timer);
    let mut sensor = As5047P::new(p_spi, cs_pin);

    match sensor.initialize().await {
        Ok(_) => info!("Sensor initialized"),
        Err(e) => error!("Sensor initialization failed, {:?}", e),
    }

    let mut foc = FocController::new(
        16.0,
        11,
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

    let mut read_sensor = {
        let s = &mut sensor;
        async || s.read_async().await.unwrap()
    };
    let set_motor = |d: DutyCycle3Phase| driver.run(d);
    let wait_seconds =
        async |s: Second| Timer::after(Duration::from_millis((s.0 * 1e3) as u64)).await;

    match foc
        .align_sensor(4.0, &mut read_sensor, set_motor, wait_seconds)
        .await
    {
        Ok(_) => {
            info!(
                "Sensor aligned {} {}",
                foc.bias_angle.0,
                foc.sensor_direction == Direction::Clockwise
            );
        }
        Err(_) => {
            error!("Sensor align failed");
            return;
        }
    };

    foc.set_target_angle(Radian(0.0));
    foc.set_target_velocity(RadianPerSecond(0.0));
    foc.set_target_torque(0.0);
    foc.set_spring(4.0);
    foc.set_damping(-0.09);
    foc.set_run_mode(RunMode::Impedance);
    foc.enable();

    let command_channel = COMMAND_CHANNEL.receiver();
    let _status_channel = STATUS_CHANNEL.sender();

    let mut last_logged_at = Instant::from_secs(0);
    let mut count: usize = 0;
    loop {
        count += 1;

        let (ia, ib, ic) = _p_adc.read();

        if let Ok(command) = command_channel.try_receive() {
            match command {
                Command::Enable => foc.enable(),
                Command::Stop => foc.stop(),
                Command::SetRunMode(m) => foc.set_run_mode(m),
                Command::SetAngle(p) => foc.set_target_angle(Radian(p)),
                Command::SetVelocity(v) => foc.set_target_velocity(RadianPerSecond(v)),
                Command::SetTorque(t) => foc.set_target_torque(t),
                Command::SetSpeedLimit(v) => foc.set_velocity_limit(RadianPerSecond(v)),
                Command::SetCurrentLimit(i) => foc.set_current_limit(i),
                Command::SetTorqueLimit(t) => foc.set_torque_limit(t),
                Command::SetVelocityKp(kp) => foc.set_velocity_kp(kp),
                Command::SetVelocityKi(ki) => foc.set_velocity_ki(ki),
                Command::SetVelocityGain(tf) => foc.set_velocity_gain(tf),
                Command::SetAngleKp(kp) => foc.set_angle_kp(kp),
                Command::SetSpring(k) => foc.set_spring(k),
                Command::SetDamping(b) => foc.set_damping(b),
                _ => (),
            }
        }
        match sensor.read_async().await {
            Ok(reading) => match foc.get_duty_cycle(&reading) {
                Ok(duty) => {
                    driver.run(duty);

                    let now = Instant::now();
                    if (now - last_logged_at) > Duration::from_millis(250) {
                        let second_since_last_log = (now - last_logged_at).as_micros() as f32 / 1e6;

                        last_logged_at = now;

                        info!("{}, {}, {}", ia, ib, ic);
                        if let Some(state) = foc.state {
                            info!(
                                "a = {}, vf = {}, err = {}, dt = {}, v_ref = {}",
                                state.angle_error.map(|x| x.0),
                                state.filtered_velocity.0,
                                state.velocity_error.0,
                                state.dt.0,
                                state.v_ref,
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
async fn can_task(p_can: Can<'static>) {
    let (mut can_tx, mut can_rx, _properties) = p_can.split();
    let mut can_id: u8 = 0x0F;
    let command_sender = COMMAND_CHANNEL.sender();
    let status_receiver = STATUS_CHANNEL.receiver();
    let mut status: Option<MotorStatus> = None;
    loop {
        for _ in 0..3 {
            if let Ok(s) = status_receiver.try_receive() {
                status = Some(s);
            } else {
                break;
            }
        }
        info!("no more status message");
        match can_rx.read().await {
            Ok(m) => {
                if let Ok(message) = can_message::CommandMessage::try_from(m.frame) {
                    if message.motor_can_id != can_id {
                        continue;
                    }
                    // TODO: send request to the main FOC task
                    match message.command {
                        can_message::Command::SetCanId(new_can_id) => {
                            can_id = new_can_id;
                        }
                        can_message::Command::RequestStatus(host_id) => {
                            if let Some(s) = status {
                                let message = StatusMessage {
                                    motor_can_id: can_id,
                                    host_can_id: host_id,
                                    motor_status: s,
                                };
                                if let Ok(frame) = message.try_into() {
                                    can_tx.write(&frame).await;
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
            Err(_) => Timer::after(Duration::from_micros(1)).await,
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
    spi_config.miso_pull = gpio::Pull::None;
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

    let _drvoff_pin = gpio::Output::new(p.PB3, gpio::Level::Low, gpio::Speed::Medium);
    let _n_sleep_pin = gpio::Input::new(p.PB5, gpio::Pull::None);

    let mut p_adc = adc::DummyAdc::new(p.ADC1, p.PA0, p.PA1, p.PC2);
    p_adc.calibrate();
    p_adc.initialize(1, 2, 8);

    let mut timer = pwm::Pwm6Timer::new(p.TIM1, p.PA8, p.PA9, p.PA10, p.PB13, p.PB14, p.PB15);
    timer.initialize((1 << 12) - 1);
    info!("Timer frequency: {}", timer.get_frequency());

    unwrap!(spawner.spawn(foc_task(&SPI, cs_out, timer, p_adc)));
    unwrap!(spawner.spawn(can_task(p_can)));
}
