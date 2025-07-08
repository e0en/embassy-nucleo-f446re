#![no_std]
#![no_main]
mod as5600;
mod bldc_driver;
mod clock;
mod i2c;
mod pwm;

use crate::{
    as5600::{As5600, MagnetStatus},
    bldc_driver::PwmDriver,
};

use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    Config, bind_interrupts,
    i2c::{Error, I2c, Master},
    peripherals,
    timer::low_level,
};
use embassy_stm32::{i2c as stm32_i2c, peripherals::TIM1};

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::khz;
use embassy_time::{Duration, Instant, Timer};
use foc::pwm_output::PwmOutput;
use foc::{
    angle_input::AngleInput,
    controller::{Direction, FocController},
    pwm_output::DutyCycle3Phase,
    units::Second,
};

#[embassy_executor::task]
async fn blinker(mut led: Output<'static>, delay: Duration) {
    loop {
        led.set_high();
        Timer::after(delay).await;
        led.set_low();
        Timer::after(delay).await;
    }
}

#[embassy_executor::task]
async fn foc_task(
    mut p_i2c: I2c<'static, embassy_stm32::mode::Async, Master>,
    pwm_timer: low_level::Timer<'static, TIM1>,
) {
    let mut driver = PwmDriver::new(pwm_timer);

    let mut sensor = As5600::new();
    sensor.initialize(&mut p_i2c).await.unwrap();

    let mut foc = FocController::new(
        16.0,
        11,
        foc::pid::PID {
            p: 0.0,
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
        let b = &mut p_i2c;
        async || s.read_async(b).await.unwrap()
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

    foc.set_command(foc::controller::Command::Velocity(
        foc::units::RadianPerSecond(core::f32::consts::PI * 2.0),
    ));

    match as5600::set_digital_output_mode(&mut p_i2c).await {
        Ok(_) => info!("Digital output mode set successfully"),
        Err(Error::Timeout) => error!("I2C operation timed out"),
        Err(e) => error!("Failed to set digital output mode: {:?}", e),
    }

    match as5600::read_magnet_status(&mut p_i2c).await {
        Ok(MagnetStatus::Ok) => info!("Magnet is ok"),
        Ok(status) => warn!("Magnet status: {:?}", status),
        Err(Error::Timeout) => error!("I2C operation timed out"),
        Err(e) => error!("Failed to read magnet status: {:?}", e),
    }

    let mut last_logged_at = Instant::from_secs(0);
    let mut count: usize = 0;
    let mut velocity_logs = [0.0; 1000];
    let mut ring_start: usize = 0;
    let mut ring_size: usize = 0;
    loop {
        count += 1;
        match sensor.read_async(&mut p_i2c).await {
            Ok(reading) => {
                if let Ok(duty) = foc.get_duty_cycle(&reading) {
                    driver.run(duty);
                    if let Some(state) = foc.state {
                        let ring_end = (ring_start + ring_size) % 1000;
                        velocity_logs[ring_end] = state.filtered_velocity.0;
                        if ring_size < 1000 {
                            ring_size += 1;
                        } else {
                            ring_start = (ring_start + 1) % 1000;
                        }
                    }

                    let now = Instant::now();
                    if (now - last_logged_at) > Duration::from_millis(50) {
                        let second_since_last_log = (now - last_logged_at).as_micros() as f32 / 1e6;

                        last_logged_at = now;

                        let mut mean = 0.0;
                        let mut stdev = 0.0;

                        if ring_size > 0 {
                            for i in 0..ring_size {
                                let i_ring = (ring_start + i) % 1000;
                                mean += velocity_logs[i_ring];
                            }
                            mean /= ring_size as f32;
                            for i in 0..ring_size {
                                let i_ring = (ring_start + i) % 1000;
                                stdev +=
                                    (velocity_logs[i_ring] - mean) * (velocity_logs[i_ring] - mean);
                            }
                            stdev /= ring_size as f32;
                        }

                        if let Some(state) = foc.state {
                            info!(
                                "vf = {}, err = {}, stdev = {}, dt = {}",
                                state.filtered_velocity.0,
                                state.velocity_error.0,
                                stdev,
                                state.dt.0
                            );
                        }

                        info!("loop = {} Hz", count as f32 / second_since_last_log);
                        count = 0;
                    };
                }
            }
            Err(Error::Timeout) => error!("I2C operation timed out"),
            Err(e) => error!("Failed to read from sensor: {:?}", e),
        }

        Timer::after(Duration::from_micros(1)).await;
    }
}

bind_interrupts!(
    struct Irqs {
        I2C1_EV => stm32_i2c::EventInterruptHandler<peripherals::I2C1>;
        I2C1_ER => stm32_i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    clock::set_clock(&mut config);
    let p = embassy_stm32::init(config);
    clock::print_clock_info(&p.RCC);

    let led = Output::new(p.PA5, Level::Low, Speed::Medium);

    // Initialize I2c
    let i2c_config = embassy_stm32::i2c::Config::default();
    let p_i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH0,
        khz(400),
        i2c_config,
    );

    let mut timer = pwm::create_timer(p.PA8, p.PA9, p.PA10, p.TIM1);
    pwm::initialize(&mut timer);
    timer.set_max_compare_value((1 << 12) - 1);
    info!("Timer frequency: {}", timer.get_frequency());

    unwrap!(spawner.spawn(blinker(led, Duration::from_millis(300))));
    unwrap!(spawner.spawn(foc_task(p_i2c, timer)));
}
