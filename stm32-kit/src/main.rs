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
use embassy_time::{Duration, Timer};
use foc::angle_input::AngleInput;
use foc::pwm_output::{DutyCycle3Phase, PwmOutput};

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
    let mut cycle = 0u32;
    let max_duty = pwm_timer.get_max_compare_value() + 1;

    let mut driver = PwmDriver::new(pwm_timer);

    let mut sensor = As5600::new();
    match as5600::set_digital_output_mode(&mut p_i2c).await {
        Ok(_) => info!("Digital output mode set successfully"),
        Err(Error::Timeout) => error!("I2C operation timed out"),
        Err(e) => error!("Failed to set digital output mode: {:?}", e),
    }
    loop {
        let duty_ratio = (cycle as f32) / (max_duty as f32);
        driver.run(DutyCycle3Phase::new((duty_ratio, duty_ratio, duty_ratio)));
        cycle = (cycle + 1) % max_duty;

        match as5600::read_magnet_status(&mut p_i2c).await {
            Ok(MagnetStatus::Ok) => info!("Magnet is ok"),
            Ok(status) => warn!("Magnet status: {:?}", status),
            Err(Error::Timeout) => error!("I2C operation timed out"),
            Err(e) => error!("Failed to read magnet status: {:?}", e),
        }

        match sensor.read_async(&mut p_i2c).await {
            Ok(reading) => info!(
                "Angle = {}, Velocity = {}, dt = {}",
                reading.angle, reading.velocity, reading.dt
            ),
            Err(Error::Timeout) => error!("I2C operation timed out"),
            Err(e) => error!("Failed to read from sensor: {:?}", e),
        }

        Timer::after(Duration::from_millis(500)).await;
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
