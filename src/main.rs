#![no_std]
#![no_main]
mod as5600;
mod i2c;

use crate::as5600::MagnetStatus;

use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::time::khz;
use embassy_time::{Duration, Timer};

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
async fn i2c_task(mut p_i2c: I2c<'static, embassy_stm32::mode::Blocking>) {
    match as5600::set_digital_output_mode(&mut p_i2c) {
        Ok(_) => info!("Digital output mode set successfully"),
        Err(Error::Timeout) => error!("I2C operation timed out"),
        Err(e) => error!("Failed to set digital output mode: {:?}", e),
    }
    loop {
        match as5600::read_magnet_status(&mut p_i2c) {
            Ok(MagnetStatus::Ok) => info!("Magnet is ok"),
            Ok(status) => warn!("Magnet status: {:?}", status),
            Err(Error::Timeout) => error!("I2C operation timed out"),
            Err(e) => error!("Failed to read magnet status: {:?}", e),
        }

        match as5600::read_raw_angle(&mut p_i2c) {
            Ok(angle) => info!("Magnet angle: {}", angle),
            Err(Error::Timeout) => error!("I2C operation timed out"),
            Err(e) => error!("Failed to read raw angle: {:?}", e),
        }

        Timer::after(Duration::from_millis(200)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = embassy_stm32::Config::default();
    let p = embassy_stm32::init(config);

    let led = Output::new(p.PA5, Level::Low, Speed::Medium);
    let i2c_config = embassy_stm32::i2c::Config::default();
    let p_i2c = I2c::new_blocking(p.I2C1, p.PB8, p.PB9, khz(100), i2c_config);

    unwrap!(spawner.spawn(blinker(led, Duration::from_millis(300))));
    unwrap!(spawner.spawn(i2c_task(p_i2c)));
}
