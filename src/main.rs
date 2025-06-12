#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Timer};

#[embassy_executor::task]
async fn blinker(mut led: Output<'static>, interval: Duration) {
    loop {
        led.set_high();
        Timer::after(interval).await;
        led.set_low();
        Timer::after(interval).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let led = Output::new(p.PA5, Level::Low, Speed::Medium);
    unwrap!(spawner.spawn(blinker(led, Duration::from_millis(300))));
}
