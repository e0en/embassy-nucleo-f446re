#![no_std]
#![no_main]
mod as5600;
mod clock;
mod foc;
mod i2c;

use crate::as5600::MagnetStatus;

use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::i2c as stm32_i2c;
use embassy_stm32::{
    Config, bind_interrupts,
    gpio::OutputType,
    i2c::{Error, I2c, Master},
    peripherals,
    timer::{low_level, simple_pwm::PwmPin},
};

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{peripherals::TIM1, time::khz};
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
async fn i2c_task(mut p_i2c: I2c<'static, embassy_stm32::mode::Async, Master>) {
    match as5600::set_digital_output_mode(&mut p_i2c).await {
        Ok(_) => info!("Digital output mode set successfully"),
        Err(Error::Timeout) => error!("I2C operation timed out"),
        Err(e) => error!("Failed to set digital output mode: {:?}", e),
    }
    loop {
        match as5600::read_magnet_status(&mut p_i2c).await {
            Ok(MagnetStatus::Ok) => info!("Magnet is ok"),
            Ok(status) => warn!("Magnet status: {:?}", status),
            Err(Error::Timeout) => error!("I2C operation timed out"),
            Err(e) => error!("Failed to read magnet status: {:?}", e),
        }

        match as5600::read_raw_angle(&mut p_i2c).await {
            Ok(angle) => info!("Magnet angle: {}", angle),
            Err(Error::Timeout) => error!("I2C operation timed out"),
            Err(e) => error!("Failed to read raw angle: {:?}", e),
        }

        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn pwm_task(pwm_timer: low_level::Timer<'static, TIM1>) {
    let mut cycle = 0u32;
    let max_duty = pwm_timer.get_max_compare_value() + 1;

    pwm_timer.enable_outputs();
    pwm_timer.start();

    [
        embassy_stm32::timer::Channel::Ch1,
        embassy_stm32::timer::Channel::Ch2,
        embassy_stm32::timer::Channel::Ch3,
    ]
    .iter()
    .for_each(|&ch| {
        pwm_timer.enable_channel(ch, true);
        pwm_timer.set_output_compare_mode(ch, low_level::OutputCompareMode::PwmMode1);
        pwm_timer.set_output_compare_preload(ch, true);
    });
    info!("Max duty cycle: {}", max_duty);
    loop {
        pwm_timer.set_compare_value(embassy_stm32::timer::Channel::Ch1, cycle);
        pwm_timer.set_compare_value(embassy_stm32::timer::Channel::Ch2, cycle);
        pwm_timer.set_compare_value(embassy_stm32::timer::Channel::Ch3, cycle);

        info!("Duty cycle: {} / {}", cycle, max_duty);
        cycle += max_duty / 100;
        cycle %= max_duty;
        Timer::after(Duration::from_millis(100)).await;
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

    let _pin1 = PwmPin::new_ch1(p.PA8, OutputType::PushPull);
    let _pin2 = PwmPin::new_ch2(p.PA9, OutputType::PushPull);
    let _pin3 = PwmPin::new_ch3(p.PA10, OutputType::PushPull);

    let timer = low_level::Timer::new(p.TIM1);
    timer.set_max_compare_value((1 << 12) - 1);
    timer.set_counting_mode(low_level::CountingMode::CenterAlignedUpInterrupts);
    info!("Timer frequency: {}", timer.get_frequency());

    unwrap!(spawner.spawn(blinker(led, Duration::from_millis(300))));
    unwrap!(spawner.spawn(i2c_task(p_i2c)));
    unwrap!(spawner.spawn(pwm_task(timer)));
}
