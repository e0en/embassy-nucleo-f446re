#![no_std]
#![no_main]
mod as5600;
mod can;
mod clock;
mod i2c;

use crate::as5600::{As5600, MagnetStatus};

use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can as stm32_can;
use embassy_stm32::i2c as stm32_i2c;
use embassy_stm32::{
    Config, bind_interrupts,
    i2c::{Error, I2c, Master},
    peripherals,
    timer::{low_level, simple_pwm::PwmPin},
};

use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::{peripherals::TIM1, time::khz};
use embassy_time::{Duration, Timer};
use foc::sensor::Sensor;

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
    let mut sensor = As5600::new();
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

        let _reading = sensor.read_async(&mut p_i2c).await;

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

#[embassy_executor::task]
async fn can_task() {
    // Placeholder for CAN task implementation
    loop {
        Timer::after(Duration::from_secs(1)).await;
        info!("CAN task running");
    }
}

bind_interrupts!(
    struct Irqs {
        I2C1_EV => stm32_i2c::EventInterruptHandler<peripherals::I2C1>;
        I2C1_ER => stm32_i2c::ErrorInterruptHandler<peripherals::I2C1>;
        CAN1_RX0 => stm32_can::Rx0InterruptHandler<peripherals::CAN1>;
        CAN1_RX1 => stm32_can::Rx1InterruptHandler<peripherals::CAN1>;
        CAN1_SCE => stm32_can::SceInterruptHandler<peripherals::CAN1>;
        CAN1_TX => stm32_can::TxInterruptHandler<peripherals::CAN1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    clock::set_clock(&mut config);
    let mut p = embassy_stm32::init(config);
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

    // Initialize PWM
    let _pin1 = PwmPin::new_ch1(p.PA8, OutputType::PushPull);
    let _pin2 = PwmPin::new_ch2(p.PA9, OutputType::PushPull);
    let _pin3 = PwmPin::new_ch3(p.PA10, OutputType::PushPull);

    let timer = low_level::Timer::new(p.TIM1);
    timer.set_max_compare_value((1 << 12) - 1);
    timer.set_counting_mode(low_level::CountingMode::CenterAlignedUpInterrupts);
    info!("Timer frequency: {}", timer.get_frequency());

    // Initialize CAN

    // Pull up PA11 for CAN loopback test
    let rx_pin = Input::new(p.PA11.reborrow(), Pull::Up);
    core::mem::forget(rx_pin);

    let mut can = stm32_can::Can::new(p.CAN1, p.PA11, p.PA12, Irqs);
    can.modify_filters().enable_bank(
        0,
        stm32_can::Fifo::Fifo0,
        stm32_can::filter::Mask32::accept_all(),
    );

    can.modify_config()
        .set_loopback(true) // Receive own frames
        .set_silent(true)
        .set_bitrate(1_000_000);
    can.enable().await;

    unwrap!(spawner.spawn(blinker(led, Duration::from_millis(300))));
    unwrap!(spawner.spawn(i2c_task(p_i2c)));
    unwrap!(spawner.spawn(pwm_task(timer)));
    unwrap!(spawner.spawn(can_task()));
}
