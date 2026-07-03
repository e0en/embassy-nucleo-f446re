#![no_std]
#![no_main]

#[cfg(all(feature = "main-fw", feature = "tuner-fw"))]
compile_error!("features `main-fw` and `tuner-fw` cannot be enabled together");

mod adc;
mod app;
mod as5047p;
mod bldc_driver;
#[cfg(feature = "tuner-fw")]
mod calibration;
mod can;
mod clock;
mod cordic;
#[cfg(feature = "cordic-self-test")]
mod cordic_selftest;
#[cfg(feature = "tuner-fw")]
mod current_tuning;
mod drv8316;
mod drv8316t;
mod dual_encoder;
mod encoder_correction;
#[cfg(feature = "tuner-fw")]
mod encoder_correction_tuning;
mod flash_config;
mod foc_isr;
mod gm3506;
#[cfg(feature = "tuner-fw")]
mod motor_tuning;
mod pwm;
#[cfg(feature = "tuner-fw")]
mod velocity_tuning;

use app::{
    ACTUATOR_REDUCTION_RATIO_MAGNITUDE, CAN_BITRATE, CAN_INTERRUPT_PRIORITY, DEFAULT_MOTOR_CAN_ID,
    INIT_DELAY_CYCLES, VELOCITY_OBSERVER_BANDWIDTH, can_control, encoder, fault, init, monitor,
    persistence::{CAN_ID, CAN_PROPERTIES, FLASH, SPI},
};
use as5047p::As5047P;
use defmt::*;
use dual_encoder::DualEncoder;
use embassy_executor::Spawner;
use embassy_stm32::{
    Config, bind_interrupts,
    can::{self as stm32_can},
    flash::Flash,
    gpio, i2c as stm32_i2c,
    interrupt::{self, InterruptExt},
    spi as stm32_spi,
    time::mhz,
};
use {defmt_rtt as _, panic_probe as _};

pub(crate) use app::encoder::read_sensor;

bind_interrupts!(
    struct Irqs {
        I2C1_EV => stm32_i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
        I2C1_ER => stm32_i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
        FDCAN1_IT1 => stm32_can::IT1InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
        FDCAN1_IT0 => stm32_can::IT0InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
    }
);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    cortex_m::asm::delay(INIT_DELAY_CYCLES);

    let mut config = Config::default();
    clock::set_clock(&mut config);
    cordic::initialize_cordic();
    #[cfg(feature = "cordic-self-test")]
    cordic_selftest::run_and_log_validation_tests();
    let p = embassy_stm32::init(config);
    clock::print_clock_info(&p.RCC);

    embassy_stm32::interrupt::TIM1_UP_TIM16.set_priority(interrupt::Priority::P0);
    embassy_stm32::interrupt::ADC1_2.set_priority(interrupt::Priority::P1);
    embassy_stm32::interrupt::FDCAN1_IT0.set_priority(CAN_INTERRUPT_PRIORITY);
    embassy_stm32::interrupt::FDCAN1_IT1.set_priority(CAN_INTERRUPT_PRIORITY);

    let mut spi_config = stm32_spi::Config::default();
    spi_config.miso_pull = gpio::Pull::Up;
    spi_config.mode = stm32_spi::MODE_1;
    spi_config.bit_order = stm32_spi::BitOrder::MsbFirst;
    spi_config.frequency = mhz(10);
    spi_config.gpio_speed = gpio::Speed::VeryHigh;
    let enc1_cs_out = gpio::Output::new(p.PB0, gpio::Level::High, gpio::Speed::VeryHigh);
    let enc2_cs_out = gpio::Output::new(p.PB1, gpio::Level::High, gpio::Speed::VeryHigh);

    {
        let p_spi = stm32_spi::Spi::new(
            p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH4, spi_config,
        );
        *(SPI.lock().await) = Some(p_spi);
    }

    let mut p_flash = Flash::new_blocking(p.FLASH);
    let stored_config = flash_config::read_config(&mut p_flash);
    let secondary_zero_offset = stored_config
        .as_ref()
        .map(|config| config.secondary_zero_offset)
        .unwrap_or(0.0);
    let startup_can_id = stored_config
        .as_ref()
        .map(|config| config.can_id)
        .unwrap_or(DEFAULT_MOTOR_CAN_ID);

    let mut can_conf = stm32_can::CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    can_conf.set_bitrate(CAN_BITRATE);
    can_conf.set_config(
        can_conf
            .config()
            .set_global_filter(stm32_can::config::GlobalFilter::reject_all()),
    );
    can_conf.properties().set_extended_filter(
        stm32_can::filter::ExtendedFilterSlot::_0,
        can_control::can_id_filter(startup_can_id),
    );
    let p_can = can_conf.into_normal_mode();
    let (can_tx, can_rx, properties) = p_can.split();

    {
        *(CAN_PROPERTIES.lock().await) = Some(properties);
    }

    let drvoff_pin = gpio::Output::new(p.PB12, gpio::Level::High, gpio::Speed::Medium);
    let drv_fault_pin = gpio::Input::new(p.PB11, gpio::Pull::Up);
    let led_fault = gpio::Output::new(p.PA4, gpio::Level::High, gpio::Speed::Low);

    let mut p_adc = adc::DummyAdc::new(p.ADC1, p.PA0, p.PA1, p.PA2, p.PA3);
    p_adc.calibrate();
    p_adc.initialize(1, 2, 3);

    let mut timer = pwm::Pwm6Timer::new(p.TIM1, p.PA8, p.PA9, p.PA10, p.PB13, p.PB14, p.PB15);
    timer.initialize(foc_isr::MAX_COMPARE_VALUE as u16);
    info!(
        "Timer edge-aligned wrap frequency (before center-aligned/REP effects): {}",
        timer.get_frequency()
    );

    let mut sensor = As5047P::new(&SPI, enc1_cs_out, VELOCITY_OBSERVER_BANDWIDTH, 0.0);
    match sensor.initialize().await {
        Ok(diagnostics) => info!("ENC1 diagnostics: {:?}", diagnostics),
        Err(e) => {
            error!("ENC1 initialization failed, {:?}", e);
            return;
        }
    }

    let mut secondary_sensor = As5047P::new(
        &SPI,
        enc2_cs_out,
        VELOCITY_OBSERVER_BANDWIDTH,
        secondary_zero_offset,
    );
    match secondary_sensor.initialize().await {
        Ok(diagnostics) => info!("ENC2 diagnostics: {:?}", diagnostics),
        Err(e) => {
            error!("ENC2 initialization failed, {:?}", e);
            return;
        }
    };

    let dual_encoder =
        DualEncoder::new(sensor, secondary_sensor, ACTUATOR_REDUCTION_RATIO_MAGNITUDE);

    unwrap!(spawner.spawn(encoder::encoder_task(dual_encoder)));

    let Some(can_id) = init::init_foc(drvoff_pin, timer, p_adc, &mut p_flash, stored_config).await
    else {
        return;
    };
    CAN_ID.store(can_id, core::sync::atomic::Ordering::Relaxed);

    {
        *(FLASH.lock().await) = Some(p_flash);
    }

    unwrap!(spawner.spawn(monitor::monitor_task()));
    unwrap!(spawner.spawn(fault::drv_fault_monitor_task(drv_fault_pin, led_fault)));
    unwrap!(spawner.spawn(can_control::command_task()));
    unwrap!(spawner.spawn(can_control::feedback_task()));
    unwrap!(spawner.spawn(can_control::can_rx_task(can_rx)));
    unwrap!(spawner.spawn(can_control::can_tx_task(can_tx)));
}
