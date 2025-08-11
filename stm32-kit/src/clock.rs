use defmt::*;
use embassy_stm32::{Config, pac, rcc};

pub fn set_clock(config: &mut Config) {
    let mut rcc_config = rcc::Config::default();
    rcc_config.sys = rcc::Sysclk::PLL1_R;
    rcc_config.pll = Some(rcc::Pll {
        source: embassy_stm32::rcc::PllSource::HSI,    // 16 MHz
        prediv: rcc::PllPreDiv::DIV4,                  // 4 MHz
        mul: rcc::PllMul::MUL85,                       // 340 MHz for PLL
        divr: Some(embassy_stm32::rcc::PllRDiv::DIV2), // 170 MHz for CPU
        divp: Some(embassy_stm32::rcc::PllPDiv::DIV2), // 170 MHz for CPU
        divq: Some(embassy_stm32::rcc::PllQDiv::DIV2), // 170 MHz for USB
    });
    rcc_config.ahb_pre = rcc::AHBPrescaler::DIV1; // 170 MHz for AHB
    rcc_config.apb1_pre = rcc::APBPrescaler::DIV1; // 170 MHz for APB1 (I2C, ...)
    rcc_config.apb2_pre = rcc::APBPrescaler::DIV1; // 170 MHz for APB2
    rcc_config.mux.fdcansel = embassy_stm32::rcc::mux::Fdcansel::PLL1_Q;
    rcc_config.mux.adc12sel = rcc::mux::Adcsel::SYS;
    rcc_config.boost = true;

    pac::RCC.ahb2enr().modify(|w| w.set_adc12en(true));
    pac::RCC
        .ccipr()
        .modify(|w| w.set_adc12sel(pac::rcc::vals::Adcsel::SYS));

    pac::ADC12_COMMON.ccr().modify(|w| {
        w.set_ckmode(pac::adccommon::vals::Ckmode::ASYNCHRONOUS);
        w.set_presc(pac::adccommon::vals::Presc::DIV8);
    });

    config.rcc = rcc_config;
}

pub fn print_clock_info(p_rcc: &embassy_stm32::Peri<'static, embassy_stm32::peripherals::RCC>) {
    let clocks = rcc::clocks(p_rcc);
    info!("Clocks: {:?}", clocks);
}
