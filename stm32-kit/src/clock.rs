use defmt::*;
use embassy_stm32::{
    Config,
    rcc::{self, clocks},
};

pub fn set_clock(config: &mut Config) {
    let mut rcc_config = rcc::Config::default();
    rcc_config.sys = rcc::Sysclk::PLL1_P;
    rcc_config.pll = Some(rcc::Pll {
        prediv: rcc::PllPreDiv::DIV16,  // 1 MHz for PLL pre-divider
        mul: rcc::PllMul::MUL336,       // 336 MHz for PLL
        divp: Some(rcc::PllPDiv::DIV2), // 168 MHz for CPU
        divq: Some(rcc::PllQDiv::DIV7), // 48 MHz for USB
        divr: None,
    });
    rcc_config.apb1_pre = rcc::APBPrescaler::DIV4; // 42 MHz for APB1
    rcc_config.apb2_pre = rcc::APBPrescaler::DIV2; // 84 MHz for APB2
    config.rcc = rcc_config;
}

pub fn print_clock_info(p_rcc: &embassy_stm32::Peri<'static, embassy_stm32::peripherals::RCC>) {
    let clocks = clocks(p_rcc);
    info!("Clocks: {:?}", clocks);
}
