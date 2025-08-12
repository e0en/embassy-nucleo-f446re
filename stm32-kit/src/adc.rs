use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};
use embassy_stm32::Peri;
use embassy_stm32::adc as stm32_adc;
use embassy_stm32::adc::AdcChannel;
use embassy_stm32::gpio;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::pac;
use embassy_stm32::peripherals::ADC1;

const JEXT_TIM1_TRGO: u8 = 0; // RM0440 JEXTSEL code for TIM1_TRGO (injected group)

pub static IA_RAW: AtomicU16 = AtomicU16::new(0);
pub static IB_RAW: AtomicU16 = AtomicU16::new(0);
pub static IC_RAW: AtomicU16 = AtomicU16::new(0);
pub static CNT: AtomicU32 = AtomicU32::new(0);

pub struct DummyAdc {
    _inner: stm32_adc::Adc<'static, ADC1>,
}

impl DummyAdc {
    pub fn new<P1, P2, P3>(
        p_adc: Peri<'static, ADC1>,
        _ch1_pin: Peri<'static, P1>,
        _ch2_pin: Peri<'static, P2>,
        _ch3_pin: Peri<'static, P3>,
    ) -> Self
    where
        Peri<'static, P1>: AdcChannel<ADC1>,
        Peri<'static, P2>: AdcChannel<ADC1>,
        Peri<'static, P3>: AdcChannel<ADC1>,
        P1: gpio::Pin,
        P2: gpio::Pin,
        P3: gpio::Pin,
    {
        Self {
            _inner: stm32_adc::Adc::new(p_adc),
        }
    }
}

#[interrupt]
fn ADC1_2() {
    // this runs on ADC1_2 event.

    let tim1 = pac::TIM1;
    let t = tim1.cnt().read().0;

    let adc1 = pac::ADC1;
    let isr = adc1.isr().read();
    if isr.jeos() {
        // Read in rank order
        let ia = adc1.jdr(1).read().0 as u16;
        let ib = adc1.jdr(2).read().0 as u16;
        let ic = adc1.jdr(3).read().0 as u16;
        IA_RAW.store(ia, Ordering::Relaxed);
        IB_RAW.store(ib, Ordering::Relaxed);
        IC_RAW.store(ic, Ordering::Relaxed);
        CNT.store(t, Ordering::Relaxed);

        // Clear JEOS
        adc1.isr().write(|w| w.set_jeos(true));
    }
}

pub fn calibrate() {
    let adc1 = pac::ADC1;

    // disable deep power down of ADC, enable ADC analog regulator
    adc1.cr().modify(|w| {
        w.set_deeppwd(false);
        w.set_advregen(true);
    });

    // wait tADCVREG = 20us = 3400 cycles @ 170MHz
    cortex_m::asm::delay(3400);

    // disable
    if adc1.cr().read().aden() {
        adc1.cr().modify(|w| w.set_addis(true));
        while adc1.cr().read().aden() {}
    }

    // run calibration
    adc1.cr()
        .modify(|w| w.set_adcaldif(stm32_adc::vals::Adcaldif::SINGLE_ENDED));
    adc1.cr().modify(|w| w.set_adcal(true));
    while adc1.cr().read().adcal() {}

    // Enable ADC and wait ADRDY
    adc1.isr().write(|w| w.set_adrdy(true)); // clear
    adc1.cr().modify(|w| w.set_aden(true));
    while !adc1.isr().read().adrdy() {}
}

pub fn initialize(ch1: u8, ch2: u8, ch3: u8) {
    let adc1 = pac::ADC1;

    // disable ADC
    if adc1.cr().read().aden() {
        adc1.cr().modify(|w| w.set_addis(true));
        while adc1.cr().read().aden() {}
    }

    // stop any ongoing adc measurements
    adc1.cr().modify(|w| w.set_jadstart(false));

    // set sample times
    adc1.smpr().modify(|w| {
        w.set_smp(ch1 as usize, stm32_adc::SampleTime::CYCLES2_5);
        w.set_smp(ch2 as usize, stm32_adc::SampleTime::CYCLES2_5);
        w.set_smp(ch3 as usize, stm32_adc::SampleTime::CYCLES2_5);
    });

    // set injected group
    adc1.jsqr().modify(|w| {
        w.set_jl(3); // = 3 channels
        w.set_jsq(1, ch1);
        w.set_jsq(2, ch2);
        w.set_jsq(3, ch3);
        w.set_jexten(stm32_adc::vals::Exten::RISING_EDGE);
        w.set_jextsel(JEXT_TIM1_TRGO);
    });

    adc1.cfgr().modify(|w| {
        w.set_jqdis(true); // use JSQR directly
        w.set_jdiscen(false); // no discontinuous
        w.set_jauto(false);
        w.set_cont(false); // hardware-triggered single sequence per trigger
    });

    // clear W1C flags
    adc1.isr().write(|w| {
        w.set_jeos(true);
        w.set_jeoc(true);
        w.set_ovr(true);
    });

    // enable interrupt
    embassy_stm32::interrupt::ADC1_2.set_priority(interrupt::Priority::P0);
    unsafe {
        embassy_stm32::interrupt::ADC1_2.enable();
    }

    adc1.ier().modify(|w| w.set_jeosie(true));

    // timer TRGO will retrigger
    adc1.cr().modify(|w| w.set_jadstart(true));

    // re-enable ADC
    adc1.cr().modify(|w| w.set_aden(true));
    while !adc1.isr().read().adrdy() {}
}
