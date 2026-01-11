use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};
use embassy_stm32::Peri;
use embassy_stm32::adc as stm32_adc;
use embassy_stm32::adc::AdcChannel;
use embassy_stm32::gpio;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::pac;

use crate::foc_isr;

const JEXT_TIM1_TRGO: u8 = 0; // RM0440 JEXTSEL code for TIM1_TRGO (injected group)

const SAMPLE_DURATION: stm32_adc::SampleTime = stm32_adc::SampleTime::CYCLES12_5;

pub static IA_RAW: AtomicU16 = AtomicU16::new(0);
pub static IB_RAW: AtomicU16 = AtomicU16::new(0);
pub static IC_RAW: AtomicU16 = AtomicU16::new(0);
pub static CNT: AtomicU32 = AtomicU32::new(0);

pub struct DummyAdc<Tadc: stm32_adc::Instance> {
    _inner: Peri<'static, Tadc>,
}

impl<Tadc> DummyAdc<Tadc>
where
    Tadc: stm32_adc::Instance + AdcSelector,
{
    pub fn new<P1, P2, P3>(
        p_adc: Peri<'static, Tadc>,
        _ch1_pin: Peri<'static, P1>,
        _ch2_pin: Peri<'static, P2>,
        _ch3_pin: Peri<'static, P3>,
    ) -> Self
    where
        Peri<'static, P1>: AdcChannel<Tadc>,
        Peri<'static, P2>: AdcChannel<Tadc>,
        Peri<'static, P3>: AdcChannel<Tadc>,
        P1: gpio::Pin,
        P2: gpio::Pin,
        P3: gpio::Pin,
    {
        Self { _inner: p_adc }
    }

    pub fn read(&self) -> (u16, u16, u16) {
        let ia = IA_RAW.load(Ordering::Relaxed);
        let ib = IB_RAW.load(Ordering::Relaxed);
        let ic = IC_RAW.load(Ordering::Relaxed);
        (ia, ib, ic)
    }

    pub fn calibrate(&mut self) {
        let p_adc = get_pac_adc(&self._inner);
        calibrate(p_adc);
    }

    pub fn initialize(&mut self, ch1: u8, ch2: u8, ch3: u8) {
        let p_adc = get_pac_adc(&self._inner);
        initialize(p_adc, ch1, ch2, ch3);
    }
}

// Trait to map ADC types to their PAC objects at compile time
pub trait AdcSelector {
    fn get_pac_adc() -> pac::adc::Adc;
}

impl AdcSelector for embassy_stm32::peripherals::ADC1 {
    fn get_pac_adc() -> pac::adc::Adc {
        pac::ADC1
    }
}

impl AdcSelector for embassy_stm32::peripherals::ADC2 {
    fn get_pac_adc() -> pac::adc::Adc {
        pac::ADC2
    }
}

fn get_pac_adc<'a, T>(_p_adc: &Peri<'a, T>) -> pac::adc::Adc
where
    T: stm32_adc::Instance + AdcSelector,
{
    T::get_pac_adc()
}

#[interrupt]
fn ADC1_2() {
    // This runs on ADC1_2 injected end-of-sequence event,
    // triggered by TIM1 TRGO at PWM frequency (~41.5 kHz).

    let tim1 = pac::TIM1;
    let t = tim1.cnt().read().0;

    let adc1 = pac::ADC1;
    let isr = adc1.isr().read();
    if isr.jeos() {
        let ia = adc1.jdr(1).read().0 as u16;
        let ib = adc1.jdr(2).read().0 as u16;
        let ic = adc1.jdr(3).read().0 as u16;

        IA_RAW.store(ia, Ordering::Relaxed);
        IB_RAW.store(ib, Ordering::Relaxed);
        IC_RAW.store(ic, Ordering::Relaxed);
        CNT.store(t, Ordering::Relaxed);

        if foc_isr::is_initialized() {
            foc_isr::run_foc_iteration(ia, ib, ic);
        }

        adc1.isr().write(|w| w.set_jeos(true));
    }
}

pub fn calibrate(adc1: pac::adc::Adc) {
    adc1.cr().modify(|w| {
        w.set_deeppwd(false);
        w.set_advregen(true);
    });

    // wait tADCVREG = 20us = 3400 cycles @ 170MHz
    cortex_m::asm::delay(3400);

    if adc1.cr().read().aden() {
        adc1.cr().modify(|w| w.set_addis(true));
        while adc1.cr().read().aden() {}
    }

    adc1.cr()
        .modify(|w| w.set_adcaldif(stm32_adc::vals::Adcaldif::SINGLE_ENDED));
    adc1.cr().modify(|w| w.set_adcal(true));
    while adc1.cr().read().adcal() {}

    adc1.isr().write(|w| w.set_adrdy(true));
    adc1.cr().modify(|w| w.set_aden(true));
    while !adc1.isr().read().adrdy() {}
}

pub fn initialize(adc1: pac::adc::Adc, ch1: u8, ch2: u8, ch3: u8) {
    if adc1.cr().read().aden() {
        adc1.cr().modify(|w| w.set_addis(true));
        while adc1.cr().read().aden() {}
    }

    adc1.cr().modify(|w| w.set_jadstart(false));

    adc1.smpr().modify(|w| {
        w.set_smp(ch1 as usize, SAMPLE_DURATION);
        w.set_smp(ch2 as usize, SAMPLE_DURATION);
        w.set_smp(ch3 as usize, SAMPLE_DURATION);
    });

    adc1.jsqr().modify(|w| {
        w.set_jl(3);
        w.set_jsq(1, ch1);
        w.set_jsq(2, ch2);
        w.set_jsq(3, ch3);
        w.set_jexten(stm32_adc::vals::Exten::RISING_EDGE);
        w.set_jextsel(JEXT_TIM1_TRGO);
    });

    adc1.cfgr().modify(|w| {
        w.set_jqdis(true);
        w.set_jdiscen(false);
        w.set_jauto(false);
        w.set_cont(false);
    });

    adc1.isr().write(|w| {
        w.set_jeos(true);
        w.set_jeoc(true);
        w.set_ovr(true);
    });

    embassy_stm32::interrupt::ADC1_2.set_priority(interrupt::Priority::P0);
    unsafe {
        embassy_stm32::interrupt::ADC1_2.enable();
    }

    adc1.ier().modify(|w| w.set_jeosie(true));

    adc1.cr().modify(|w| w.set_jadstart(true));
    adc1.cr().modify(|w| w.set_aden(true));
    while !adc1.isr().read().adrdy() {}
}
