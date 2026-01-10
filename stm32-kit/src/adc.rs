use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};
use embassy_stm32::Peri;
use embassy_stm32::adc as stm32_adc;
use embassy_stm32::adc::AdcChannel;
use embassy_stm32::gpio;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::pac;

use crate::foc_isr;

// VM_SENSE voltage divider: 110kΩ (top) + 10kΩ (bottom)
// V_adc = V_motor * 10k / (110k + 10k) = V_motor / 12
const VM_SENSE_DIVIDER_RATIO: f32 = 12.0;
// ADC reference voltage from AP2112K-3.3 LDO
const ADC_VREF: f32 = 3.3;
// 12-bit ADC resolution
const ADC_MAX_VALUE: f32 = 4096.0;

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

/// Measure motor supply voltage from VM_SENSE pin (PA3 = ADC1_IN4).
/// Performs a single software-triggered conversion on the regular group.
/// Returns the measured voltage in volts.
pub fn measure_vm_sense() -> f32 {
    let adc1 = pac::ADC1;

    // VM_SENSE is on PA3 = ADC1 channel 4
    const VM_SENSE_CHANNEL: u8 = 4;

    // Save current JSQR state (injected sequence used for current sensing)
    let jsqr_backup = adc1.jsqr().read();

    // Stop any ongoing injected conversion
    adc1.cr()
        .modify(|w| w.set_jadstp(pac::adc::vals::Adstp::STOP));
    while adc1.cr().read().jadstart() {}

    // Configure regular sequence for single channel conversion
    adc1.sqr1().modify(|w| {
        w.set_l(0); // 1 conversion (L = 0 means 1 conversion)
        w.set_sq(1, VM_SENSE_CHANNEL);
    });

    // Set sample time for the VM_SENSE channel
    adc1.smpr().modify(|w| {
        w.set_smp(VM_SENSE_CHANNEL as usize, SAMPLE_DURATION);
    });

    // Start regular conversion
    adc1.cr().modify(|w| w.set_adstart(true));

    // Wait for conversion to complete (EOC flag)
    while !adc1.isr().read().eoc() {}

    // Read the result
    let raw_value = adc1.dr().read().0 as u16;

    // Clear EOC flag
    adc1.isr().write(|w| w.set_eoc(true));

    // Restore injected sequence and restart
    adc1.jsqr().write_value(jsqr_backup);
    adc1.cr().modify(|w| w.set_jadstart(true));

    // Convert raw ADC value to voltage
    let adc_voltage = (raw_value as f32) * ADC_VREF / ADC_MAX_VALUE;
    adc_voltage * VM_SENSE_DIVIDER_RATIO
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
