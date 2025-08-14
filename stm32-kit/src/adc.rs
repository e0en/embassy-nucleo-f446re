use core::marker::PhantomData;
use embassy_stm32::Peri;
use embassy_stm32::adc as stm32_adc;
use embassy_stm32::adc::AdcChannel;
use embassy_stm32::adc::RxDma;
use embassy_stm32::dma;
use embassy_stm32::gpio;
use embassy_stm32::pac;

pub const DMA_BUFFER_SIZE: usize = 3 * 2;
const TIM1_TRGO: u8 = 9; // RM0440 JEXTSEL code for TIM1_TRGO (injected group)

pub struct DummyAdc<Tadc: stm32_adc::Instance, Tdma: dma::Channel + RxDma<Tadc>> {
    _inner: Peri<'static, Tadc>,
    dma_buffer: dma::ReadableRingBuffer<'static, u16>,
    _phantom1: PhantomData<Tdma>,
}

impl<Tadc, Tdma> DummyAdc<Tadc, Tdma>
where
    Tadc: stm32_adc::Instance + AdcSelector,
    Tdma: dma::Channel + RxDma<Tadc>,
{
    pub fn new<P1, P2, P3>(
        p_adc: Peri<'static, Tadc>,
        p_dma: Peri<'static, Tdma>,
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
        let dma_buffer: &mut [u16; DMA_BUFFER_SIZE] =
            cortex_m::singleton!(: [u16; DMA_BUFFER_SIZE] = [0u16; DMA_BUFFER_SIZE]).unwrap();
        let request = p_dma.request();
        let mut dma_opts = dma::TransferOptions::default();
        dma_opts.circular = true;
        dma_opts.priority = dma::Priority::VeryHigh;
        dma_opts.complete_transfer_ir = true;

        let peri_addr = {
            let p = get_pac_adc(&p_adc);
            p.dr().as_ptr() as *mut u16
        };

        let dma_buffer = unsafe {
            dma::ReadableRingBuffer::new(p_dma, request, peri_addr, dma_buffer, dma_opts)
        };

        Self {
            _inner: p_adc,
            dma_buffer,
            _phantom1: PhantomData,
        }
    }

    pub fn read(&mut self, buf: &mut [u16]) {
        let _ = self.dma_buffer.read(buf);
    }

    pub fn calibrate(&mut self) {
        let p_adc = get_pac_adc(&self._inner);
        calibrate(p_adc);
    }

    pub fn initialize(&mut self, ch1: u8, ch2: u8, ch3: u8) {
        let p_adc = get_pac_adc(&self._inner);
        initialize(p_adc, ch1, ch2, ch3);
        self.dma_buffer.start();
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

pub fn calibrate(adc1: pac::adc::Adc) {
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

pub fn initialize(adc1: pac::adc::Adc, ch1: u8, ch2: u8, ch3: u8) {
    // disable ADC
    if adc1.cr().read().aden() {
        adc1.cr().modify(|w| w.set_addis(true));
        while adc1.cr().read().aden() {}
    }

    // stop any ongoing adc measurements
    adc1.cr().modify(|w| w.set_adstart(false));

    // set sample times
    adc1.smpr().modify(|w| {
        w.set_smp(ch1 as usize, stm32_adc::SampleTime::CYCLES2_5);
        w.set_smp(ch2 as usize, stm32_adc::SampleTime::CYCLES2_5);
        w.set_smp(ch3 as usize, stm32_adc::SampleTime::CYCLES2_5);
    });

    // set injected group
    adc1.sqr1().modify(|w| {
        w.set_l(2); // = 3 channels
        w.set_sq(0, ch1);
        w.set_sq(1, ch2);
        w.set_sq(2, ch3);
    });

    adc1.cfgr().modify(|w| {
        w.set_jqdis(true);

        w.set_cont(false); // hardware-triggered single sequence per trigger
        w.set_discen(false); // no discontinuous
        w.set_align(false);
        w.set_ovrmod(stm32_adc::vals::Ovrmod::OVERWRITE);
        w.set_dmacfg(stm32_adc::vals::Dmacfg::CIRCULAR);
        w.set_dmaen(stm32_adc::vals::Dmaen::ENABLE);

        w.set_exten(stm32_adc::vals::Exten::RISING_EDGE);
        w.set_extsel(TIM1_TRGO);
    });

    adc1.isr().write(|w| {
        w.set_eoc(true);
        w.set_eos(true);
        w.set_jeoc(true);
        w.set_jeos(true);
    });

    // no interrupt handler calls
    adc1.ier().modify(|w| {
        w.set_jeocie(false);
        w.set_jeosie(false);
        w.set_eocie(false);
        w.set_eosie(false);
        w.set_ovrie(false);
    });

    // timer TRGO will retrigger
    adc1.cr().modify(|w| w.set_adstart(true));

    // re-enable ADC
    adc1.cr().modify(|w| w.set_aden(true));
    while !adc1.isr().read().adrdy() {}
}
