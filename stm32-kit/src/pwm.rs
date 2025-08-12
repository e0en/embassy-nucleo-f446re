use embassy_stm32::gpio::OutputType;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::complementary_pwm::ComplementaryPwmPin;
use embassy_stm32::timer::{
    AdvancedInstance4Channel, TimerComplementaryPin, TimerPin, low_level, simple_pwm::PwmPin,
};
use embassy_stm32::timer::{Ch1, Ch2, Ch3};
use embassy_stm32::{Peri, pac};

use core::marker::PhantomData;

#[allow(dead_code)]
pub struct Pwm3Timer<
    'a,
    TIM: AdvancedInstance4Channel,
    T1: TimerPin<TIM, Ch1>,
    T2: TimerPin<TIM, Ch2>,
    T3: TimerPin<TIM, Ch3>,
> {
    pub timer: low_level::Timer<'a, TIM>,
    pub _pin1: PwmPin<'a, TIM, Ch1>,
    pub _pin2: PwmPin<'a, TIM, Ch2>,
    pub _pin3: PwmPin<'a, TIM, Ch3>,
    _phantom1: PhantomData<T1>,
    _phantom2: PhantomData<T2>,
    _phantom3: PhantomData<T3>,
}

#[allow(dead_code)]
impl<
    'a,
    TIM: AdvancedInstance4Channel,
    T1: TimerPin<TIM, Ch1>,
    T2: TimerPin<TIM, Ch2>,
    T3: TimerPin<TIM, Ch3>,
> Pwm3Timer<'a, TIM, T1, T2, T3>
{
    pub fn new(
        p_timer: Peri<'a, TIM>,
        p1: Peri<'a, T1>,
        p2: Peri<'a, T2>,
        p3: Peri<'a, T3>,
    ) -> Self {
        let pin1 = PwmPin::new(p1, OutputType::PushPull);
        let pin2 = PwmPin::new(p2, OutputType::PushPull);
        let pin3 = PwmPin::new(p3, OutputType::PushPull);

        let timer = low_level::Timer::new(p_timer);
        timer.set_counting_mode(low_level::CountingMode::CenterAlignedUpInterrupts);

        Self {
            timer,
            _pin1: pin1,
            _pin2: pin2,
            _pin3: pin3,
            _phantom1: PhantomData,
            _phantom2: PhantomData,
            _phantom3: PhantomData,
        }
    }

    pub fn get_frequency(&self) -> Hertz {
        self.timer.get_frequency()
    }

    pub fn initialize(&mut self) {
        self.timer.stop();
        [
            embassy_stm32::timer::Channel::Ch1,
            embassy_stm32::timer::Channel::Ch2,
            embassy_stm32::timer::Channel::Ch3,
        ]
        .iter()
        .for_each(|&ch| {
            self.timer.enable_channel(ch, true);
            self.timer
                .set_output_compare_mode(ch, low_level::OutputCompareMode::PwmMode1);
            self.timer.set_output_compare_preload(ch, true);
            self.timer.enable_complementary_channel(ch, true);
        });

        self.timer.regs_advanced().cr1().modify(|w| {
            w.set_opm(false); // not one-pulse
            w.set_udis(false); // do not suppress UEV
            w.set_urs(pac::timer::vals::Urs::COUNTER_ONLY); // only real over/underflow causes UEV (stable TRGO)
            w.set_cms(pac::timer::vals::Cms::CENTER_ALIGNED3);
            w.set_dir(pac::timer::vals::Dir::DOWN); // start by counting down
        });

        // set timer max compare value
        self.timer
            .regs_advanced()
            .arr()
            .write(|r| r.set_arr((1 << 12) - 1));

        self.timer
            .regs_advanced()
            .smcr()
            .modify(|w| w.set_sms(pac::timer::vals::Sms::DISABLED));
        self.timer
            .regs_advanced()
            .cr2()
            .modify(|w| w.set_mms(pac::timer::vals::Mms::UPDATE));

        self.timer.regs_advanced().rcr().write(|w| w.set_rep(10)); // no down-sample while debugging
        self.timer.regs_advanced().sr().write(|w| w.set_uif(false));

        self.timer.enable_outputs();
        self.timer.start();
    }
}

pub struct Pwm6Timer<
    'a,
    TIM: AdvancedInstance4Channel,
    T1: TimerPin<TIM, Ch1>,
    T2: TimerPin<TIM, Ch2>,
    T3: TimerPin<TIM, Ch3>,
    T1N: TimerComplementaryPin<TIM, Ch1>,
    T2N: TimerComplementaryPin<TIM, Ch2>,
    T3N: TimerComplementaryPin<TIM, Ch3>,
> {
    pub timer: low_level::Timer<'a, TIM>,
    pub _pin1: PwmPin<'a, TIM, Ch1>,
    pub _pin2: PwmPin<'a, TIM, Ch2>,
    pub _pin3: PwmPin<'a, TIM, Ch3>,
    pub _pin1n: ComplementaryPwmPin<'a, TIM, Ch1>,
    pub _pin2n: ComplementaryPwmPin<'a, TIM, Ch2>,
    pub _pin3n: ComplementaryPwmPin<'a, TIM, Ch3>,
    _phantom1: PhantomData<T1>,
    _phantom2: PhantomData<T2>,
    _phantom3: PhantomData<T3>,
    _phantom1n: PhantomData<T1N>,
    _phantom2n: PhantomData<T2N>,
    _phantom3n: PhantomData<T3N>,
}

impl<
    'a,
    TIM: AdvancedInstance4Channel,
    T1: TimerPin<TIM, Ch1>,
    T2: TimerPin<TIM, Ch2>,
    T3: TimerPin<TIM, Ch3>,
    T1N: TimerComplementaryPin<TIM, Ch1>,
    T2N: TimerComplementaryPin<TIM, Ch2>,
    T3N: TimerComplementaryPin<TIM, Ch3>,
> Pwm6Timer<'a, TIM, T1, T2, T3, T1N, T2N, T3N>
{
    pub fn new(
        p_timer: Peri<'a, TIM>,
        p1: Peri<'a, T1>,
        p2: Peri<'a, T2>,
        p3: Peri<'a, T3>,
        p1n: Peri<'a, T1N>,
        p2n: Peri<'a, T2N>,
        p3n: Peri<'a, T3N>,
    ) -> Self {
        let pin1 = PwmPin::new(p1, OutputType::PushPull);
        let pin2 = PwmPin::new(p2, OutputType::PushPull);
        let pin3 = PwmPin::new(p3, OutputType::PushPull);

        let pin1n = ComplementaryPwmPin::new(p1n, OutputType::PushPull);
        let pin2n = ComplementaryPwmPin::new(p2n, OutputType::PushPull);
        let pin3n = ComplementaryPwmPin::new(p3n, OutputType::PushPull);

        let timer = low_level::Timer::new(p_timer);
        timer.set_counting_mode(low_level::CountingMode::CenterAlignedUpInterrupts);

        Self {
            timer,
            _pin1: pin1,
            _pin2: pin2,
            _pin3: pin3,
            _pin1n: pin1n,
            _pin2n: pin2n,
            _pin3n: pin3n,
            _phantom1: PhantomData,
            _phantom2: PhantomData,
            _phantom3: PhantomData,
            _phantom1n: PhantomData,
            _phantom2n: PhantomData,
            _phantom3n: PhantomData,
        }
    }

    pub fn get_frequency(&self) -> Hertz {
        self.timer.get_frequency()
    }

    pub fn initialize(&mut self) {
        self.timer.stop();
        [
            embassy_stm32::timer::Channel::Ch1,
            embassy_stm32::timer::Channel::Ch2,
            embassy_stm32::timer::Channel::Ch3,
        ]
        .iter()
        .for_each(|&ch| {
            self.timer.enable_channel(ch, true);
            self.timer.enable_complementary_channel(ch, true);
            self.timer
                .set_output_compare_mode(ch, low_level::OutputCompareMode::PwmMode1);
            self.timer.set_output_compare_preload(ch, true);
        });

        self.timer.regs_advanced().cr1().modify(|w| {
            w.set_opm(false); // not one-pulse
            w.set_udis(false); // do not suppress UEV
            w.set_urs(pac::timer::vals::Urs::COUNTER_ONLY); // only real over/underflow causes UEV (stable TRGO)
            w.set_cms(pac::timer::vals::Cms::CENTER_ALIGNED3);
            w.set_dir(pac::timer::vals::Dir::DOWN); // start by counting down
        });

        // set timer max compare value
        self.timer
            .regs_advanced()
            .arr()
            .write(|r| r.set_arr((1 << 12) - 1));

        self.timer
            .regs_advanced()
            .smcr()
            .modify(|w| w.set_sms(pac::timer::vals::Sms::DISABLED));
        self.timer
            .regs_advanced()
            .cr2()
            .modify(|w| w.set_mms(pac::timer::vals::Mms::UPDATE));

        self.timer.regs_advanced().rcr().write(|w| w.set_rep(10));
        self.timer.regs_advanced().sr().write(|w| w.set_uif(false));

        self.timer.enable_outputs();
        self.timer.start();
    }
}

pub fn set_duty_cycle<'a, TIM>(timer: &mut low_level::Timer<'a, TIM>, ch1: u32, ch2: u32, ch3: u32)
where
    TIM: AdvancedInstance4Channel,
{
    timer.set_compare_value(embassy_stm32::timer::Channel::Ch1, ch1);
    timer.set_compare_value(embassy_stm32::timer::Channel::Ch2, ch2);
    timer.set_compare_value(embassy_stm32::timer::Channel::Ch3, ch3);
}
