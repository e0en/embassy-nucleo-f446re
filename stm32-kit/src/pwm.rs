use embassy_stm32::gpio::{OutputType, Speed};
use embassy_stm32::timer::{
    AdvancedInstance4Channel, GeneralInstance4Channel, TimerComplementaryPin, TimerPin, low_level,
    simple_pwm::PwmPin,
};
use embassy_stm32::timer::{Ch1, Ch2, Ch3};
use embassy_stm32::{Peri, gpio, pac};

#[allow(dead_code)]
pub fn create_3pwm_timer<'a, TIM, T1, T2, T3>(
    p1: Peri<T1>,
    p2: Peri<T2>,
    p3: Peri<T3>,
    p_timer: Peri<'a, TIM>,
) -> low_level::Timer<'a, TIM>
where
    TIM: GeneralInstance4Channel,
    T1: TimerPin<TIM, Ch1>,
    T2: TimerPin<TIM, Ch2>,
    T3: TimerPin<TIM, Ch3>,
{
    let _pin1 = PwmPin::new(p1, OutputType::PushPull);
    let _pin2 = PwmPin::new(p2, OutputType::PushPull);
    let _pin3 = PwmPin::new(p3, OutputType::PushPull);

    let timer = low_level::Timer::new(p_timer);
    timer.set_counting_mode(low_level::CountingMode::CenterAlignedUpInterrupts);
    timer
}

#[allow(dead_code)]
pub fn create_6pwm_timer<'a, TIM, T1, T2, T3, T1N, T2N, T3N>(
    p1: Peri<T1>,
    p2: Peri<T2>,
    p3: Peri<T3>,
    p1n: Peri<T1N>,
    p2n: Peri<T2N>,
    p3n: Peri<T3N>,
    p_timer: Peri<'a, TIM>,
) -> low_level::Timer<'a, TIM>
where
    TIM: AdvancedInstance4Channel,
    T1: TimerPin<TIM, Ch1>,
    T2: TimerPin<TIM, Ch2>,
    T3: TimerPin<TIM, Ch3>,
    T1N: TimerComplementaryPin<TIM, Ch1>,
    T2N: TimerComplementaryPin<TIM, Ch2>,
    T3N: TimerComplementaryPin<TIM, Ch3>,
{
    let _pin1 = PwmPin::new(p1, OutputType::PushPull);
    let _pin2 = PwmPin::new(p2, OutputType::PushPull);
    let _pin3 = PwmPin::new(p3, OutputType::PushPull);

    let o = gpio::Output::new(p1n, gpio::Level::Low, Speed::VeryHigh);
    core::mem::forget(o);
    let o = gpio::Output::new(p2n, gpio::Level::Low, Speed::VeryHigh);
    core::mem::forget(o);
    let o = gpio::Output::new(p3n, gpio::Level::Low, Speed::VeryHigh);
    core::mem::forget(o);

    low_level::Timer::new(p_timer)
}

pub fn initialize<'a, TIM>(timer: &mut low_level::Timer<'a, TIM>)
where
    TIM: AdvancedInstance4Channel,
{
    timer.stop();
    [
        embassy_stm32::timer::Channel::Ch1,
        embassy_stm32::timer::Channel::Ch2,
        embassy_stm32::timer::Channel::Ch3,
    ]
    .iter()
    .for_each(|&ch| {
        timer.enable_channel(ch, true);
        timer.set_output_compare_mode(ch, low_level::OutputCompareMode::PwmMode1);
        timer.set_output_compare_preload(ch, true);
        timer.enable_complementary_channel(ch, true);
    });

    timer.regs_advanced().cr1().modify(|w| {
        w.set_opm(false); // not one-pulse
        w.set_udis(false); // do not suppress UEV
        w.set_urs(pac::timer::vals::Urs::COUNTER_ONLY); // only real over/underflow causes UEV (stable TRGO)
        w.set_cms(pac::timer::vals::Cms::CENTER_ALIGNED3);
        w.set_dir(pac::timer::vals::Dir::DOWN); // start by counting down
    });

    // set timer max compare value
    timer
        .regs_advanced()
        .arr()
        .write(|r| r.set_arr((1 << 12) - 1));

    timer
        .regs_advanced()
        .smcr()
        .modify(|w| w.set_sms(pac::timer::vals::Sms::DISABLED));
    timer
        .regs_advanced()
        .cr2()
        .modify(|w| w.set_mms(pac::timer::vals::Mms::UPDATE));

    timer.regs_advanced().rcr().write(|w| w.set_rep(0)); // no down-sample while debugging
    timer.regs_advanced().sr().write(|w| w.set_uif(false));
    timer.start();
}

pub fn set_duty_cycle<'a, TIM>(timer: &mut low_level::Timer<'a, TIM>, ch1: u32, ch2: u32, ch3: u32)
where
    TIM: GeneralInstance4Channel,
{
    timer.set_compare_value(embassy_stm32::timer::Channel::Ch1, ch1);
    timer.set_compare_value(embassy_stm32::timer::Channel::Ch2, ch2);
    timer.set_compare_value(embassy_stm32::timer::Channel::Ch3, ch3);
}
