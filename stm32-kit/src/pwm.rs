use embassy_stm32::Peri;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::timer::{
    Channel1Pin, Channel2Pin, Channel3Pin, GeneralInstance4Channel, low_level, simple_pwm::PwmPin,
};

pub fn create_timer<'a, TIM, T1, T2, T3>(
    p1: Peri<T1>,
    p2: Peri<T2>,
    p3: Peri<T3>,
    p_timer: Peri<'a, TIM>,
) -> low_level::Timer<'a, TIM>
where
    TIM: GeneralInstance4Channel,
    T1: Channel1Pin<TIM>,
    T2: Channel2Pin<TIM>,
    T3: Channel3Pin<TIM>,
{
    let _pin1 = PwmPin::new_ch1(p1, OutputType::PushPull);
    let _pin2 = PwmPin::new_ch2(p2, OutputType::PushPull);
    let _pin3 = PwmPin::new_ch3(p3, OutputType::PushPull);

    let timer = low_level::Timer::new(p_timer);
    timer.set_counting_mode(low_level::CountingMode::CenterAlignedUpInterrupts);
    timer
}

pub fn initialize<'a, TIM>(timer: &mut low_level::Timer<'a, TIM>)
where
    TIM: GeneralInstance4Channel,
{
    timer.enable_outputs();
    timer.start();

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
    });
}

pub fn set_duty_cycle<'a, TIM>(timer: &mut low_level::Timer<'a, TIM>, ch1: u32, ch2: u32, ch3: u32)
where
    TIM: GeneralInstance4Channel,
{
    timer.set_compare_value(embassy_stm32::timer::Channel::Ch1, ch1);
    timer.set_compare_value(embassy_stm32::timer::Channel::Ch2, ch2);
    timer.set_compare_value(embassy_stm32::timer::Channel::Ch3, ch3);
}
