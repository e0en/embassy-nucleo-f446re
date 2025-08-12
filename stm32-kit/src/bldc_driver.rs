use crate::pwm::set_duty_cycle;
use embassy_stm32::timer::low_level;
use foc::pwm_output::{DutyCycle3Phase, PwmOutput};

use embassy_stm32::timer::AdvancedInstance4Channel;

pub struct PwmDriver<'a, TIM: AdvancedInstance4Channel> {
    pwm_timer: low_level::Timer<'a, TIM>,
    max_pwm_duty_cycle: u32,
}

impl<'a, TIM> PwmDriver<'a, TIM>
where
    TIM: AdvancedInstance4Channel,
{
    pub fn new(pwm_timer: low_level::Timer<'a, TIM>) -> Self {
        let max_pwm_duty_cycle = pwm_timer.get_max_compare_value() + 1;
        Self {
            pwm_timer,
            max_pwm_duty_cycle,
        }
    }
}

impl<'a, TIM> PwmOutput for PwmDriver<'a, TIM>
where
    TIM: AdvancedInstance4Channel,
{
    fn run(&mut self, signal: DutyCycle3Phase) {
        let d1 = (signal.t1 * (self.max_pwm_duty_cycle as f32)) as u32;
        let d2 = (signal.t2 * (self.max_pwm_duty_cycle as f32)) as u32;
        let d3 = (signal.t3 * (self.max_pwm_duty_cycle as f32)) as u32;
        set_duty_cycle(&mut self.pwm_timer, d1, d2, d3);
    }
}
