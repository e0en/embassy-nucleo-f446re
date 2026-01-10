use crate::adc;
use crate::adc::AdcSelector;
use crate::bldc_driver::PwmDriver;
use crate::drv8316;

use embassy_stm32::adc as stm32_adc;
use embassy_stm32::timer::AdvancedInstance4Channel;
use embassy_time::{Duration, Instant, Timer};
use foc::angle_input::AngleReading;
use foc::controller::{FocController, RunMode};
use foc::pwm_output::PwmOutput;

#[allow(dead_code)]
pub async fn find_kv_rating<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: Fsensor,
) -> f32
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> AngleReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let n_sample = 256;
    let old_mode = foc.mode;
    let old_v = foc.target.voltage;

    foc.set_run_mode(RunMode::Voltage);
    let mut i_sample;
    let mut v_sample = [0.0f32; 4];
    let voltage_step = 0.1;

    for (i_voltage, v_avg) in v_sample.iter_mut().enumerate() {
        let voltage = 1.0 + i_voltage as f32 * voltage_step;
        foc.set_target_voltage(voltage);
        let t0 = Instant::now();
        Timer::after_secs(1).await;

        i_sample = 0;
        while i_sample < n_sample {
            let now = Instant::now();
            let reading = read_sensor();
            let (ia_raw, ib_raw, ic_raw) = p_adc.read();
            let phase_current = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
            if let Ok(duty) = foc.get_duty_cycle(&reading, phase_current) {
                driver.run(duty);
            }

            if let Some(dt) = now.checked_duration_since(t0)
                && dt > Duration::from_secs(2)
            {
                *v_avg += reading.velocity / n_sample as f32;
                i_sample += 1;
            }
            Timer::after_millis(1).await;
        }
    }

    for (i_voltage, v_avg) in v_sample.iter_mut().enumerate() {
        let voltage = 1.0 + i_voltage as f32 * voltage_step;
        defmt::info!("{}: {}", voltage, v_avg);
    }

    let x_mean = 1.0 + (5 - 1) as f32 * voltage_step * 0.5;
    let mut x_sum = 0.0;
    let mut y_sum = 0.0;
    for (i_voltage, v) in v_sample.iter().enumerate() {
        let voltage = 1.0 + i_voltage as f32 * voltage_step;
        x_sum += voltage * (voltage - x_mean);
        y_sum += v * (voltage - x_mean);
    }
    let kv = (y_sum / x_sum).abs();

    foc.set_run_mode(old_mode);
    foc.set_target_voltage(old_v);
    kv
}
