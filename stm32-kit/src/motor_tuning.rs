use crate::adc;
use crate::adc::AdcSelector;
use crate::as5047p::As5047P;
use crate::bldc_driver::PwmDriver;
use crate::drv8316;

use embassy_stm32::adc as stm32_adc;
use embassy_stm32::timer::AdvancedInstance4Channel;
use embassy_time::{Duration, Instant, Timer};
use foc::angle_input::AngleInput;
use foc::controller::{FocController, RunMode};
use foc::pwm_output::PwmOutput;

pub async fn find_kv_rating<'a, Fsincos, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    sensor: &mut As5047P<'a>,
) -> f32
where
    Fsincos: Fn(f32) -> (f32, f32),
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let n_sample = 1000;
    let old_mode = foc.mode;
    let old_v = foc.target.voltage;

    // store run mode and Vq_ref
    // set to voltage mode
    foc.set_run_mode(RunMode::Voltage);
    let mut i_sample;
    let mut v_sample = [0.0f32; 5];

    for (i_voltage, v_avg) in v_sample.iter_mut().enumerate() {
        let voltage = 1.0 + i_voltage as f32 / 2.0;
        foc.set_target_voltage(voltage);
        let t0 = Instant::now();
        Timer::after_secs(1).await;

        i_sample = 0;
        while i_sample < n_sample {
            let now = Instant::now();
            if let Ok(reading) = sensor.read_async().await {
                let (ia_raw, ib_raw, ic_raw) = p_adc.read();
                let phase_current = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
                if let Ok(duty) = foc.get_duty_cycle(&reading, phase_current) {
                    driver.run(duty);
                }

                if let Some(dt) = now.checked_duration_since(t0)
                    && dt > Duration::from_secs(1)
                {
                    *v_avg += reading.velocity / n_sample as f32;
                    i_sample += 1;
                }
            }
        }
    }

    for (i_voltage, v_avg) in v_sample.iter_mut().enumerate() {
        let voltage = 1.0 + i_voltage as f32 / 2.0;
        defmt::info!("{}: {}", voltage, v_avg);
    }

    let x_mean = 1.0 + (5 - 1) as f32 / 2.0 * 0.5;
    let mut x_sum = 0.0;
    let mut y_sum = 0.0;
    for (i_voltage, v) in v_sample.iter().enumerate() {
        let voltage = 1.0 + i_voltage as f32 / 2.0;
        x_sum += voltage * (voltage - x_mean);
        y_sum += v * (voltage - x_mean);
    }
    let kv = (y_sum / x_sum).abs();

    // restore run mode and Vq_ref
    foc.set_run_mode(old_mode);
    foc.set_target_voltage(old_v);
    kv
}
