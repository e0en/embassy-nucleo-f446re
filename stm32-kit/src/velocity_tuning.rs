use crate::adc;
use crate::adc::AdcSelector;
use crate::bldc_driver::PwmDriver;
use crate::drv8316;

use embassy_stm32::adc as stm32_adc;
use embassy_stm32::timer::AdvancedInstance4Channel;
use embassy_time::Timer;
use foc::controller::{FocController, RunMode};
use foc::encoder::EncoderReading;
use foc::pid::PID;
use foc::pwm_output::PwmOutput;
use foc::velocity_tuning::{calculate_velocity_pi, linear_regression_slope};

const N_SAMPLES: usize = 32;
const TEST_CURRENT: f32 = 0.5;
const PRE_STEP_SETTLE_MS: u64 = 200;
const POST_STEP_SETTLE_MS: u64 = 8;
const MIN_K_PLANT: f32 = 0.1;
const MIN_EFFECTIVE_TEST_CURRENT: f32 = 0.05;

fn estimate_plant_gain(acceleration: f32, average_iq: f32) -> f32 {
    let current_for_estimation = if average_iq.abs() >= MIN_EFFECTIVE_TEST_CURRENT {
        average_iq.abs()
    } else {
        TEST_CURRENT
    };
    (acceleration / current_for_estimation).abs()
}

fn estimate_velocity_from_angle(
    previous_elapsed: f32,
    previous_angle: f32,
    elapsed: f32,
    angle: f32,
) -> f32 {
    let dt = elapsed - previous_elapsed;
    if dt <= 0.0 {
        0.0
    } else {
        (angle - previous_angle) / dt
    }
}

/// Apply a torque step, measure acceleration via linear regression, compute velocity PI gains.
/// Returns None if the motor appears stalled (K_plant too low).
pub async fn tune_velocity_pi<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: Fsensor,
    bandwidth_hz: f32,
) -> Option<PID>
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> EncoderReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    async fn measure_step_response<Fsincos, Fsensor, TIM, T>(
        foc: &mut FocController<Fsincos>,
        driver: &mut PwmDriver<'_, TIM>,
        p_adc: &mut adc::DummyAdc<T>,
        csa_gain: drv8316::CsaGain,
        read_sensor: &Fsensor,
        test_current: f32,
    ) -> Option<f32>
    where
        Fsincos: Fn(f32) -> (f32, f32),
        Fsensor: Fn() -> EncoderReading,
        TIM: AdvancedInstance4Channel,
        T: stm32_adc::Instance + AdcSelector,
    {
        foc.set_target_torque(0.0);

        // Hold zero torque before the test so the sample window starts at the torque step.
        for _ in 0..PRE_STEP_SETTLE_MS {
            let encoder_reading = read_sensor();
            let (ia_raw, ib_raw, ic_raw) = p_adc.read();
            let phase_current = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
            if let Ok(duty) = foc.get_duty_cycle(&encoder_reading, phase_current) {
                driver.run(duty);
            }
            Timer::after_millis(1).await;
        }

        foc.set_target_torque(test_current);

        // Let the current loop settle before estimating the velocity plant.
        for _ in 0..POST_STEP_SETTLE_MS {
            let encoder_reading = read_sensor();
            let (ia_raw, ib_raw, ic_raw) = p_adc.read();
            let phase_current = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
            if let Ok(duty) = foc.get_duty_cycle(&encoder_reading, phase_current) {
                driver.run(duty);
            }
            Timer::after_millis(1).await;
        }

        // Collect a short velocity ramp after the current transient.
        let mut t_buf = [0.0f32; N_SAMPLES];
        let mut v_buf = [0.0f32; N_SAMPLES];
        let mut iq_sum = 0.0f32;
        let t0 = embassy_time::Instant::now();
        let mut previous_elapsed = 0.0f32;
        let mut previous_angle = 0.0f32;
        let mut velocity_samples = 0usize;

        for i in 0..N_SAMPLES {
            let encoder_reading = read_sensor();
            let (ia_raw, ib_raw, ic_raw) = p_adc.read();
            let phase_current = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
            if let Ok(duty) = foc.get_duty_cycle(&encoder_reading, phase_current) {
                driver.run(duty);
            }
            let elapsed = embassy_time::Instant::now()
                .checked_duration_since(t0)
                .map(|d| d.as_micros() as f32 / 1e6)
                .unwrap_or(0.0);
            if i > 0 {
                t_buf[velocity_samples] = elapsed;
                v_buf[velocity_samples] = estimate_velocity_from_angle(
                    previous_elapsed,
                    previous_angle,
                    elapsed,
                    encoder_reading.cumulative_angle,
                );
                velocity_samples += 1;
            }
            iq_sum += foc.state.i_q;
            previous_elapsed = elapsed;
            previous_angle = encoder_reading.cumulative_angle;
            Timer::after_millis(1).await;
        }

        foc.set_target_torque(0.0);

        if velocity_samples < 2 {
            return None;
        }

        let acceleration =
            linear_regression_slope(&t_buf[..velocity_samples], &v_buf[..velocity_samples]);
        let average_iq = iq_sum / N_SAMPLES as f32;
        let k_plant = estimate_plant_gain(acceleration, average_iq);

        if k_plant >= MIN_K_PLANT {
            Some(k_plant)
        } else {
            None
        }
    }

    let old_mode = foc.mode;
    let old_torque = foc.target.torque;

    foc.set_run_mode(RunMode::Torque);
    let positive_k_plant =
        measure_step_response(foc, driver, p_adc, csa_gain, &read_sensor, TEST_CURRENT).await;
    let negative_k_plant =
        measure_step_response(foc, driver, p_adc, csa_gain, &read_sensor, -TEST_CURRENT).await;

    // Stop motor, restore state
    foc.set_target_torque(0.0);
    foc.set_run_mode(old_mode);
    foc.set_target_torque(old_torque);

    let k_plant = match (positive_k_plant, negative_k_plant) {
        (Some(pos), Some(neg)) => 0.5 * (pos + neg),
        (Some(pos), None) => pos,
        (None, Some(neg)) => neg,
        (None, None) => return None,
    };

    Some(calculate_velocity_pi(k_plant, bandwidth_hz))
}
