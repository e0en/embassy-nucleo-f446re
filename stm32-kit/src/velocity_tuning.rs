use crate::adc;
use crate::adc::AdcSelector;
use crate::bldc_driver::PwmDriver;
use crate::drv8316;

use embassy_stm32::adc as stm32_adc;
use embassy_stm32::timer::AdvancedInstance4Channel;
use embassy_time::Timer;
use foc::angle_input::AngleReading;
use foc::controller::{FocController, RunMode};
use foc::pid::PID;
use foc::pwm_output::PwmOutput;
use foc::velocity_tuning::{calculate_velocity_pi, linear_regression_slope};

const N_SAMPLES: usize = 64;
const TEST_CURRENT: f32 = 0.5;
const PRE_STEP_SETTLE_MS: u64 = 200;
const MIN_K_PLANT: f32 = 0.1;
const MIN_EFFECTIVE_TEST_CURRENT: f32 = 0.05;

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
    Fsensor: Fn() -> AngleReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let old_mode = foc.mode;
    let old_torque = foc.target.torque;

    foc.set_run_mode(RunMode::Torque);
    foc.set_target_torque(0.0);

    // Hold zero torque before the test so the sample window starts at the torque step.
    for _ in 0..PRE_STEP_SETTLE_MS {
        let reading = read_sensor();
        let (ia_raw, ib_raw, ic_raw) = p_adc.read();
        let phase_current = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
        if let Ok(duty) = foc.get_duty_cycle(&reading, phase_current) {
            driver.run(duty);
        }
        Timer::after_millis(1).await;
    }

    foc.set_target_torque(TEST_CURRENT);

    // Collect the initial velocity ramp immediately after the torque step.
    let mut t_buf = [0.0f32; N_SAMPLES];
    let mut v_buf = [0.0f32; N_SAMPLES];
    let mut iq_sum = 0.0f32;
    let t0 = embassy_time::Instant::now();

    for i in 0..N_SAMPLES {
        let reading = read_sensor();
        let (ia_raw, ib_raw, ic_raw) = p_adc.read();
        let phase_current = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
        if let Ok(duty) = foc.get_duty_cycle(&reading, phase_current) {
            driver.run(duty);
        }
        let elapsed = embassy_time::Instant::now()
            .checked_duration_since(t0)
            .map(|d| d.as_micros() as f32 / 1e6)
            .unwrap_or(0.0);
        t_buf[i] = elapsed;
        v_buf[i] = reading.velocity;
        iq_sum += foc.state.i_q;
        Timer::after_millis(1).await;
    }

    // Stop motor, restore state
    foc.set_target_torque(0.0);
    foc.set_run_mode(old_mode);
    foc.set_target_torque(old_torque);

    let acceleration = linear_regression_slope(&t_buf, &v_buf);
    let effective_test_current = (iq_sum / N_SAMPLES as f32).abs();
    let current_for_estimation = if effective_test_current >= MIN_EFFECTIVE_TEST_CURRENT {
        effective_test_current
    } else {
        TEST_CURRENT
    };
    let k_plant = acceleration / current_for_estimation;

    if k_plant.abs() < MIN_K_PLANT {
        return None;
    }

    Some(calculate_velocity_pi(k_plant.abs(), bandwidth_hz))
}
