use crate::adc;
use crate::adc::AdcSelector;
use crate::bldc_driver::PwmDriver;
use crate::cordic;
use crate::drv8316;
use core::hint::spin_loop;
use core::sync::atomic::Ordering;

use foc::angle_input::AngleReading;
use foc::controller::FocController;
use foc::pid::PID;
use foc::pwm_output::PwmOutput;

use embassy_stm32::adc as stm32_adc;
use embassy_stm32::timer::AdvancedInstance4Channel;
use embassy_time::Instant;

const IMPEDANCE_SAMPLE_COUNT: usize = 2048;
const INDUCTANCE_STEP_VOLTAGE: f32 = 2.0;
const INDUCTANCE_SETTLE_SAMPLES: usize = 256;
const INDUCTANCE_STEP_SAMPLES: usize = 64;
const RESISTANCE_TEST_VOLTAGE: f32 = 1.0;
const ADC_SAMPLE_WAIT_SPINS: usize = 100_000;

#[allow(dead_code)]
pub struct MotorImpedance {
    pub r_s: f32,
    pub l_d: f32,
    pub l_q: f32,
}

#[allow(dead_code)]
pub struct CurrentPidGains {
    pub p: f32,
    pub i: f32,
    pub d: PID,
    pub q: PID,
}

#[inline]
fn wait_for_next_adc_sample(previous_sample_seq: u32) -> u32 {
    let mut sample_seq = previous_sample_seq;
    for _ in 0..ADC_SAMPLE_WAIT_SPINS {
        sample_seq = adc::SAMPLE_SEQ.load(Ordering::Relaxed);
        if sample_seq != previous_sample_seq {
            break;
        }
        spin_loop();
    }
    sample_seq
}

#[inline]
fn read_synced_phase_current<T>(
    p_adc: &adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    previous_sample_seq: u32,
) -> (foc::current::PhaseCurrent, u32)
where
    T: stm32_adc::Instance + AdcSelector,
{
    let sample_seq = wait_for_next_adc_sample(previous_sample_seq);
    let (ia_raw, ib_raw, ic_raw) = p_adc.read();
    (
        drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain),
        sample_seq,
    )
}

fn measure_d_axis_current<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: &Fsensor,
    v_d: f32,
    previous_sample_seq: u32,
) -> Option<(f32, u32)>
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> AngleReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let angle = read_sensor().angle;
    let duty = foc.get_vd_duty_cycle(v_d, angle).ok()?;
    driver.run(duty);
    let (phase_current, sample_seq) =
        read_synced_phase_current(p_adc, csa_gain, previous_sample_seq);
    let electrical_angle = foc.to_electrical_angle(read_sensor().angle);
    let (_, i_d) = foc.calculate_pq_currents(phase_current, electrical_angle);
    Some((i_d, sample_seq))
}

fn measure_d_axis_inductance<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: &Fsensor,
) -> f32
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> AngleReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let mut baseline = 0.0;
    let mut sample_seq = adc::SAMPLE_SEQ.load(Ordering::Relaxed);
    for _ in 0..INDUCTANCE_SETTLE_SAMPLES {
        if let Some((i_d, next_sample_seq)) =
            measure_d_axis_current(foc, driver, p_adc, csa_gain, read_sensor, 0.0, sample_seq)
        {
            sample_seq = next_sample_seq;
            baseline += i_d / (INDUCTANCE_SETTLE_SAMPLES as f32);
        }
    }

    let mut t_samples = [0.0f32; INDUCTANCE_STEP_SAMPLES];
    let mut i_samples = [0.0f32; INDUCTANCE_STEP_SAMPLES];
    let t0 = Instant::now().as_micros();

    for idx in 0..INDUCTANCE_STEP_SAMPLES {
        if let Some((i_d, next_sample_seq)) = measure_d_axis_current(
            foc,
            driver,
            p_adc,
            csa_gain,
            read_sensor,
            INDUCTANCE_STEP_VOLTAGE,
            sample_seq,
        ) {
            sample_seq = next_sample_seq;
            t_samples[idx] = ((Instant::now().as_micros() - t0) as f32) / 1e6;
            i_samples[idx] = i_d - baseline;
        }
    }

    let mut t_mean = 0.0;
    let mut i_mean = 0.0;
    for idx in 0..INDUCTANCE_STEP_SAMPLES {
        t_mean += t_samples[idx] / (INDUCTANCE_STEP_SAMPLES as f32);
        i_mean += i_samples[idx] / (INDUCTANCE_STEP_SAMPLES as f32);
    }

    let mut cov_ti = 0.0;
    let mut var_t = 0.0;
    for idx in 0..INDUCTANCE_STEP_SAMPLES {
        let dt = t_samples[idx] - t_mean;
        cov_ti += dt * (i_samples[idx] - i_mean);
        var_t += dt * dt;
    }

    let slope = cov_ti / var_t.max(f32::EPSILON);
    (INDUCTANCE_STEP_VOLTAGE / slope.abs()).abs()
}

fn measure_resistance<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: &Fsensor,
    n_sample: usize,
) -> f32
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> AngleReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let mut i_avg = 0.0;
    let mut sample_seq = adc::SAMPLE_SEQ.load(Ordering::Relaxed);
    for i in 0..(n_sample * 2) {
        let reading = read_sensor();
        let angle = reading.angle;
        if let Ok(duty) = foc.get_vd_duty_cycle(RESISTANCE_TEST_VOLTAGE, angle) {
            driver.run(duty);
            let (phase_current, next_sample_seq) =
                read_synced_phase_current(p_adc, csa_gain, sample_seq);
            sample_seq = next_sample_seq;
            let electrical_angle = foc.to_electrical_angle(read_sensor().angle);
            let (_, i_d) = foc.calculate_pq_currents(phase_current, electrical_angle);
            if i >= n_sample {
                i_avg += i_d / (n_sample as f32);
            }
        }
    }
    RESISTANCE_TEST_VOLTAGE / i_avg
}

pub async fn find_motor_impedance<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: Fsensor,
) -> MotorImpedance
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> AngleReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let l_d = measure_d_axis_inductance(foc, driver, p_adc, csa_gain, &read_sensor);
    let l_q = l_d;

    let r_s = measure_resistance(
        foc,
        driver,
        p_adc,
        csa_gain,
        &read_sensor,
        IMPEDANCE_SAMPLE_COUNT,
    );
    MotorImpedance { l_d, l_q, r_s }
}

pub fn calculate_current_pi(impedance: MotorImpedance, bandwidth: f32) -> CurrentPidGains {
    let omega = core::f32::consts::TAU * bandwidth;
    let ki = impedance.r_s * omega;
    let q = PID {
        p: impedance.l_q * omega,
        i: ki,
        d: 0.0,
    };
    let d = PID {
        p: impedance.l_d * omega,
        i: ki,
        d: 0.0,
    };
    CurrentPidGains {
        p: q.p,
        i: q.i,
        d,
        q,
    }
}

#[allow(dead_code)]
pub fn calculate_inductance(
    v_buffer: &[f32],
    i_buffer: &[f32],
    t_buffer: &[f32],
    test_frequency: f32,
) -> f32 {
    let n_sample = v_buffer.len();

    let mut v_mean = 0.0;
    let mut i_mean = 0.0;
    for i in 0..n_sample {
        v_mean += v_buffer[i] / (n_sample as f32);
        i_mean += i_buffer[i] / (n_sample as f32);
    }

    let mut vc = 0.0;
    let mut vs = 0.0;
    let mut ic = 0.0;
    let mut is = 0.0;
    for i in 0..n_sample {
        let angle = core::f32::consts::TAU * test_frequency * t_buffer[i];
        let v = v_buffer[i] - v_mean;
        let i = i_buffer[i] - i_mean;

        let (sin_a, cos_a) = cordic::sincos(angle);
        vc += v * cos_a;
        vs += v * sin_a;
        ic += i * cos_a;
        is += i * sin_a;
    }

    let norm = 2.0 / (n_sample as f32);
    vc *= norm;
    vs *= norm;
    ic *= norm;
    is *= norm;
    let v_mag = cordic::sqrt_scaled(vc * vc + vs * vs);
    let i_mag = cordic::sqrt_scaled(ic * ic + is * is);
    let phi = cordic::atan2(vs, vc) - cordic::atan2(is, ic);
    let z_mag = v_mag / i_mag;

    let (sin_phi, _) = cordic::sincos(phi);
    let omega = core::f32::consts::TAU * test_frequency;
    (z_mag * sin_phi / omega).abs()
}
