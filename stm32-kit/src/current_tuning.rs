use crate::adc;
use crate::adc::AdcSelector;
use crate::bldc_driver::PwmDriver;
use crate::cordic;
use crate::drv8316;
use core::hint::spin_loop;
use core::sync::atomic::Ordering;

use foc::angle_input::SensorReading;
use foc::controller::FocController;
use foc::pid::PID;
use foc::pwm_output::PwmOutput;

use embassy_stm32::adc as stm32_adc;
use embassy_stm32::timer::AdvancedInstance4Channel;
use embassy_time::Instant;
use foc::current::PhaseCurrent;
use foc::pwm_output::DutyCycle3Phase;

const IMPEDANCE_SAMPLE_COUNT: usize = 2048;
const IMPEDANCE_TEST_VOLTAGE_START: f32 = 0.1;
const IMPEDANCE_TEST_VOLTAGE_STEP: f32 = 0.1;
const INDUCTANCE_STEP_VOLTAGE_MAX: f32 = 2.0;
const RESISTANCE_TEST_VOLTAGE_MAX: f32 = 1.0;
const INDUCTANCE_TARGET_DELTA_CURRENT: f32 = 0.2;
const RESISTANCE_TARGET_CURRENT: f32 = 0.3;
const INDUCTANCE_SETTLE_SAMPLES: usize = 256;
const INDUCTANCE_STEP_SAMPLES: usize = 64;
const INDUCTANCE_MEASUREMENT_REPEATS: usize = 16;
const INDUCTANCE_TRIM_COUNT: usize = 2;
const TEST_VOLTAGE_SEARCH_SAMPLES: usize = 64;
const ADC_SAMPLE_WAIT_SPINS: usize = 100_000;

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum ImpedanceError {
    DutyGenerationFailed,
    Overcurrent,
}

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

#[inline]
fn stop_impedance_drive<TIM>(driver: &mut PwmDriver<'_, TIM>)
where
    TIM: AdvancedInstance4Channel,
{
    driver.run(DutyCycle3Phase::zero());
}

#[inline]
fn max_abs_phase_current(current: PhaseCurrent) -> f32 {
    current.a.abs().max(current.b.abs()).max(current.c.abs())
}

#[inline]
fn guard_impedance_current<TIM>(
    driver: &mut PwmDriver<'_, TIM>,
    phase_current: PhaseCurrent,
    max_test_current: f32,
) -> Result<(), ImpedanceError>
where
    TIM: AdvancedInstance4Channel,
{
    let peak_current = max_abs_phase_current(phase_current);
    if peak_current > max_test_current {
        stop_impedance_drive(driver);
        defmt::warn!(
            "Impedance current limit hit: peak={}A limit={}A phase=({},{},{})",
            peak_current,
            max_test_current,
            phase_current.a,
            phase_current.b,
            phase_current.c
        );
        return Err(ImpedanceError::Overcurrent);
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn measure_d_axis_current<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: &Fsensor,
    v_d: f32,
    previous_sample_seq: u32,
    max_test_current: f32,
) -> Result<(f32, u32), ImpedanceError>
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> SensorReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let angle = read_sensor().output_phase;
    let duty = foc
        .get_vd_duty_cycle(v_d, angle)
        .map_err(|_| ImpedanceError::DutyGenerationFailed)?;
    driver.run(duty);
    let (phase_current, sample_seq) =
        read_synced_phase_current(p_adc, csa_gain, previous_sample_seq);
    guard_impedance_current(driver, phase_current, max_test_current)?;
    let electrical_angle = foc.to_electrical_angle(angle);
    let (_, i_d) = foc.calculate_pq_currents(phase_current, electrical_angle);
    Ok((i_d, sample_seq))
}

#[allow(clippy::too_many_arguments)]
fn measure_q_axis_current<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: &Fsensor,
    v_q: f32,
    previous_sample_seq: u32,
    max_test_current: f32,
) -> Result<(f32, u32), ImpedanceError>
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> SensorReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let angle = read_sensor().output_phase;
    let duty = foc
        .get_vq_duty_cycle(v_q, angle)
        .map_err(|_| ImpedanceError::DutyGenerationFailed)?;
    driver.run(duty);
    let (phase_current, sample_seq) =
        read_synced_phase_current(p_adc, csa_gain, previous_sample_seq);
    guard_impedance_current(driver, phase_current, max_test_current)?;
    let electrical_angle = foc.to_electrical_angle(angle);
    let (i_q, _) = foc.calculate_pq_currents(phase_current, electrical_angle);
    Ok((i_q, sample_seq))
}

fn trimmed_mean(values: &mut [f32]) -> f32 {
    for idx in 1..values.len() {
        let value = values[idx];
        let mut insert_idx = idx;
        while insert_idx > 0 && values[insert_idx - 1] > value {
            values[insert_idx] = values[insert_idx - 1];
            insert_idx -= 1;
        }
        values[insert_idx] = value;
    }
    let trim = INDUCTANCE_TRIM_COUNT.min(values.len() / 2);
    let start = trim;
    let end = values.len() - trim;

    let mut sum = 0.0;
    for value in &values[start..end] {
        sum += *value;
    }
    sum / ((end - start) as f32)
}

fn measure_d_axis_inductance_once<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: &Fsensor,
    step_voltage: f32,
    max_test_current: f32,
) -> Result<f32, ImpedanceError>
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> SensorReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let mut baseline = 0.0;
    let mut sample_seq = adc::SAMPLE_SEQ.load(Ordering::Relaxed);
    for _ in 0..INDUCTANCE_SETTLE_SAMPLES {
        let (i_d, next_sample_seq) = measure_d_axis_current(
            foc,
            driver,
            p_adc,
            csa_gain,
            read_sensor,
            0.0,
            sample_seq,
            max_test_current,
        )?;
        sample_seq = next_sample_seq;
        baseline += i_d / (INDUCTANCE_SETTLE_SAMPLES as f32);
    }

    let mut t_samples = [0.0f32; INDUCTANCE_STEP_SAMPLES];
    let mut i_samples = [0.0f32; INDUCTANCE_STEP_SAMPLES];
    let t0 = Instant::now().as_micros();

    for idx in 0..INDUCTANCE_STEP_SAMPLES {
        let (i_d, next_sample_seq) = measure_d_axis_current(
            foc,
            driver,
            p_adc,
            csa_gain,
            read_sensor,
            step_voltage,
            sample_seq,
            max_test_current,
        )?;
        sample_seq = next_sample_seq;
        t_samples[idx] = ((Instant::now().as_micros() - t0) as f32) / 1e6;
        i_samples[idx] = i_d - baseline;
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
    Ok((step_voltage / slope.abs()).abs())
}

fn measure_d_axis_inductance<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: &Fsensor,
    step_voltage: f32,
    max_test_current: f32,
) -> Result<f32, ImpedanceError>
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> SensorReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let mut measurements = [0.0; INDUCTANCE_MEASUREMENT_REPEATS];
    for measurement in &mut measurements {
        *measurement = measure_d_axis_inductance_once(
            foc,
            driver,
            p_adc,
            csa_gain,
            read_sensor,
            step_voltage,
            max_test_current,
        )?;
    }
    Ok(trimmed_mean(&mut measurements))
}

fn measure_q_axis_inductance_once<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: &Fsensor,
    step_voltage: f32,
    max_test_current: f32,
) -> Result<f32, ImpedanceError>
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> SensorReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let mut baseline = 0.0;
    let mut sample_seq = adc::SAMPLE_SEQ.load(Ordering::Relaxed);
    for _ in 0..INDUCTANCE_SETTLE_SAMPLES {
        let (i_q, next_sample_seq) = measure_q_axis_current(
            foc,
            driver,
            p_adc,
            csa_gain,
            read_sensor,
            0.0,
            sample_seq,
            max_test_current,
        )?;
        sample_seq = next_sample_seq;
        baseline += i_q / (INDUCTANCE_SETTLE_SAMPLES as f32);
    }

    let mut t_samples = [0.0f32; INDUCTANCE_STEP_SAMPLES];
    let mut i_samples = [0.0f32; INDUCTANCE_STEP_SAMPLES];
    let t0 = Instant::now().as_micros();

    for idx in 0..INDUCTANCE_STEP_SAMPLES {
        let (i_q, next_sample_seq) = measure_q_axis_current(
            foc,
            driver,
            p_adc,
            csa_gain,
            read_sensor,
            step_voltage,
            sample_seq,
            max_test_current,
        )?;
        sample_seq = next_sample_seq;
        t_samples[idx] = ((Instant::now().as_micros() - t0) as f32) / 1e6;
        i_samples[idx] = i_q - baseline;
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
    Ok((step_voltage / slope.abs()).abs())
}

fn measure_q_axis_inductance<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: &Fsensor,
    step_voltage: f32,
    max_test_current: f32,
) -> Result<f32, ImpedanceError>
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> SensorReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let mut measurements = [0.0; INDUCTANCE_MEASUREMENT_REPEATS];
    for measurement in &mut measurements {
        *measurement = measure_q_axis_inductance_once(
            foc,
            driver,
            p_adc,
            csa_gain,
            read_sensor,
            step_voltage,
            max_test_current,
        )?;
    }
    Ok(trimmed_mean(&mut measurements))
}

#[allow(clippy::too_many_arguments)]
fn measure_resistance<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: &Fsensor,
    n_sample: usize,
    test_voltage: f32,
    max_test_current: f32,
) -> Result<f32, ImpedanceError>
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> SensorReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let mut sample_seq = adc::SAMPLE_SEQ.load(Ordering::Relaxed);
    let mut i_pos_avg = 0.0;
    let mut i_neg_avg = 0.0;

    for i in 0..(n_sample * 2) {
        let (i_d, next_sample_seq) = measure_d_axis_current(
            foc,
            driver,
            p_adc,
            csa_gain,
            read_sensor,
            test_voltage,
            sample_seq,
            max_test_current,
        )?;
        sample_seq = next_sample_seq;
        if i >= n_sample {
            i_pos_avg += i_d / (n_sample as f32);
        }
    }

    for i in 0..(n_sample * 2) {
        let (i_d, next_sample_seq) = measure_d_axis_current(
            foc,
            driver,
            p_adc,
            csa_gain,
            read_sensor,
            -test_voltage,
            sample_seq,
            max_test_current,
        )?;
        sample_seq = next_sample_seq;
        if i >= n_sample {
            i_neg_avg += i_d / (n_sample as f32);
        }
    }

    Ok((2.0 * test_voltage) / (i_pos_avg - i_neg_avg))
}

fn find_inductance_step_voltage<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: &Fsensor,
    max_test_current: f32,
) -> Result<f32, ImpedanceError>
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> SensorReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let mut voltage = IMPEDANCE_TEST_VOLTAGE_START;
    while voltage <= INDUCTANCE_STEP_VOLTAGE_MAX {
        let mut baseline = 0.0;
        let mut sample_seq = adc::SAMPLE_SEQ.load(Ordering::Relaxed);
        for _ in 0..INDUCTANCE_SETTLE_SAMPLES {
            let (i_d, next_sample_seq) = measure_d_axis_current(
                foc,
                driver,
                p_adc,
                csa_gain,
                read_sensor,
                0.0,
                sample_seq,
                max_test_current,
            )?;
            sample_seq = next_sample_seq;
            baseline += i_d / (INDUCTANCE_SETTLE_SAMPLES as f32);
        }

        let mut peak_delta = 0.0f32;
        let mut peak_current = 0.0f32;
        for _ in 0..INDUCTANCE_STEP_SAMPLES {
            let (i_d, next_sample_seq) = measure_d_axis_current(
                foc,
                driver,
                p_adc,
                csa_gain,
                read_sensor,
                voltage,
                sample_seq,
                max_test_current,
            )?;
            sample_seq = next_sample_seq;
            let delta = (i_d - baseline).abs();
            peak_delta = peak_delta.max(delta);
            peak_current = peak_current.max(i_d.abs());
        }
        stop_impedance_drive(driver);

        if peak_current <= max_test_current && peak_delta >= INDUCTANCE_TARGET_DELTA_CURRENT {
            return Ok(voltage);
        }

        voltage += IMPEDANCE_TEST_VOLTAGE_STEP;
    }

    Ok(INDUCTANCE_STEP_VOLTAGE_MAX)
}

fn find_resistance_test_voltage<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: &Fsensor,
    max_test_current: f32,
) -> Result<f32, ImpedanceError>
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> SensorReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let mut voltage = IMPEDANCE_TEST_VOLTAGE_START;
    while voltage <= RESISTANCE_TEST_VOLTAGE_MAX {
        let mut sample_seq = adc::SAMPLE_SEQ.load(Ordering::Relaxed);
        let mut avg_current = 0.0f32;
        let mut peak_current = 0.0f32;

        for i in 0..(TEST_VOLTAGE_SEARCH_SAMPLES * 2) {
            let (i_d, next_sample_seq) = measure_d_axis_current(
                foc,
                driver,
                p_adc,
                csa_gain,
                read_sensor,
                voltage,
                sample_seq,
                max_test_current,
            )?;
            sample_seq = next_sample_seq;
            if i >= TEST_VOLTAGE_SEARCH_SAMPLES {
                avg_current += i_d.abs() / (TEST_VOLTAGE_SEARCH_SAMPLES as f32);
            }
            peak_current = peak_current.max(i_d.abs());
        }
        stop_impedance_drive(driver);

        if peak_current <= max_test_current && avg_current >= RESISTANCE_TARGET_CURRENT {
            return Ok(voltage);
        }

        voltage += IMPEDANCE_TEST_VOLTAGE_STEP;
    }

    Ok(RESISTANCE_TEST_VOLTAGE_MAX)
}

pub async fn find_motor_impedance<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: Fsensor,
    max_test_current: f32,
    mut shutdown: impl FnMut(),
) -> Result<MotorImpedance, ImpedanceError>
where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> SensorReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let inductance_step_voltage = match find_inductance_step_voltage(
        foc,
        driver,
        p_adc,
        csa_gain,
        &read_sensor,
        max_test_current,
    ) {
        Ok(voltage) => voltage,
        Err(err) => {
            stop_impedance_drive(driver);
            shutdown();
            return Err(err);
        }
    };
    let resistance_test_voltage = match find_resistance_test_voltage(
        foc,
        driver,
        p_adc,
        csa_gain,
        &read_sensor,
        max_test_current,
    ) {
        Ok(voltage) => voltage,
        Err(err) => {
            stop_impedance_drive(driver);
            shutdown();
            return Err(err);
        }
    };
    defmt::info!(
        "Impedance test voltages: inductance={} resistance={}",
        inductance_step_voltage,
        resistance_test_voltage
    );

    let l_d = match measure_d_axis_inductance(
        foc,
        driver,
        p_adc,
        csa_gain,
        &read_sensor,
        inductance_step_voltage,
        max_test_current,
    ) {
        Ok(value) => value,
        Err(err) => {
            stop_impedance_drive(driver);
            shutdown();
            return Err(err);
        }
    };
    let l_q = match measure_q_axis_inductance(
        foc,
        driver,
        p_adc,
        csa_gain,
        &read_sensor,
        inductance_step_voltage,
        max_test_current,
    ) {
        Ok(value) => value,
        Err(err) => {
            stop_impedance_drive(driver);
            shutdown();
            return Err(err);
        }
    };

    let r_s = match measure_resistance(
        foc,
        driver,
        p_adc,
        csa_gain,
        &read_sensor,
        IMPEDANCE_SAMPLE_COUNT,
        resistance_test_voltage,
        max_test_current,
    ) {
        Ok(value) => value,
        Err(err) => {
            stop_impedance_drive(driver);
            shutdown();
            return Err(err);
        }
    };
    stop_impedance_drive(driver);
    Ok(MotorImpedance { l_d, l_q, r_s })
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
