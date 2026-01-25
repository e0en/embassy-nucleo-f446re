use crate::adc;
use crate::adc::AdcSelector;
use crate::bldc_driver::PwmDriver;
use crate::cordic;
use crate::drv8316;

use foc::angle_input::AngleReading;
use foc::controller::FocController;
use foc::pid::PID;
use foc::pwm_output::{DutyCycle3Phase, PwmOutput};

use embassy_stm32::adc as stm32_adc;
use embassy_stm32::timer::AdvancedInstance4Channel;
use embassy_time::Instant;

const IMPEDANCE_SAMPLE_COUNT: usize = 2048;
const TEST_VOLTAGE: f32 = 0.5;
const TEST_FREQUENCY_HZ: f32 = 256.0;
const RESISTANCE_TEST_VOLTAGE: f32 = 1.0;

#[derive(Clone, Copy)]
enum Axis {
    D,
    Q,
}

#[allow(dead_code)]
pub struct MotorImpedance {
    pub r_s: f32,
    pub l_d: f32,
    pub l_q: f32,
}

struct MeasurementBuffers<'a> {
    t: &'a mut [f32],
    i: &'a mut [f32],
    v: &'a mut [f32],
    t0: u64,
}

fn measure_inductance_samples<'a, Fsincos, Fsensor, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    read_sensor: &Fsensor,
    axis: Axis,
    buf: &mut MeasurementBuffers<'_>,
) where
    Fsincos: Fn(f32) -> (f32, f32),
    Fsensor: Fn() -> AngleReading,
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let n_sample = buf.t.len();
    let get_duty: fn(&mut FocController<Fsincos>, f32, f32) -> Result<DutyCycle3Phase, _> =
        match axis {
            Axis::D => FocController::get_vd_duty_cycle,
            Axis::Q => FocController::get_vq_duty_cycle,
        };

    for i in 0..(n_sample * 2) {
        let now = Instant::now().as_micros();
        let t = ((now - buf.t0) as f32) / 1e6;
        let (sin_val, _) = cordic::sincos(core::f32::consts::TAU * TEST_FREQUENCY_HZ * t);
        let v = TEST_VOLTAGE * sin_val;
        let reading = read_sensor();
        let angle = reading.angle;
        let electrical_angle = foc.to_electrical_angle(angle);
        if let Ok(duty) = get_duty(foc, v, angle) {
            driver.run(duty);
            if i >= n_sample {
                let (ia_raw, ib_raw, ic_raw) = p_adc.read();
                let phase_current = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
                let (i_q, i_d) = foc.calculate_pq_currents(phase_current, electrical_angle);
                let idx = i - n_sample;
                buf.t[idx] = t;
                buf.i[idx] = match axis {
                    Axis::D => i_d,
                    Axis::Q => i_q,
                };
                buf.v[idx] = v;
            }
        }
    }
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
    for i in 0..(n_sample * 2) {
        let reading = read_sensor();
        let angle = reading.angle;
        let electrical_angle = foc.to_electrical_angle(angle);
        if let Ok(duty) = foc.get_vd_duty_cycle(RESISTANCE_TEST_VOLTAGE, angle) {
            driver.run(duty);
            let (ia_raw, ib_raw, ic_raw) = p_adc.read();
            let phase_current = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
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
    let mut t_buffer = [0f32; IMPEDANCE_SAMPLE_COUNT];
    let mut i_buffer = [0f32; IMPEDANCE_SAMPLE_COUNT];
    let mut v_buffer = [0f32; IMPEDANCE_SAMPLE_COUNT];
    let mut buf = MeasurementBuffers {
        t: &mut t_buffer,
        i: &mut i_buffer,
        v: &mut v_buffer,
        t0: Instant::now().as_micros(),
    };

    measure_inductance_samples(
        foc,
        driver,
        p_adc,
        csa_gain,
        &read_sensor,
        Axis::D,
        &mut buf,
    );
    let l_d = calculate_inductance(buf.v, buf.i, buf.t, TEST_FREQUENCY_HZ);

    measure_inductance_samples(
        foc,
        driver,
        p_adc,
        csa_gain,
        &read_sensor,
        Axis::Q,
        &mut buf,
    );
    let l_q = calculate_inductance(buf.v, buf.i, buf.t, TEST_FREQUENCY_HZ);

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

pub fn calculate_current_pi(impedance: MotorImpedance, bandwidth: f32) -> PID {
    let omega = core::f32::consts::TAU * bandwidth;
    let kp = impedance.l_q * omega;
    let ki = impedance.r_s * omega;
    PID {
        p: kp,
        i: ki,
        d: 0.0,
    }
}

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
        let angle = test_frequency * t_buffer[i];
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
    (z_mag * sin_phi / test_frequency).abs()
}
