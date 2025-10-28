use crate::adc;
use crate::adc::AdcSelector;
use crate::as5047p::As5047P;
use crate::bldc_driver::PwmDriver;
use crate::drv8316;

use foc::angle_input::AngleInput;
use foc::controller::FocController;
use foc::pid::PID;
use foc::pwm_output::PwmOutput;

use embassy_stm32::adc as stm32_adc;
use embassy_stm32::timer::AdvancedInstance4Channel;
use embassy_time::Instant;

const IMPEDANCE_SAMPLE_COUNT: usize = 2048;

#[allow(dead_code)]
pub struct MotorImpedance {
    r_s: f32,
    l_d: f32,
    l_q: f32,
}

pub async fn find_motor_impedance<'a, Fsincos, TIM, T>(
    foc: &mut FocController<Fsincos>,
    driver: &mut PwmDriver<'a, TIM>,
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: drv8316::CsaGain,
    sensor: &mut As5047P<'a>,
) -> MotorImpedance
where
    Fsincos: Fn(f32) -> (f32, f32),
    TIM: AdvancedInstance4Channel,
    T: stm32_adc::Instance + AdcSelector,
{
    let n_sample = IMPEDANCE_SAMPLE_COUNT;
    let test_voltage = 4.0;
    let test_frequency = 512.0;
    let _t0 = Instant::now().as_micros();

    let mut t_buffer = [0f32; IMPEDANCE_SAMPLE_COUNT];
    let mut i_buffer = [0f32; IMPEDANCE_SAMPLE_COUNT];
    let mut v_buffer = [0f32; IMPEDANCE_SAMPLE_COUNT];

    // 1. use v_q = 0.0, v_d = sin(2 * pi * f * t to measure L_d)
    for i in 0..(n_sample * 2) {
        let now = Instant::now().as_micros();
        let t = ((now - _t0) as f32) / 1e6;
        let v_d = test_voltage * libm::sinf(2.0 * core::f32::consts::PI * test_frequency * t);
        if let Ok(reading) = sensor.read_async().await {
            let angle = reading.angle;
            let electrical_angle = foc.to_electrical_angle(angle);
            if let Ok(duty) = foc.get_vd_duty_cycle(v_d, angle) {
                driver.run(duty);
                if i >= n_sample {
                    let (ia_raw, ib_raw, ic_raw) = p_adc.read();
                    let phase_current =
                        drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
                    let (_, i_d) = foc.calculate_pq_currents(phase_current, electrical_angle);
                    t_buffer[i - n_sample] = t;
                    i_buffer[i - n_sample] = i_d;
                    v_buffer[i - n_sample] = v_d;
                }
            }
        }
    }

    let l_d = calculate_inductance(&v_buffer, &i_buffer, &t_buffer, test_frequency);

    // 2. use v_q = sin(2 * pi * f * t, v_d = 0.0) to measure L_q
    for i in 0..(n_sample * 2) {
        let now = Instant::now().as_micros();
        let t = ((now - _t0) as f32) / 1e6;
        let v_q = test_voltage * libm::sinf(2.0 * core::f32::consts::PI * test_frequency * t);
        if let Ok(reading) = sensor.read_async().await {
            let angle = reading.angle;
            let electrical_angle = foc.to_electrical_angle(angle);
            if let Ok(duty) = foc.get_vq_duty_cycle(v_q, angle) {
                driver.run(duty);
                if i >= n_sample {
                    let (ia_raw, ib_raw, ic_raw) = p_adc.read();
                    let phase_current =
                        drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
                    let (i_q, _) = foc.calculate_pq_currents(phase_current, electrical_angle);
                    t_buffer[i - n_sample] = t;
                    i_buffer[i - n_sample] = i_q;
                    v_buffer[i - n_sample] = v_q;
                }
            }
        }
    }
    let l_q = calculate_inductance(&v_buffer, &i_buffer, &t_buffer, test_frequency);

    // 3. use v_q = 0.0, v_d = 1.0 V to measure R_s
    let mut i_avg = 0.0;
    for i in 0..(n_sample * 2) {
        let v_d = 1.0;
        if let Ok(reading) = sensor.read_async().await {
            let angle = reading.angle;
            let electrical_angle = foc.to_electrical_angle(angle);
            if let Ok(duty) = foc.get_vd_duty_cycle(v_d, angle) {
                driver.run(duty);
                let (ia_raw, ib_raw, ic_raw) = p_adc.read();
                let phase_current = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
                let (_, i_d) = foc.calculate_pq_currents(phase_current, electrical_angle);
                if i >= n_sample {
                    i_avg += i_d / (n_sample as f32);
                }
            }
        }
    }
    let r_s = 1.0 / i_avg;

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

        vc += v * libm::cosf(angle);
        vs += v * libm::sinf(angle);
        ic += i * libm::cosf(angle);
        is += i * libm::sinf(angle);
    }

    let norm = 2.0 / (n_sample as f32);
    vc *= norm;
    vs *= norm;
    ic *= norm;
    is *= norm;
    let v_mag = libm::sqrtf(vc * vc + vs * vs);
    let i_mag = libm::sqrtf(ic * ic + is * is);
    let phi = libm::atan2f(vs, vc) - libm::atan2f(is, ic);
    let z_mag = v_mag / i_mag;

    (z_mag * libm::sinf(phi) / test_frequency).abs()
}
