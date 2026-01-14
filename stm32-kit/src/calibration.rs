use embassy_stm32::{adc as stm32_adc, timer::AdvancedInstance4Channel};
use embassy_time::Timer;
use foc::{
    controller::{FocController, clarke_transform, park_transform},
    current::PhaseCurrent,
    pwm_output::{DutyCycle3Phase, PwmOutput},
};

use crate::{
    adc::{self, AdcSelector},
    bldc_driver::PwmDriver,
    cordic::{atan2, sincos},
    drv8316::{self, CsaGain},
    read_sensor,
};

pub async fn get_average_adc<T>(
    p_adc: &mut adc::DummyAdc<T>,
    csa_gain: CsaGain,
    n: usize,
) -> PhaseCurrent
where
    T: stm32_adc::Instance + AdcSelector,
{
    let mut ia_avg = 0.0;
    let mut ib_avg = 0.0;
    let mut ic_avg = 0.0;
    for _ in 0..n {
        let (ia_raw, ib_raw, ic_raw) = p_adc.read();
        let measured = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
        ia_avg += measured.a;
        ib_avg += measured.b;
        ic_avg += measured.c;
        Timer::after_millis(1).await;
    }
    ia_avg /= n as f32;
    ib_avg /= n as f32;
    ic_avg /= n as f32;
    PhaseCurrent::new(ia_avg, ib_avg, ic_avg)
}

pub async fn get_current_offset<'a, T, TIM>(
    p_adc: &mut adc::DummyAdc<T>,
    driver: &mut PwmDriver<'a, TIM>,
    csa_gain: CsaGain,
) -> PhaseCurrent
where
    T: stm32_adc::Instance + AdcSelector,
    TIM: AdvancedInstance4Channel,
{
    driver.run(DutyCycle3Phase::zero());
    Timer::after_millis(100).await;
    get_average_adc(p_adc, csa_gain, 100).await
}

async fn ramp_and_measure_phase<T, TIM>(
    p_adc: &mut adc::DummyAdc<T>,
    driver: &mut PwmDriver<'_, TIM>,
    csa_gain: CsaGain,
    offset: PhaseCurrent,
    align_ratio: f32,
    phase_setter: impl Fn(&mut DutyCycle3Phase, f32),
) -> Option<u8>
where
    T: stm32_adc::Instance + AdcSelector,
    TIM: AdvancedInstance4Channel,
{
    let mut phase = DutyCycle3Phase::zero();
    for i in 0..100 {
        phase_setter(&mut phase, align_ratio * (i as f32) / 100.0);
        driver.run(phase);
        Timer::after_millis(1).await;
    }
    Timer::after_millis(100).await;
    let c = get_average_adc(p_adc, csa_gain, 20).await - offset;
    max_index(c.a, c.b, c.c)
}

pub async fn get_phase_mapping<'a, T, TIM>(
    p_adc: &mut adc::DummyAdc<T>,
    driver: &mut PwmDriver<'a, TIM>,
    csa_gain: CsaGain,
    offset: PhaseCurrent,
    align_ratio: f32,
) -> Option<(u8, u8, u8)>
where
    T: stm32_adc::Instance + AdcSelector,
    TIM: AdvancedInstance4Channel,
{
    let phase_1 = ramp_and_measure_phase(p_adc, driver, csa_gain, offset, align_ratio, |p, v| {
        p.t1 = v
    })
    .await?;

    let phase_2 = ramp_and_measure_phase(p_adc, driver, csa_gain, offset, align_ratio, |p, v| {
        p.t2 = v
    })
    .await?;
    if phase_2 == phase_1 {
        return None;
    }

    let phase_3 = ramp_and_measure_phase(p_adc, driver, csa_gain, offset, align_ratio, |p, v| {
        p.t3 = v
    })
    .await?;
    if phase_3 == phase_1 || phase_3 == phase_2 {
        return None;
    }

    Some((phase_1, phase_2, phase_3))
}

fn max_index(v1: f32, v2: f32, v3: f32) -> Option<u8> {
    if v1 > v2 && v1 > v3 {
        Some(0)
    } else if v2 > v1 && v2 > v3 {
        Some(1)
    } else if v3 > v1 && v3 > v2 {
        Some(2)
    } else {
        None
    }
}

pub async fn align_current<'a, T, TIM, Fsincos>(
    p_adc: &mut adc::DummyAdc<T>,
    driver: &mut PwmDriver<'a, TIM>,
    csa_gain: CsaGain,
    foc: &FocController<Fsincos>,
) -> Option<f32>
where
    T: stm32_adc::Instance + AdcSelector,
    TIM: AdvancedInstance4Channel,
    Fsincos: Fn(f32) -> (f32, f32),
{
    // assumption: mechanical angle and voltage (electrical) angle are aligned
    let a_duty = 0.1;

    let mut duty = DutyCycle3Phase::zero();
    duty.t1 = a_duty;
    driver.run(duty);
    Timer::after_millis(100).await;

    let n_max_try = 100;
    let n_measure = 100;
    let mut n_success = 0;
    let mut avg_angle = 0.0;
    for _ in 0..n_max_try {
        let (ia_raw, ib_raw, ic_raw) = p_adc.read();
        let measured = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, csa_gain);
        let mapped = foc.normalize_current(measured);
        let reading = read_sensor();
        let angle = reading.angle;
        let e_angle = foc.to_electrical_angle(angle);

        let (i_alpha, i_beta) = clarke_transform(mapped.a, mapped.b, mapped.c);
        let (i_q, i_d) = park_transform(i_alpha, i_beta, e_angle, sincos);
        let i_angle = atan2(i_q, i_d);
        avg_angle += i_angle - e_angle;
        n_success += 1;

        if n_success >= n_measure {
            break;
        }
        Timer::after_millis(1).await;
    }

    match n_success {
        0 => None,
        _ => Some(avg_angle / (n_success as f32)),
    }
}
