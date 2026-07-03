use defmt::{error, info, warn};
use embassy_stm32::{flash::Flash, gpio, peripherals};
use embassy_time::{Duration, Timer};
use foc::{
    pwm_output::{DutyCycle3Phase, PwmOutput},
    svpwm,
};

use crate::{
    adc::{self, measure_vm_sense},
    bldc_driver::PwmDriver,
    calibration,
    cordic::sincos,
    current_tuning, drv8316, drv8316t,
    encoder_correction::{runtime_snapshot, set_runtime_correction, wrap_pm_pi},
    flash_config, foc_isr, motor_tuning, velocity_tuning,
};

use super::{
    CURRENT_SENSE_GAIN, DEFAULT_MOTOR_CAN_ID, FocControllerType,
    encoder::{apply_secondary_zero_offset, read_sensor},
    foc_setup::{MotorTimer, create_foc_controller},
    tuning::{
        ALIGN_VOLTAGE, ALIGN_VOLTAGE_SEARCH_DELTA_EPSILON_RAD,
        ALIGN_VOLTAGE_SEARCH_MIN_DOMINANT_STEPS, ALIGN_VOLTAGE_SEARCH_MIN_NET_ANGLE_RAD,
        ALIGN_VOLTAGE_SEARCH_SETTLE_MS, ALIGN_VOLTAGE_SEARCH_START, ALIGN_VOLTAGE_SEARCH_STEP,
        ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_MS, ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_RAD,
        ALIGN_VOLTAGE_SEARCH_SWEEP_STEPS, CURRENT_PI_FREQUENCY, IMPEDANCE_TUNING_MAX_CURRENT,
        PHASE_MAPPING_TOLERANCE, VELOCITY_PI_FREQUENCY,
    },
};

async fn run_motor_calibration(
    foc: &mut FocControllerType,
    p_adc: &mut adc::DummyAdc<peripherals::ADC1>,
    driver: &mut PwmDriver<'_, peripherals::TIM1>,
    gate_driver: &mut drv8316t::Drv8316T<'static>,
    csa_gain: drv8316::CsaGain,
) -> Result<f32, &'static str> {
    let current_offset = calibration::get_current_offset(p_adc, driver, csa_gain).await;
    if let Some(m) = calibration::get_phase_mapping(
        p_adc,
        driver,
        csa_gain,
        current_offset,
        PHASE_MAPPING_TOLERANCE,
    )
    .await
    {
        info!("phase mapping = {} {} {}", m.0, m.1, m.2);
        foc.current_mapping = m;
    } else {
        return Err("Current phase mapping failed");
    }
    foc.set_current_offset(current_offset);

    let align_voltage = find_minimum_align_voltage(driver, measure_vm_sense()).await?;
    info!("Using align voltage {}", align_voltage);
    let wait_seconds = async |s: f32| Timer::after(Duration::from_millis((s * 1e3) as u64)).await;

    match foc
        .detect_pole_pairs(align_voltage, read_sensor, |d| driver.run(d), wait_seconds)
        .await
    {
        Ok(pp) => info!("Detected {} pole pairs", pp),
        Err(_) => warn!(
            "Pole pair detection failed, using default: {}",
            foc.motor.pole_pair_count
        ),
    }

    let set_motor = |d: DutyCycle3Phase| driver.run(d);
    let wait_seconds = async |s: f32| Timer::after(Duration::from_millis((s * 1e3) as u64)).await;

    if foc
        .align_sensor(align_voltage, read_sensor, set_motor, wait_seconds)
        .await
        .is_err()
    {
        return Err("Sensor align failed");
    }

    if let Some(offset) =
        calibration::align_current(p_adc, driver, csa_gain, foc, align_voltage).await
    {
        foc.set_current_phase_bias(offset);
        info!("current phase bias = {}", offset);

        let impedance = current_tuning::find_motor_impedance(
            foc,
            driver,
            p_adc,
            csa_gain,
            read_sensor,
            IMPEDANCE_TUNING_MAX_CURRENT,
            || gate_driver.turn_off(),
        )
        .await
        .map_err(|err| match err {
            current_tuning::ImpedanceError::Overcurrent => {
                "Motor impedance measurement overcurrent"
            }
            current_tuning::ImpedanceError::DutyGenerationFailed => {
                "Motor impedance measurement failed"
            }
        })?;
        foc.motor.phase_resistance = impedance.r_s;
        info!(
            "R_s = {}, L_q = {}, L_d = {}",
            impedance.r_s, impedance.l_q, impedance.l_d
        );
        let current_gain = current_tuning::calculate_current_pi(impedance, CURRENT_PI_FREQUENCY);
        foc.set_current_d_kp(current_gain.d.p);
        foc.set_current_d_ki(current_gain.d.i);
        foc.set_current_q_kp(current_gain.q.p);
        foc.set_current_q_ki(current_gain.q.i);
        info!(
            "current d: kp={}, ki={} / q: kp={}, ki={}",
            current_gain.d.p, current_gain.d.i, current_gain.q.p, current_gain.q.i
        );
    }

    Ok(align_voltage)
}

struct AlignSweep {
    positive_steps: usize,
    negative_steps: usize,
    net_angle: f32,
}

impl AlignSweep {
    fn dominant_positive(&self) -> bool {
        self.positive_steps > self.negative_steps
    }

    fn dominant_steps(&self) -> usize {
        if self.dominant_positive() {
            self.positive_steps
        } else {
            self.negative_steps
        }
    }
}

async fn measure_align_sweep(
    driver: &mut PwmDriver<'_, peripherals::TIM1>,
    candidate_voltage: f32,
    psu_voltage: f32,
    electrical_angle: &mut f32,
    previous_angle: &mut f32,
    direction: f32,
) -> Result<AlignSweep, &'static str> {
    let mut positive_steps = 0;
    let mut negative_steps = 0;
    let mut net_angle = 0.0;

    for _ in 0..ALIGN_VOLTAGE_SEARCH_SWEEP_STEPS {
        *electrical_angle += direction * ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_RAD;
        let signal = svpwm(
            candidate_voltage,
            0.0,
            *electrical_angle,
            psu_voltage,
            sincos,
        )
        .map_err(|_| "Failed to generate align search sweep signal")?;
        driver.run(signal);
        Timer::after_millis(ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_MS).await;
        let angle = read_sensor().phase;
        let delta = wrap_pm_pi(angle - *previous_angle);
        if delta >= ALIGN_VOLTAGE_SEARCH_DELTA_EPSILON_RAD {
            positive_steps += 1;
        } else if delta <= -ALIGN_VOLTAGE_SEARCH_DELTA_EPSILON_RAD {
            negative_steps += 1;
        }
        net_angle += delta;
        *previous_angle = angle;
    }

    Ok(AlignSweep {
        positive_steps,
        negative_steps,
        net_angle,
    })
}

async fn find_minimum_align_voltage(
    driver: &mut PwmDriver<'_, peripherals::TIM1>,
    psu_voltage: f32,
) -> Result<f32, &'static str> {
    const LOCK_ANGLE: f32 = 3.0 * core::f32::consts::FRAC_PI_2;

    if psu_voltage <= 0.0 {
        return Err("Invalid PSU voltage for align search");
    }

    let mut candidate_voltage = ALIGN_VOLTAGE_SEARCH_START;
    while candidate_voltage <= ALIGN_VOLTAGE {
        let lock_signal = svpwm(candidate_voltage, 0.0, LOCK_ANGLE, psu_voltage, sincos)
            .map_err(|_| "Failed to generate align search lock signal")?;
        driver.run(lock_signal);
        Timer::after_millis(ALIGN_VOLTAGE_SEARCH_SETTLE_MS).await;

        let mut electrical_angle = LOCK_ANGLE;
        let mut previous_angle = read_sensor().phase;
        let forward_sweep = measure_align_sweep(
            driver,
            candidate_voltage,
            psu_voltage,
            &mut electrical_angle,
            &mut previous_angle,
            1.0,
        )
        .await?;
        let backward_sweep = measure_align_sweep(
            driver,
            candidate_voltage,
            psu_voltage,
            &mut electrical_angle,
            &mut previous_angle,
            -1.0,
        )
        .await?;

        driver.run(DutyCycle3Phase::zero());
        let forward_dominant_positive = forward_sweep.dominant_positive();
        let backward_dominant_positive = backward_sweep.dominant_positive();
        let forward_dominant_steps = forward_sweep.dominant_steps();
        let backward_dominant_steps = backward_sweep.dominant_steps();
        info!(
            "Align search {}V: fwd(+/-)={}/{} fwd_net={} bwd(+/-)={}/{} bwd_net={}",
            candidate_voltage,
            forward_sweep.positive_steps,
            forward_sweep.negative_steps,
            forward_sweep.net_angle,
            backward_sweep.positive_steps,
            backward_sweep.negative_steps,
            backward_sweep.net_angle
        );

        if forward_dominant_steps >= ALIGN_VOLTAGE_SEARCH_MIN_DOMINANT_STEPS
            && backward_dominant_steps >= ALIGN_VOLTAGE_SEARCH_MIN_DOMINANT_STEPS
            && forward_sweep.net_angle.abs() >= ALIGN_VOLTAGE_SEARCH_MIN_NET_ANGLE_RAD
            && backward_sweep.net_angle.abs() >= ALIGN_VOLTAGE_SEARCH_MIN_NET_ANGLE_RAD
            && forward_sweep.net_angle * backward_sweep.net_angle < 0.0
            && forward_dominant_positive != backward_dominant_positive
        {
            return Ok(candidate_voltage);
        }

        candidate_voltage += ALIGN_VOLTAGE_SEARCH_STEP;
    }

    Err("Failed to find align voltage with directional response")
}

async fn calibrate_encoder_tables(
    foc: &FocControllerType,
    driver: &mut PwmDriver<'_, peripherals::TIM1>,
    align_voltage: f32,
) -> f32 {
    let mut secondary_zero_offset = 0.0;

    match calibration::calibrate_encoder_correction(driver, foc, align_voltage).await {
        Ok(correction) => {
            set_runtime_correction(correction);
            info!("Encoder LUT calibration complete");
        }
        Err(e) => warn!("{}", e),
    }

    match calibration::calibrate_secondary_zero_offset(driver, foc, align_voltage).await {
        Ok(calibrated_secondary_zero_offset) => {
            if apply_secondary_zero_offset(calibrated_secondary_zero_offset).await {
                secondary_zero_offset = calibrated_secondary_zero_offset;
                info!("Secondary encoder zero offset = {}", secondary_zero_offset);
            } else {
                warn!("Failed to apply calibrated secondary zero offset");
            }
        }
        Err(e) => warn!("{}", e),
    }

    secondary_zero_offset
}

async fn tune_runtime_gains(
    foc: &mut FocControllerType,
    p_adc: &mut adc::DummyAdc<peripherals::ADC1>,
    driver: &mut PwmDriver<'_, peripherals::TIM1>,
    csa_gain: drv8316::CsaGain,
) {
    if let Some(velocity_gain) = velocity_tuning::tune_velocity_pi(
        foc,
        driver,
        p_adc,
        csa_gain,
        read_sensor,
        VELOCITY_PI_FREQUENCY,
    )
    .await
    {
        foc.set_velocity_kp(velocity_gain.p);
        foc.set_velocity_ki(velocity_gain.i);
        info!("velocity kp={}, ki={}", velocity_gain.p, velocity_gain.i);
    } else {
        warn!("Velocity PI tuning failed, using defaults");
    }

    let kv = motor_tuning::find_kv_rating(foc, driver, p_adc, csa_gain, read_sensor).await;
    foc.motor.kv_rating = kv;
    info!("Motor kv rating = {}", kv / core::f32::consts::TAU * 60.0);
}

fn save_calibration(
    foc: &FocControllerType,
    p_flash: &mut Flash<'static, embassy_stm32::flash::Blocking>,
    can_id: u8,
    secondary_zero_offset: f32,
) {
    let mut config = flash_config::from_foc(foc, can_id);
    config.output_zero_offset = 0.0;
    config.secondary_zero_offset = secondary_zero_offset;
    config.set_encoder_correction(runtime_snapshot());
    if let Err(e) = flash_config::write_config(p_flash, &mut config) {
        warn!("Failed to save calibration to flash: {:?}", e);
    } else {
        info!("Calibration saved to flash");
    }
}

#[allow(clippy::too_many_arguments)]
async fn run_full_calibration(
    foc: &mut FocControllerType,
    p_adc: &mut adc::DummyAdc<peripherals::ADC1>,
    driver: &mut PwmDriver<'_, peripherals::TIM1>,
    gate_driver: &mut drv8316t::Drv8316T<'static>,
    p_flash: &mut Flash<'static, embassy_stm32::flash::Blocking>,
    can_id: u8,
    csa_gain: drv8316::CsaGain,
) -> Result<(), &'static str> {
    let align_voltage = run_motor_calibration(foc, p_adc, driver, gate_driver, csa_gain).await?;

    let secondary_zero_offset = calibrate_encoder_tables(foc, driver, align_voltage).await;

    foc.enable();
    tune_runtime_gains(foc, p_adc, driver, csa_gain).await;
    save_calibration(foc, p_flash, can_id, secondary_zero_offset);
    foc_isr::set_output_zero_offset(0.0);

    Ok(())
}

#[inline]
pub(crate) async fn init_foc(
    drvoff_pin: gpio::Output<'static>,
    timer: MotorTimer,
    mut p_adc: adc::DummyAdc<peripherals::ADC1>,
    p_flash: &mut Flash<'static, embassy_stm32::flash::Blocking>,
    stored_config: Option<flash_config::ConfigData>,
) -> Option<u8> {
    let csa_gain = CURRENT_SENSE_GAIN;

    let can_id = stored_config
        .as_ref()
        .map(|c| c.can_id)
        .unwrap_or(DEFAULT_MOTOR_CAN_ID);

    info!("Running motor calibration");

    let mut driver = PwmDriver::new(timer.timer);
    let mut gate_driver = drv8316t::Drv8316T::new(drvoff_pin);
    Timer::after_millis(1).await;

    let mut foc = create_foc_controller();

    gate_driver.turn_on();
    foc.set_psu_voltage(measure_vm_sense());

    if let Err(e) = run_full_calibration(
        &mut foc,
        &mut p_adc,
        &mut driver,
        &mut gate_driver,
        p_flash,
        can_id,
        csa_gain,
    )
    .await
    {
        error!("{}", e);
        gate_driver.turn_off();
        return None;
    }

    driver.run(DutyCycle3Phase::zero());
    gate_driver.turn_off();
    info!("Motor calibration complete; flash main firmware next");
    None
}
