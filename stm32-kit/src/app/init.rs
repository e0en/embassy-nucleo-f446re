use defmt::{error, info, warn};
use embassy_stm32::{flash::Flash, gpio, peripherals};
use embassy_time::{Duration, Timer};
use foc::{
    controller::{FocController, OutputLimit, RunMode},
    pwm_output::{DutyCycle3Phase, PwmOutput},
    svpwm,
};

use crate::{
    adc::{self, measure_vm_sense},
    bldc_driver::PwmDriver,
    calibration,
    cordic::sincos,
    current_tuning, drv8316, drv8316t,
    encoder_correction::{runtime_snapshot, set_runtime_correction},
    flash_config, foc_isr, gm3506 as motor, motor_tuning, pwm, velocity_tuning,
};

use super::{
    ALIGN_VOLTAGE, ALIGN_VOLTAGE_SEARCH_DELTA_EPSILON_RAD, ALIGN_VOLTAGE_SEARCH_MIN_DOMINANT_STEPS,
    ALIGN_VOLTAGE_SEARCH_MIN_NET_ANGLE_RAD, ALIGN_VOLTAGE_SEARCH_SETTLE_MS,
    ALIGN_VOLTAGE_SEARCH_START, ALIGN_VOLTAGE_SEARCH_STEP, ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_MS,
    ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_RAD, ALIGN_VOLTAGE_SEARCH_SWEEP_STEPS, CURRENT_PI_FREQUENCY,
    CURRENT_SAFETY_MARGIN, FocControllerType, IMPEDANCE_TUNING_MAX_CURRENT,
    PHASE_MAPPING_TOLERANCE, PSU_VOLTAGE, VELOCITY_PI_FREQUENCY, VOLTAGE_RAMP_RATE,
    encoder::read_sensor, persistence::GATE_DRIVER,
};

fn create_foc_controller(use_current_sensing: bool) -> FocControllerType {
    let mut foc = FocController::new(
        motor::SETUP,
        motor::CURRENT_PID,
        motor::ANGLE_PID,
        motor::VELOCITY_PID,
        OutputLimit {
            max_value: PSU_VOLTAGE,
            ramp: VOLTAGE_RAMP_RATE,
        },
        sincos as fn(f32) -> (f32, f32),
    );
    foc.set_driver_current_limit(drv8316::MAX_CURRENT * CURRENT_SAFETY_MARGIN);

    if use_current_sensing {
        foc.enable_current_sensing();
    } else {
        foc.disable_current_sensing();
    }

    foc
}

async fn run_motor_calibration(
    foc: &mut FocControllerType,
    p_adc: &mut adc::DummyAdc<peripherals::ADC1>,
    driver: &mut PwmDriver<'_, peripherals::TIM1>,
    gate_driver: &mut drv8316t::Drv8316T<'static>,
    csa_gain: drv8316::CsaGain,
    use_current_sensing: bool,
) -> Result<f32, &'static str> {
    if use_current_sensing {
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
    }

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

    if use_current_sensing
        && let Some(offset) =
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
        let mut previous_angle = read_sensor().cumulative_angle;
        let mut forward_positive_steps = 0;
        let mut forward_negative_steps = 0;
        let mut forward_net_angle = 0.0;

        for _ in 0..ALIGN_VOLTAGE_SEARCH_SWEEP_STEPS {
            electrical_angle += ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_RAD;
            let signal = svpwm(
                candidate_voltage,
                0.0,
                electrical_angle,
                psu_voltage,
                sincos,
            )
            .map_err(|_| "Failed to generate align search sweep signal")?;
            driver.run(signal);
            Timer::after_millis(ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_MS).await;
            let angle = read_sensor().cumulative_angle;
            let delta = angle - previous_angle;
            if delta >= ALIGN_VOLTAGE_SEARCH_DELTA_EPSILON_RAD {
                forward_positive_steps += 1;
            } else if delta <= -ALIGN_VOLTAGE_SEARCH_DELTA_EPSILON_RAD {
                forward_negative_steps += 1;
            }
            forward_net_angle += delta;
            previous_angle = angle;
        }

        let mut backward_positive_steps = 0;
        let mut backward_negative_steps = 0;
        let mut backward_net_angle = 0.0;
        for _ in 0..ALIGN_VOLTAGE_SEARCH_SWEEP_STEPS {
            electrical_angle -= ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_RAD;
            let signal = svpwm(
                candidate_voltage,
                0.0,
                electrical_angle,
                psu_voltage,
                sincos,
            )
            .map_err(|_| "Failed to generate align search sweep signal")?;
            driver.run(signal);
            Timer::after_millis(ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_MS).await;
            let angle = read_sensor().cumulative_angle;
            let delta = angle - previous_angle;
            if delta <= -ALIGN_VOLTAGE_SEARCH_DELTA_EPSILON_RAD {
                backward_negative_steps += 1;
            } else if delta >= ALIGN_VOLTAGE_SEARCH_DELTA_EPSILON_RAD {
                backward_positive_steps += 1;
            }
            backward_net_angle += delta;
            previous_angle = angle;
        }

        driver.run(DutyCycle3Phase::zero());
        let forward_dominant_positive = forward_positive_steps > forward_negative_steps;
        let backward_dominant_positive = backward_positive_steps > backward_negative_steps;
        let forward_dominant_steps = if forward_dominant_positive {
            forward_positive_steps
        } else {
            forward_negative_steps
        };
        let backward_dominant_steps = if backward_dominant_positive {
            backward_positive_steps
        } else {
            backward_negative_steps
        };
        info!(
            "Align search {}V: fwd(+/-)={}/{} fwd_net={} bwd(+/-)={}/{} bwd_net={}",
            candidate_voltage,
            forward_positive_steps,
            forward_negative_steps,
            forward_net_angle,
            backward_positive_steps,
            backward_negative_steps,
            backward_net_angle
        );

        if forward_dominant_steps >= ALIGN_VOLTAGE_SEARCH_MIN_DOMINANT_STEPS
            && backward_dominant_steps >= ALIGN_VOLTAGE_SEARCH_MIN_DOMINANT_STEPS
            && forward_net_angle.abs() >= ALIGN_VOLTAGE_SEARCH_MIN_NET_ANGLE_RAD
            && backward_net_angle.abs() >= ALIGN_VOLTAGE_SEARCH_MIN_NET_ANGLE_RAD
            && forward_net_angle * backward_net_angle < 0.0
            && forward_dominant_positive != backward_dominant_positive
        {
            return Ok(candidate_voltage);
        }

        candidate_voltage += ALIGN_VOLTAGE_SEARCH_STEP;
    }

    Err("Failed to find align voltage with directional response")
}

#[inline]
pub(crate) async fn init_foc(
    drvoff_pin: gpio::Output<'static>,
    timer: pwm::Pwm6Timer<
        'static,
        peripherals::TIM1,
        peripherals::PA8,
        peripherals::PA9,
        peripherals::PA10,
        peripherals::PB13,
        peripherals::PB14,
        peripherals::PB15,
    >,
    mut p_adc: adc::DummyAdc<peripherals::ADC1>,
    p_flash: &mut Flash<'static, embassy_stm32::flash::Blocking>,
    stored_config: Option<flash_config::ConfigData>,
) -> Option<u8> {
    let use_current_sensing = true;
    let csa_gain = drv8316::CsaGain::Gain0_3V;

    let has_stored_config = stored_config.is_some();
    let can_id = stored_config.as_ref().map(|c| c.can_id).unwrap_or(0x0F);

    if has_stored_config {
        info!("Found stored calibration, skipping motor calibration");
    } else {
        info!("No stored calibration found, running motor calibration");
    }

    let mut driver = PwmDriver::new(timer.timer);
    let mut gate_driver = drv8316t::Drv8316T::new(drvoff_pin);
    Timer::after_millis(1).await;

    let mut foc = create_foc_controller(use_current_sensing);
    gate_driver.turn_on();

    foc.set_psu_voltage(measure_vm_sense());

    if let Some(config) = &stored_config {
        flash_config::apply_to_foc(&mut foc, config);
        set_runtime_correction(config.get_encoder_correction());
        info!(
            "Loaded calibration: mapping=({},{},{}), bias_angle={}",
            config.current_mapping.0,
            config.current_mapping.1,
            config.current_mapping.2,
            config.bias_angle
        );
    } else {
        let align_voltage = match run_motor_calibration(
            &mut foc,
            &mut p_adc,
            &mut driver,
            &mut gate_driver,
            csa_gain,
            use_current_sensing,
        )
        .await
        {
            Ok(v) => v,
            Err(e) => {
                error!("{}", e);
                gate_driver.turn_off();
                return None;
            }
        };

        match calibration::calibrate_encoder_correction(&mut driver, &foc, align_voltage).await {
            Ok(correction) => {
                set_runtime_correction(correction);
                info!("Encoder LUT calibration complete");
            }
            Err(e) => warn!("{}", e),
        }

        foc.enable();

        if let Some(velocity_gain) = velocity_tuning::tune_velocity_pi(
            &mut foc,
            &mut driver,
            &mut p_adc,
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

        let kv =
            motor_tuning::find_kv_rating(&mut foc, &mut driver, &mut p_adc, csa_gain, read_sensor)
                .await;
        foc.motor.kv_rating = kv;
        info!("Motor kv rating = {}", kv / core::f32::consts::TAU * 60.0);

        let mut config = flash_config::from_foc(&foc, can_id);
        config.primary_zero_offset = 0.0;
        config.secondary_zero_offset = 0.0;
        config.set_encoder_correction(runtime_snapshot());
        if let Err(e) = flash_config::write_config(p_flash, &mut config) {
            warn!("Failed to save calibration to flash: {:?}", e);
        } else {
            info!("Calibration saved to flash");
        }
    }

    foc.set_run_mode(RunMode::Torque);
    foc.set_target_torque(0.0);
    foc.enable();

    foc_isr::initialize_foc_context(foc, csa_gain, use_current_sensing);
    info!("FOC interrupt-driven control started");

    *(GATE_DRIVER.lock().await) = Some(gate_driver);

    core::mem::forget(driver);

    Some(can_id)
}
