use defmt::{error, info};
use embassy_stm32::{flash::Flash, gpio, peripherals};
use embassy_time::Timer;
use foc::controller::RunMode;

use crate::{
    adc::{self, measure_vm_sense},
    bldc_driver::PwmDriver,
    drv8316, drv8316t,
    encoder_correction::set_runtime_correction,
    flash_config, foc_isr,
};

use super::{
    CURRENT_SENSE_GAIN, FocControllerType,
    foc_setup::{MotorTimer, create_foc_controller},
    persistence::GATE_DRIVER,
};

fn apply_stored_calibration(foc: &mut FocControllerType, config: &flash_config::ConfigData) {
    flash_config::apply_to_foc(foc, config);
    set_runtime_correction(config.get_encoder_correction());
    foc_isr::set_output_zero_offset(config.output_zero_offset);
    info!(
        "Loaded calibration: mapping=({},{},{}), bias_angle={}",
        config.current_mapping.0,
        config.current_mapping.1,
        config.current_mapping.2,
        config.bias_angle
    );
}

fn start_foc(foc: FocControllerType, csa_gain: drv8316::CsaGain) {
    foc_isr::initialize_foc_context(foc, csa_gain);
    info!("FOC interrupt-driven control started");
}

async fn start_configured_foc(
    mut foc: FocControllerType,
    mut gate_driver: drv8316t::Drv8316T<'static>,
    driver: PwmDriver<'static, peripherals::TIM1>,
    csa_gain: drv8316::CsaGain,
) {
    gate_driver.turn_on();

    foc.set_run_mode(RunMode::Torque);
    foc.set_target_torque(0.0);
    foc.enable();

    start_foc(foc, csa_gain);

    *(GATE_DRIVER.lock().await) = Some(gate_driver);

    core::mem::forget(driver);
}

#[inline]
pub(crate) async fn init_foc(
    drvoff_pin: gpio::Output<'static>,
    timer: MotorTimer,
    _p_adc: adc::DummyAdc<peripherals::ADC1>,
    _p_flash: &mut Flash<'static, embassy_stm32::flash::Blocking>,
    stored_config: Option<flash_config::ConfigData>,
) -> Option<u8> {
    let csa_gain = CURRENT_SENSE_GAIN;
    let driver = PwmDriver::new(timer.timer);
    let mut gate_driver = drv8316t::Drv8316T::new(drvoff_pin);
    Timer::after_millis(1).await;

    let Some(config) = stored_config.as_ref() else {
        error!("No stored calibration found; flash tuner firmware first");
        gate_driver.turn_off();
        return None;
    };

    info!("Found stored calibration, skipping motor calibration");

    let can_id = config.can_id;
    let mut foc = create_foc_controller();

    foc.set_psu_voltage(measure_vm_sense());
    apply_stored_calibration(&mut foc, config);
    start_configured_foc(foc, gate_driver, driver, csa_gain).await;

    Some(can_id)
}
