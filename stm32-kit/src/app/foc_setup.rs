use embassy_stm32::peripherals;
use foc::controller::{FocController, OutputLimit};

use crate::{cordic::sincos, drv8316, gm3506 as motor, pwm};

use super::{CURRENT_SAFETY_MARGIN, FocControllerType, PSU_VOLTAGE, VOLTAGE_RAMP_RATE};

pub(crate) type MotorTimer = pwm::Pwm6Timer<
    'static,
    peripherals::TIM1,
    peripherals::PA8,
    peripherals::PA9,
    peripherals::PA10,
    peripherals::PB13,
    peripherals::PB14,
    peripherals::PB15,
>;

pub(crate) fn create_foc_controller() -> FocControllerType {
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
    foc
}
