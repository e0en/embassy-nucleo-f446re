use libm::cosf;
use libm::floorf;
use libm::sinf;
use libm::sqrtf;

use core::assert;
use core::fmt::Debug;
use core::prelude::rust_2024::derive;
use core::result::Result;
use core::result::Result::Err;
use core::result::Result::Ok;

pub struct Radian(pub f32);

#[derive(Debug)]
pub enum FocError {
    InvalidParameters,
    CalculationError,
}

pub struct Motor {
    pub pole_pairs: u8,
    pub max_current: f32, // Amperes
    pub max_voltage: f32, // Volts
    pub max_power: f32,   // Watts
    pub max_rpm: u16,
    // TODO: add more parameters as needed
    // pub resistance: f32, // Ohms
}

pub struct PID {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

pub struct FOCParameters {
    pub align_voltage: f32, // Volts
    pub angle_pid: PID,
    pub velocity_pid: PID,
    pub velocity_output_limit: f32, // Volts per second
    pub velocity_time_filter: f32,  // Seconds,
    pub motor: Motor,
}

pub struct FOC {
    pub parameters: FOCParameters,
    pub angle: f32,           // Radians
    pub target_angle: f32,    // Radians
    pub sensor_angle: f32,    // Radians
    pub velocity: f32,        // Radians per second
    pub target_velocity: f32, // Radians per second
    pub sensor_velocity: f32, // Radians per second
    pub current: f32,         // Amperes
}

pub fn align_sensor() {}

fn svpwm(v_ref: f32, angle: Radian, v_max: f32) -> Result<(f32, f32, f32), FocError> {
    // Calculate the duty cycles for the three phases based on the voltage and angle

    if v_ref < 0.0 || v_max <= 0.0 || v_ref > v_max {
        return Err(FocError::InvalidParameters);
    }
    let pi_third = core::f32::consts::PI / 3.0;
    let inv_sqrt3 = 1.0 / sqrtf(3.0);

    let v_x = v_ref * cosf(angle.0);
    let v_y = v_ref * sinf(angle.0);

    // get the sector based on angle
    let sector = floorf(angle.0 / pi_third) as u8 % 6;
    match sector {
        0 => {
            let t_100 = v_x - v_y * inv_sqrt3;
            let t_110 = v_y * 2.0 * inv_sqrt3;
            let t_111 = (1.0 - t_100 - t_110) / 2.0;
            Ok((t_100 + t_110 + t_111, t_110 + t_111, t_111))
        }
        1 => {
            let t_110 = v_x + v_y * inv_sqrt3;
            let t_010 = -v_x + v_y * inv_sqrt3;
            let t_111 = (1.0 - t_110 - t_110) / 2.0;
            Ok((t_110 + t_111, t_010 + t_110 + t_111, t_111))
        }
        2 => {
            let t_010 = v_y * 2.0 * inv_sqrt3;
            let t_011 = -v_x - v_y * inv_sqrt3;
            let t_111 = (1.0 - t_010 - t_011) / 2.0;
            Ok((t_111, t_010 + t_011 + t_111, t_011 + t_111))
        }
        3 => {
            let t_011 = -v_x - v_y * inv_sqrt3;
            let t_001 = -v_y * 2.0 * inv_sqrt3;
            let t_111 = (1.0 - t_011 - t_001) / 2.0;
            Ok((t_111, t_011 + t_111, t_011 + t_001 + t_111))
        }
        4 => {
            let t_001 = -v_x + v_y * inv_sqrt3;
            let t_101 = v_x - v_y * inv_sqrt3;
            let t_111 = (1.0 - t_001 - t_101) / 2.0;
            Ok((t_101 + t_111, t_111, t_001 + t_101 + t_111))
        }
        5 => {
            let t_101 = -v_y * 2.0 * inv_sqrt3;
            let t_100 = v_x + v_y * inv_sqrt3;
            let t_111 = (1.0 - t_101 - t_100) / 2.0;
            Ok((t_101 + t_100 + t_111, t_111, t_101 + t_111))
        }
        _ => Err(FocError::CalculationError),
    }
}

pub fn set_angle(angle: f32) {
    // set target velocity from angle difference
    // set target current from velocity difference
    // calculate FOC control signals
}
pub fn set_velocity(rpm: u16) {
    // set target current from velocity difference
    // calculate FOC control signals
}
pub fn set_torque(current: f32) {
    // calculate FOC control signals
}
