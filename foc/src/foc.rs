use libm::cosf;
use libm::floorf;
use libm::sinf;
use libm::sqrtf;

use core::fmt::Debug;
use core::prelude::rust_2024::derive;
use core::result::Result;
use core::result::Result::Err;
use core::result::Result::Ok;

pub struct Radian(pub f32);
pub struct RadianPerSecond(pub f32);

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
    pub angle: Radian,                    // Radians
    pub target_angle: Radian,             // Radians
    pub sensor_angle: Radian,             // Radians
    pub velocity: RadianPerSecond,        // Radians per second
    pub target_velocity: RadianPerSecond, // Radians per second
    pub sensor_velocity: RadianPerSecond, // Radians per second
    pub current: f32,                     // Amperes
}

pub fn svpwm(
    v_ref: f32,
    electrical_angle: Radian,
    v_max: f32,
) -> Result<(f32, f32, f32), FocError> {
    // Calculate the duty cycles for the three phases based on the voltage and angle

    // Check input parameters and theoretical SVPWM maximum (sqrt(3)/2 * v_max)
    let theoretical_max = sqrtf(3.0) / 2.0 * v_max;
    if v_ref < 0.0 || v_max <= 0.0 || v_ref > theoretical_max {
        return Err(FocError::InvalidParameters);
    }
    let pi_third = core::f32::consts::PI / 3.0;
    let inv_sqrt3 = 1.0 / sqrtf(3.0);

    let v_normalized = v_ref / v_max;

    let target_angle = electrical_angle.0 + core::f32::consts::PI / 2.0;

    // use electrical angle + pi/2 for maximum torque
    let v_x = v_normalized * cosf(target_angle);
    let v_y = v_normalized * sinf(target_angle);

    // get the sector based on angle
    let sector = floorf(target_angle / pi_third) as u8 % 6;
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
            let t_111 = (1.0 - t_110 - t_010) / 2.0;
            Ok((t_110 + t_111, t_010 + t_110 + t_111, t_111))
        }
        2 => {
            let t_010 = v_y * 2.0 * inv_sqrt3;
            let t_011 = -v_x - v_y * inv_sqrt3;
            let t_111 = (1.0 - t_010 - t_011) / 2.0;
            Ok((t_111, t_010 + t_011 + t_111, t_011 + t_111))
        }
        3 => {
            let t_011 = -v_x + v_y * inv_sqrt3;
            let t_001 = -v_y * 2.0 * inv_sqrt3;
            let t_111 = (1.0 - t_011 - t_001) / 2.0;
            Ok((t_111, t_011 + t_111, t_011 + t_001 + t_111))
        }
        4 => {
            let t_001 = -v_x - v_y * inv_sqrt3;
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

pub fn set_angle(_angle: f32) {
    // set target velocity from angle difference
    // set target current from velocity difference
    // calculate FOC control signals
}
pub fn set_velocity(_rpm: u16) {
    // set target current from velocity difference
    // calculate FOC control signals
}
pub fn set_torque(_current: f32) {
    // calculate FOC control signals
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_svpwm_valid_parameters() {
        let result = svpwm(1.0, Radian(0.0), 2.0);
        assert!(result.is_ok());

        let (a, b, c) = result.unwrap();
        assert!(a >= 0.0 && a <= 1.0);
        assert!(b >= 0.0 && b <= 1.0);
        assert!(c >= 0.0 && c <= 1.0);
    }

    #[test]
    fn test_svpwm_invalid_negative_vref() {
        let result = svpwm(-1.0, Radian(0.0), 2.0);
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), FocError::InvalidParameters));
    }

    #[test]
    fn test_svpwm_invalid_zero_vmax() {
        let result = svpwm(1.0, Radian(0.0), 0.0);
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), FocError::InvalidParameters));
    }

    #[test]
    fn test_svpwm_invalid_vref_greater_than_vmax() {
        let result = svpwm(3.0, Radian(0.0), 2.0);
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), FocError::InvalidParameters));
    }

    #[test]
    fn test_svpwm_all_sectors() {
        let pi_third = core::f32::consts::PI / 3.0;

        for sector in 0..6 {
            let angle = sector as f32 * pi_third + 0.1;
            let result = svpwm(0.5, Radian(angle), 1.0);
            assert!(result.is_ok(), "Failed for sector {}", sector);

            let (a, b, c) = result.unwrap();
            println!(
                "Sector {}: angle={:.3}, a={:.6}, b={:.6}, c={:.6}",
                sector, angle, a, b, c
            );

            assert!(
                a >= 0.0 && a <= 1.0,
                "Phase A out of range for sector {}: {}",
                sector,
                a
            );
            assert!(
                b >= 0.0 && b <= 1.0,
                "Phase B out of range for sector {}: {}",
                sector,
                b
            );
            assert!(
                c >= 0.0 && c <= 1.0,
                "Phase C out of range for sector {}: {}",
                sector,
                c
            );
        }
    }

    #[test]
    fn test_svpwm_maximum_voltage() {
        // Test at theoretical maximum (sqrt(3)/2 ≈ 0.866)
        let pi_third = core::f32::consts::PI / 3.0;
        let sqrt3_over_2 = sqrtf(3.0) / 2.0;

        // Test at theoretical maximum
        let v_ref = sqrt3_over_2;
        for sector in 0..6 {
            let angle = sector as f32 * pi_third + 0.1;
            let result = svpwm(v_ref, Radian(angle), 1.0);
            assert!(
                result.is_ok(),
                "Failed for sector {} at theoretical max",
                sector
            );

            let (a, b, c) = result.unwrap();
            println!(
                "Max V Sector {}: angle={:.3}, v_ref={:.6}, a={:.6}, b={:.6}, c={:.6}",
                sector, angle, v_ref, a, b, c
            );

            // SVPWM outputs must be strictly within [0.0, 1.0] range
            assert!(
                a >= 0.0 && a <= 1.0,
                "Phase A out of range for sector {}: {}",
                sector,
                a
            );
            assert!(
                b >= 0.0 && b <= 1.0,
                "Phase B out of range for sector {}: {}",
                sector,
                b
            );
            assert!(
                c >= 0.0 && c <= 1.0,
                "Phase C out of range for sector {}: {}",
                sector,
                c
            );
        }
    }

    #[test]
    fn test_svpwm_duty_cycle() {
        // Test that the sum of duty cycles is always 1.0
        let v_ref = 0.5;
        for sector in 0..6 {
            let angle = sector as f32 * core::f32::consts::PI / 3.0 + 0.1;
            let result = svpwm(v_ref, Radian(angle), 1.0);
            assert!(result.is_ok(), "Failed for sector {}", sector);

            let (a, b, c) = result.unwrap();
            let max_cycle = a.max(b).max(c);
            let min_cycle = a.min(b).min(c);
            println!(
                "Sector {}: angle={}, max_cycle={}, min_cycle={}",
                sector, angle, max_cycle, min_cycle
            );

            assert!(
                (max_cycle + min_cycle - 1.0).abs() < 1e-6,
                "Duty cycle max + min not equal to 1 for sector {}, max: {}, min: {}",
                sector,
                max_cycle,
                min_cycle
            );
        }
    }
}
