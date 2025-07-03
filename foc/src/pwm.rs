use libm::cosf;
use libm::floorf;
use libm::sinf;
use libm::sqrtf;

use crate::pwm_output::DutyCycle3Phase;
use crate::units::Radian;

use core::fmt::Debug;
use core::prelude::rust_2024::derive;
use core::result::Result;
use core::result::Result::Err;
use core::result::Result::Ok;

#[derive(Debug)]
pub enum FocError {
    InvalidParameters,
    CalculationError,
    AlignError,
}

pub fn svpwm(
    v_ref: f32,
    electrical_angle: Radian,
    v_max: f32,
) -> Result<DutyCycle3Phase, FocError> {
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
            Ok(DutyCycle3Phase {
                t1: t_100 + t_110 + t_111,
                t2: t_110 + t_111,
                t3: t_111,
            })
        }
        1 => {
            let t_110 = v_x + v_y * inv_sqrt3;
            let t_010 = -v_x + v_y * inv_sqrt3;
            let t_111 = (1.0 - t_110 - t_010) / 2.0;
            Ok(DutyCycle3Phase {
                t1: t_110 + t_111,
                t2: t_010 + t_110 + t_111,
                t3: t_111,
            })
        }
        2 => {
            let t_010 = v_y * 2.0 * inv_sqrt3;
            let t_011 = -v_x - v_y * inv_sqrt3;
            let t_111 = (1.0 - t_010 - t_011) / 2.0;
            Ok(DutyCycle3Phase {
                t1: t_111,
                t2: t_010 + t_011 + t_111,
                t3: t_011 + t_111,
            })
        }
        3 => {
            let t_011 = -v_x + v_y * inv_sqrt3;
            let t_001 = -v_y * 2.0 * inv_sqrt3;
            let t_111 = (1.0 - t_011 - t_001) / 2.0;
            Ok(DutyCycle3Phase {
                t1: t_111,
                t2: t_011 + t_111,
                t3: t_011 + t_001 + t_111,
            })
        }
        4 => {
            let t_001 = -v_x - v_y * inv_sqrt3;
            let t_101 = v_x - v_y * inv_sqrt3;
            let t_111 = (1.0 - t_001 - t_101) / 2.0;
            Ok(DutyCycle3Phase {
                t1: t_101 + t_111,
                t2: t_111,
                t3: t_001 + t_101 + t_111,
            })
        }
        5 => {
            let t_101 = -v_y * 2.0 * inv_sqrt3;
            let t_100 = v_x + v_y * inv_sqrt3;
            let t_111 = (1.0 - t_101 - t_100) / 2.0;
            Ok(DutyCycle3Phase {
                t1: t_101 + t_100 + t_111,
                t2: t_111,
                t3: t_101 + t_111,
            })
        }
        _ => Err(FocError::CalculationError),
    }
}

pub fn svpwm_simplified(
    v_ref: f32,
    electrical_angle: Radian,
    v_max: f32,
) -> Result<DutyCycle3Phase, FocError> {
    // Calculate the duty cycles for the three phases based on the voltage and angle

    // Check input parameters and theoretical SVPWM maximum (sqrt(3)/2 * v_max)
    let theoretical_max = sqrtf(3.0) / 2.0 * v_max;
    if v_ref < 0.0 || v_max <= 0.0 || v_ref > theoretical_max {
        return Err(FocError::InvalidParameters);
    }

    let v_normalized = v_ref / v_max / 1.5;

    let v_x = -v_normalized * sinf(electrical_angle.0);
    let v_y = v_normalized * cosf(electrical_angle.0);

    // use SimpleFOC's implementation
    // https://github.com/simplefoc/Arduino-FOC/blob/05672d28187320c10f74f8fa382e92b09cfadce0/src/BLDCMotor.cpp#L568-L602
    let half_sqrt3 = sqrtf(3.0) / 2.0;
    let u_a = v_x;
    let u_b = -v_x / 2.0 + half_sqrt3 * v_y;
    let u_c = -v_x / 2.0 - half_sqrt3 * v_y;

    let u_max = u_a.max(u_b.max(u_c));
    let u_min = u_a.min(u_b.min(u_c));

    let center = 0.5 - (u_max + u_min) / 2.0;

    Ok(DutyCycle3Phase {
        t1: u_a + center,
        t2: u_b + center,
        t3: u_c + center,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_svpwm_valid_parameters() {
        let result = svpwm(1.0, Radian(0.0), 2.0);
        assert!(result.is_ok());

        let duty_cycle = result.unwrap();
        assert!(duty_cycle.t1 >= 0.0 && duty_cycle.t1 <= 1.0);
        assert!(duty_cycle.t2 >= 0.0 && duty_cycle.t2 <= 1.0);
        assert!(duty_cycle.t3 >= 0.0 && duty_cycle.t3 <= 1.0);
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
            assert!(result.is_ok(), "Failed for sector {sector}");

            let duty_cycle = result.unwrap();
            println!(
                "Sector {sector}: angle={:.3}, t1={:.6}, t2={:.6}, t3={:.6}",
                angle, duty_cycle.t1, duty_cycle.t2, duty_cycle.t3
            );

            assert!(
                duty_cycle.t1 >= 0.0 && duty_cycle.t1 <= 1.0,
                "Phase t1 out of range for sector {sector}: {}",
                duty_cycle.t1
            );
            assert!(
                duty_cycle.t2 >= 0.0 && duty_cycle.t2 <= 1.0,
                "Phase t2 out of range for sector {sector}: {}",
                duty_cycle.t2
            );
            assert!(
                duty_cycle.t3 >= 0.0 && duty_cycle.t3 <= 1.0,
                "Phase t3 out of range for sector {sector}: {}",
                duty_cycle.t3
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
                "Failed for sector {sector} at theoretical max",
            );

            let duty_cycle = result.unwrap();
            println!(
                "Max V Sector {sector}: angle={:.3}, v_ref={:.6}, t1={:.6}, t2={:.6}, t3={:.6}",
                angle, v_ref, duty_cycle.t1, duty_cycle.t2, duty_cycle.t3
            );

            // SVPWM outputs must be strictly within [0.0, 1.0] range
            assert!(
                duty_cycle.t1 >= 0.0 && duty_cycle.t1 <= 1.0,
                "Phase t1 out of range for sector {sector}: {}",
                duty_cycle.t1
            );
            assert!(
                duty_cycle.t2 >= 0.0 && duty_cycle.t2 <= 1.0,
                "Phase t2 out of range for sector {sector}: {}",
                duty_cycle.t2
            );
            assert!(
                duty_cycle.t3 >= 0.0 && duty_cycle.t3 <= 1.0,
                "Phase t3 out of range for sector {sector}: {}",
                duty_cycle.t3
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
            assert!(result.is_ok(), "Failed for sector {sector}");

            let duty_cycle = result.unwrap();
            let max_cycle = duty_cycle.t1.max(duty_cycle.t2).max(duty_cycle.t3);
            let min_cycle = duty_cycle.t1.min(duty_cycle.t2).min(duty_cycle.t3);
            println!(
                "Sector {sector}: angle={angle}, max_cycle={max_cycle}, min_cycle={min_cycle}"
            );

            assert!(
                (max_cycle + min_cycle - 1.0).abs() < 1e-6,
                "Duty cycle max + min not equal to 1 for sector {sector}, max: {max_cycle}, min: {min_cycle}"
            );
        }
    }

    #[test]
    fn test_svpwm_equivalency() {
        // Test that two implementations of svpwm give identical result
        let v_max = 1.0;
        for v_ref in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6] {
            for angle_div_pi in [
                0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5,
                1.6, 1.7, 1.8, 1.9,
            ] {
                let angle = Radian(angle_div_pi * core::f32::consts::PI);
                let duty_reference = svpwm(v_ref, angle, v_max).unwrap();
                let duty_simple = svpwm_simplified(v_ref, angle, v_max).unwrap();

                assert!(
                    (duty_reference.t1 - duty_simple.t1).abs() < 1e-6,
                    "t1: {:?} != {:?} at v_ref={}, angle={}",
                    duty_reference,
                    duty_simple,
                    v_ref,
                    angle.0
                );

                assert!(
                    (duty_reference.t2 - duty_simple.t2).abs() < 1e-6,
                    "t2: {:?} != {:?} at v_ref={}, angle={}",
                    duty_reference,
                    duty_simple,
                    v_ref,
                    angle.0
                );

                assert!(
                    (duty_reference.t3 - duty_simple.t3).abs() < 1e-6,
                    "t3: {:?} != {:?} at v_ref={}, angle={}",
                    duty_reference,
                    duty_simple,
                    v_ref,
                    angle.0
                );
            }
        }
    }
}
