use crate::pwm_output::DutyCycle3Phase;
use crate::units::Radian;

use core::result::Result;
use core::result::Result::Err;
use core::result::Result::Ok;

#[derive(Debug)]
pub enum FocError {
    InvalidParameters,
    CalculationError,
    AlignError,
}

const HALF_SQRT3: f32 = 0.866_025_4;

pub fn svpwm(
    v_q: f32,
    v_d: f32,
    mut electrical_angle: Radian,
    v_max: f32,
) -> Result<DutyCycle3Phase, FocError> {
    // Calculate the duty cycles for the three phases based on the voltage and angle

    // Check input parameters and theoretical SVPWM maximum (sqrt(3)/2 * v_max)
    if v_max <= 0.0 {
        return Err(FocError::InvalidParameters);
    }
    let theoretical_max = HALF_SQRT3 * v_max;
    let v_q = v_q.min(theoretical_max).max(-theoretical_max);
    let v_d = v_d.min(theoretical_max).max(-theoretical_max);

    let v_d = v_d / v_max / 1.5;
    let v_q = v_q / v_max / 1.5;

    let (sa, ca) = electrical_angle.get_sin_cos();

    // inverse park transform
    let v_alpha = v_d * ca - v_q * sa;
    let v_beta = v_d * sa + v_q * ca;

    // use SimpleFOC's implementation
    // https://github.com/simplefoc/Arduino-FOC/blob/05672d28187320c10f74f8fa382e92b09cfadce0/src/BLDCMotor.cpp#L568-L602

    // inverse clarke transform
    let u_a = v_alpha;
    let u_b = -v_alpha / 2.0 + HALF_SQRT3 * v_beta;
    let u_c = -v_alpha / 2.0 - HALF_SQRT3 * v_beta;

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
    use libm::{cosf, floorf, sinf};
    const INV_SQRT3: f32 = 0.577_350_26;

    fn svpwm_reference(
        v_ref: f32,
        electrical_angle: Radian,
        v_max: f32,
    ) -> Result<DutyCycle3Phase, FocError> {
        // Calculate the duty cycles for the three phases based on the voltage and angle

        // Clamp input parameters and theoretical SVPWM maximum (sqrt(3)/2 * v_max)
        let theoretical_max = HALF_SQRT3 * v_max;
        let v_ref = v_ref.max(-theoretical_max).min(theoretical_max);

        let v_normalized = v_ref / v_max;

        let target_angle = electrical_angle.angle + core::f32::consts::FRAC_PI_2;

        // use electrical angle + pi/2 for maximum torque
        let v_x = v_normalized * cosf(target_angle);
        let v_y = v_normalized * sinf(target_angle);

        // get the sector based on angle
        let sector = floorf(target_angle / core::f32::consts::FRAC_PI_3) as u8 % 6;
        match sector {
            0 => {
                let t_100 = v_x - v_y * INV_SQRT3;
                let t_110 = v_y * 2.0 * INV_SQRT3;
                let t_111 = (1.0 - t_100 - t_110) / 2.0;
                Ok(DutyCycle3Phase {
                    t1: t_100 + t_110 + t_111,
                    t2: t_110 + t_111,
                    t3: t_111,
                })
            }
            1 => {
                let t_110 = v_x + v_y * INV_SQRT3;
                let t_010 = -v_x + v_y * INV_SQRT3;
                let t_111 = (1.0 - t_110 - t_010) / 2.0;
                Ok(DutyCycle3Phase {
                    t1: t_110 + t_111,
                    t2: t_010 + t_110 + t_111,
                    t3: t_111,
                })
            }
            2 => {
                let t_010 = v_y * 2.0 * INV_SQRT3;
                let t_011 = -v_x - v_y * INV_SQRT3;
                let t_111 = (1.0 - t_010 - t_011) / 2.0;
                Ok(DutyCycle3Phase {
                    t1: t_111,
                    t2: t_010 + t_011 + t_111,
                    t3: t_011 + t_111,
                })
            }
            3 => {
                let t_011 = -v_x + v_y * INV_SQRT3;
                let t_001 = -v_y * 2.0 * INV_SQRT3;
                let t_111 = (1.0 - t_011 - t_001) / 2.0;
                Ok(DutyCycle3Phase {
                    t1: t_111,
                    t2: t_011 + t_111,
                    t3: t_011 + t_001 + t_111,
                })
            }
            4 => {
                let t_001 = -v_x - v_y * INV_SQRT3;
                let t_101 = v_x - v_y * INV_SQRT3;
                let t_111 = (1.0 - t_001 - t_101) / 2.0;
                Ok(DutyCycle3Phase {
                    t1: t_101 + t_111,
                    t2: t_111,
                    t3: t_001 + t_101 + t_111,
                })
            }
            5 => {
                let t_101 = -v_y * 2.0 * INV_SQRT3;
                let t_100 = v_x + v_y * INV_SQRT3;
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

    #[test]
    fn test_svpwm_valid_parameters() {
        let result = svpwm(1.0, 0.0, Radian::new(0.0), 2.0);
        assert!(result.is_ok());

        let duty_cycle = result.unwrap();
        assert!(duty_cycle.t1 >= 0.0 && duty_cycle.t1 <= 1.0);
        assert!(duty_cycle.t2 >= 0.0 && duty_cycle.t2 <= 1.0);
        assert!(duty_cycle.t3 >= 0.0 && duty_cycle.t3 <= 1.0);
    }

    #[test]
    fn test_svpwm_invalid_negative_vref() {
        let result = svpwm(-1.0, 0.0, Radian::new(0.0), 2.0);
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), FocError::InvalidParameters));
    }

    #[test]
    fn test_svpwm_invalid_zero_vmax() {
        let result = svpwm(1.0, 0.0, Radian::new(0.0), 0.0);
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), FocError::InvalidParameters));
    }

    #[test]
    fn test_svpwm_invalid_vref_greater_than_vmax() {
        let result = svpwm(3.0, 0.0, Radian::new(0.0), 2.0);
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), FocError::InvalidParameters));
    }

    #[test]
    fn test_svpwm_all_sectors() {
        let pi_third = core::f32::consts::PI / 3.0;

        for sector in 0..6 {
            let angle = sector as f32 * pi_third + 0.1;
            let result = svpwm(0.5, 0.0, Radian::new(angle), 1.0);
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
        let sqrt3_over_2 = HALF_SQRT3;

        // Test at theoretical maximum
        let v_ref = sqrt3_over_2;
        for sector in 0..6 {
            let angle = sector as f32 * pi_third + 0.1;
            let result = svpwm(v_ref, 0.0, Radian::new(angle), 1.0);
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
            let result = svpwm(v_ref, 0.0, Radian::new(angle), 1.0);
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
                let angle = Radian::new(angle_div_pi * core::f32::consts::PI);
                let duty_reference = svpwm_reference(v_ref, angle, v_max).unwrap();
                let duty = svpwm(v_ref, 0.0, angle, v_max).unwrap();

                assert!(
                    (duty_reference.t1 - duty.t1).abs() < 1e-6,
                    "t1: {:?} != {:?} at v_ref={}, angle={}",
                    duty_reference,
                    duty,
                    v_ref,
                    angle.angle
                );

                assert!(
                    (duty_reference.t2 - duty.t2).abs() < 1e-6,
                    "t2: {:?} != {:?} at v_ref={}, angle={}",
                    duty_reference,
                    duty,
                    v_ref,
                    angle.angle
                );

                assert!(
                    (duty_reference.t3 - duty.t3).abs() < 1e-6,
                    "t3: {:?} != {:?} at v_ref={}, angle={}",
                    duty_reference,
                    duty,
                    v_ref,
                    angle.angle
                );
            }
        }
    }
}
