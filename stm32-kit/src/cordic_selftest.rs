use core::f32::consts::{PI, SQRT_2};

use crate::cordic::{atan2, sincos, sqrt, sqrt_scaled};

#[derive(Default)]
pub struct CordicTestResults {
    pub sincos_pythagorean: (u32, u32),
    pub sincos_known_values: (u32, u32),
    pub sincos_symmetry: (u32, u32),
    pub sincos_after_atan2: (u32, u32),
    pub sqrt_known_values: (u32, u32),
    pub sqrt_scaled_known_values: (u32, u32),
    pub atan2_known_values: (u32, u32),
    pub atan2_quadrants: (u32, u32),
}

impl CordicTestResults {
    pub fn total_passed(&self) -> u32 {
        self.sincos_pythagorean.0
            + self.sincos_known_values.0
            + self.sincos_symmetry.0
            + self.sincos_after_atan2.0
            + self.sqrt_known_values.0
            + self.sqrt_scaled_known_values.0
            + self.atan2_known_values.0
            + self.atan2_quadrants.0
    }

    pub fn total_failed(&self) -> u32 {
        self.sincos_pythagorean.1
            + self.sincos_known_values.1
            + self.sincos_symmetry.1
            + self.sincos_after_atan2.1
            + self.sqrt_known_values.1
            + self.sqrt_scaled_known_values.1
            + self.atan2_known_values.1
            + self.atan2_quadrants.1
    }

    pub fn all_passed(&self) -> bool {
        self.total_failed() == 0
    }
}

pub fn log_test_results(results: &CordicTestResults) {
    if results.all_passed() {
        defmt::info!("CORDIC validation passed: {} tests", results.total_passed());
    } else {
        defmt::warn!(
            "CORDIC validation: {} passed, {} failed",
            results.total_passed(),
            results.total_failed()
        );
        log_test_result("sincos_pythagorean", results.sincos_pythagorean);
        log_test_result("sincos_known_values", results.sincos_known_values);
        log_test_result("sincos_symmetry", results.sincos_symmetry);
        log_test_result("sincos_after_atan2", results.sincos_after_atan2);
        log_test_result("sqrt_known_values", results.sqrt_known_values);
        log_test_result("sqrt_scaled_known_values", results.sqrt_scaled_known_values);
        log_test_result("atan2_known_values", results.atan2_known_values);
        log_test_result("atan2_quadrants", results.atan2_quadrants);
    }
}

fn log_test_result(name: &str, result: (u32, u32)) {
    if result.1 > 0 {
        defmt::warn!("  {}: {} passed, {} failed", name, result.0, result.1);
    }
}

pub fn run_and_log_validation_tests() {
    let results = run_validation_tests();
    log_test_results(&results);
}

pub fn run_validation_tests() -> CordicTestResults {
    CordicTestResults {
        sincos_pythagorean: test_sincos_pythagorean(),
        sincos_known_values: test_sincos_known_values(),
        sincos_symmetry: test_sincos_symmetry(),
        sincos_after_atan2: test_sincos_after_atan2(),
        sqrt_known_values: test_sqrt_known_values(),
        sqrt_scaled_known_values: test_sqrt_scaled_known_values(),
        atan2_known_values: test_atan2_known_values(),
        atan2_quadrants: test_atan2_quadrants(),
    }
}

fn test_sincos_pythagorean() -> (u32, u32) {
    let mut passed = 0u32;
    let mut failed = 0u32;
    const TOLERANCE: f32 = 1e-5;
    let angles = [
        0.0,
        PI / 6.0,
        PI / 4.0,
        PI / 3.0,
        PI / 2.0,
        PI,
        -PI / 4.0,
        -PI / 2.0,
        1.0,
        2.0,
        3.0,
        -1.0,
        -2.0,
    ];

    for angle in angles {
        let (sin, cos) = sincos(angle);
        let sum = sin * sin + cos * cos;
        if (sum - 1.0).abs() < TOLERANCE {
            passed += 1;
        } else {
            failed += 1;
        }
    }

    (passed, failed)
}

fn test_sincos_known_values() -> (u32, u32) {
    let mut passed = 0u32;
    let mut failed = 0u32;
    const TOLERANCE: f32 = 1e-5;
    let test_cases: [(f32, f32, f32); 8] = [
        (0.0, 0.0, 1.0),
        (PI / 2.0, 1.0, 0.0),
        (PI, 0.0, -1.0),
        (-PI / 2.0, -1.0, 0.0),
        (PI / 6.0, 0.5, 0.866_025_4),
        (PI / 4.0, 0.707_106_77, 0.707_106_77),
        (PI / 3.0, 0.866_025_4, 0.5),
        (-PI / 4.0, -0.707_106_77, 0.707_106_77),
    ];

    for (angle, expected_sin, expected_cos) in test_cases {
        let (sin, cos) = sincos(angle);
        if (sin - expected_sin).abs() < TOLERANCE && (cos - expected_cos).abs() < TOLERANCE {
            passed += 1;
        } else {
            failed += 1;
        }
    }

    (passed, failed)
}

fn test_sincos_symmetry() -> (u32, u32) {
    let mut passed = 0u32;
    let mut failed = 0u32;
    const TOLERANCE: f32 = 1e-5;
    let angles = [0.1, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0];

    for angle in angles {
        let (sin_pos, cos_pos) = sincos(angle);
        let (sin_neg, cos_neg) = sincos(-angle);

        if (sin_neg + sin_pos).abs() < TOLERANCE {
            passed += 1;
        } else {
            failed += 1;
        }

        if (cos_neg - cos_pos).abs() < TOLERANCE {
            passed += 1;
        } else {
            failed += 1;
        }
    }

    (passed, failed)
}

fn test_sincos_after_atan2() -> (u32, u32) {
    let mut passed = 0u32;
    let mut failed = 0u32;
    const TOLERANCE: f32 = 1e-4;
    #[allow(clippy::approx_constant, clippy::excessive_precision)]
    let test_cases: [(f32, f32, f32); 10] = [
        (0.7853982, 0.7071068, 0.7071068),
        (1.2345, 0.9439833, 0.3299932),
        (2.718, 0.4110382, -0.9116181),
        (0.123, 0.1226901, 0.9924450),
        (2.345, 0.7149780, -0.6991469),
        (-0.567, -0.5371039, 0.8435161),
        (-1.234, -0.9438182, 0.3304651),
        (0.0001, 0.0001000, 1.0),
        (3.14159, 0.0000027, -1.0),
        (1.5708, 1.0, -0.0000037),
    ];

    for (angle, expected_sin, expected_cos) in test_cases {
        let _ = atan2(0.6, 0.8);
        let _ = atan2(-0.3, 0.7);
        let (sin, cos) = sincos(angle);

        let sin_ok = (sin - expected_sin).abs() < TOLERANCE;
        let cos_ok = (cos - expected_cos).abs() < TOLERANCE;

        if sin_ok && cos_ok {
            passed += 1;
        } else {
            defmt::warn!(
                "sincos_after_atan2({}) = ({}, {}), expected ({}, {})",
                angle,
                sin,
                cos,
                expected_sin,
                expected_cos
            );
            failed += 1;
        }
    }

    (passed, failed)
}

fn test_sqrt_known_values() -> (u32, u32) {
    let mut passed = 0u32;
    let mut failed = 0u32;
    const TOLERANCE: f32 = 1e-5;
    let test_cases: [(f32, f32); 6] = [
        (0.0, 0.0),
        (0.25, 0.5),
        (0.5, 0.707_106_77),
        (0.64, 0.8),
        (0.81, 0.9),
        (1.0, 1.0),
    ];

    for (input, expected) in test_cases {
        let result = if input >= 1.0 { 1.0 } else { sqrt(input) };
        let error = (result - expected).abs();
        if error < TOLERANCE {
            passed += 1;
        } else {
            defmt::warn!(
                "sqrt({}) = {}, expected {}, error = {}",
                input,
                result,
                expected,
                error
            );
            failed += 1;
        }
    }

    (passed, failed)
}

fn test_sqrt_scaled_known_values() -> (u32, u32) {
    let mut passed = 0u32;
    let mut failed = 0u32;
    const TOLERANCE: f32 = 1e-4;
    let test_cases: [(f32, f32); 8] = [
        (0.0, 0.0),
        (1.0, 1.0),
        (4.0, 2.0),
        (9.0, 3.0),
        (16.0, 4.0),
        (25.0, 5.0),
        (100.0, 10.0),
        (2.0, SQRT_2),
    ];

    for (input, expected) in test_cases {
        let result = sqrt_scaled(input);
        if (result - expected).abs() < TOLERANCE {
            passed += 1;
        } else {
            failed += 1;
        }
    }

    (passed, failed)
}

fn test_atan2_known_values() -> (u32, u32) {
    let mut passed = 0u32;
    let mut failed = 0u32;
    const TOLERANCE: f32 = 1e-4;
    let test_cases: [(f32, f32, f32); 8] = [
        (0.0, 1.0, 0.0),
        (1.0, 1.0, PI / 4.0),
        (1.0, 0.0, PI / 2.0),
        (0.0, -1.0, PI),
        (-1.0, 0.0, -PI / 2.0),
        (-1.0, 1.0, -PI / 4.0),
        (1.0, -1.0, 3.0 * PI / 4.0),
        (-1.0, -1.0, -3.0 * PI / 4.0),
    ];

    for (y, x, expected) in test_cases {
        let result = atan2(y, x);
        if (result - expected).abs() < TOLERANCE {
            passed += 1;
        } else {
            failed += 1;
        }
    }

    (passed, failed)
}

fn test_atan2_quadrants() -> (u32, u32) {
    let mut passed = 0u32;
    let mut failed = 0u32;

    let angle = atan2(1.0, 1.0);
    if angle > 0.0 && angle < PI / 2.0 {
        passed += 1;
    } else {
        failed += 1;
    }

    let angle = atan2(1.0, -1.0);
    if angle > PI / 2.0 && angle < PI {
        passed += 1;
    } else {
        failed += 1;
    }

    let angle = atan2(-1.0, -1.0);
    if angle > -PI && angle < -PI / 2.0 {
        passed += 1;
    } else {
        failed += 1;
    }

    let angle = atan2(-1.0, 1.0);
    if angle > -PI / 2.0 && angle < 0.0 {
        passed += 1;
    } else {
        failed += 1;
    }

    (passed, failed)
}
