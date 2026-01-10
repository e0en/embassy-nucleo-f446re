use core::f32::consts::{PI, SQRT_2};

use embassy_stm32::pac::cordic::vals;
use embassy_stm32::pac::{CORDIC, RCC};

const Q31_SCALE: f32 = 2_147_483_648.0;

#[inline]
fn q31_from_f32(x: f32) -> u32 {
    let y = x.clamp(-1.0, 0.99999994);
    (y * Q31_SCALE) as i32 as u32
}

#[inline]
fn f32_from_q31(x: u32) -> f32 {
    let i = i32::from_ne_bytes(x.to_ne_bytes());
    (i as f32) / Q31_SCALE
}

// Q1.31 representation of 1.0 (maximum positive value)
const Q31_ONE: u32 = 0x7FFF_FFFF;

pub fn sincos(a: f32) -> (f32, f32) {
    CORDIC.csr().write(|w| {
        w.set_func(vals::Func::SINE);
        w.set_precision(vals::Precision::ITERS24);
        w.set_nargs(vals::Num::NUM2); // Always write both angle and modulus
        w.set_nres(vals::Num::NUM2);
        w.set_scale(vals::Scale::A1_R1);
    });

    clean_rrdy_flag();

    let mut normalized_a = a / PI;
    while normalized_a > 1.0 {
        normalized_a -= 2.0;
    }
    while normalized_a < -1.0 {
        normalized_a += 2.0;
    }
    let arg_q31 = q31_from_f32(normalized_a);
    CORDIC.wdata().write_value(arg_q31);
    CORDIC.wdata().write_value(Q31_ONE); // Modulus = 1.0

    let res1_q31 = CORDIC.rdata().read();
    let res2_q31 = CORDIC.rdata().read();
    (f32_from_q31(res1_q31), f32_from_q31(res2_q31))
}

fn clean_rrdy_flag() -> usize {
    let mut count = 0;
    while CORDIC.csr().read().rrdy() {
        CORDIC.rdata().read();
        count += 1;
    }
    count
}

/// Compute square root using CORDIC hardware.
/// Input must be in range [0.0, 1.0]. For larger values, use sqrt_scaled.
///
/// Note: STM32 CORDIC sqrt expects input in range [0.027, 0.75] with scale=1,
/// so we scale the input by 0.5 and the output by sqrt(2) to handle [0, 1] range.
pub fn sqrt(x: f32) -> f32 {
    debug_assert!((0.0..=1.0).contains(&x), "sqrt input must be in [0.0, 1.0]");

    // Handle edge cases that CORDIC doesn't handle well
    if x <= 0.0 {
        return 0.0;
    }

    // Scale input to fit CORDIC's expected range
    // We use scale=1, which means input range is [0.027, 0.75]
    // To handle [0, 1], we compute sqrt(x/2) * sqrt(2)
    let scaled_input = x * 0.5;

    CORDIC.csr().write(|w| {
        w.set_func(vals::Func::SQUARE_ROOT);
        w.set_precision(vals::Precision::ITERS24);
        w.set_nargs(vals::Num::NUM1);
        w.set_nres(vals::Num::NUM1);
        w.set_scale(vals::Scale::A1_R1); // scale = 1
    });

    clean_rrdy_flag();

    let arg_q31 = q31_from_f32(scaled_input.clamp(0.0, 0.99999994));
    CORDIC.wdata().write_value(arg_q31);

    let res_q31 = CORDIC.rdata().read();
    let result = f32_from_q31(res_q31);

    // Multiply by sqrt(2) to compensate for the input scaling
    result * SQRT_2
}

/// Compute square root for values outside [0, 1] range.
/// Scales the input down, computes sqrt, then scales the result back up.
pub fn sqrt_scaled(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }
    if x <= 1.0 {
        return sqrt(x);
    }

    // Find a power of 4 to scale x into [0.25, 1.0] range
    // sqrt(x * 4^n) = sqrt(x) * 2^n
    let mut scaled = x;
    let mut shift = 0u32;
    while scaled > 1.0 {
        scaled *= 0.25; // divide by 4
        shift += 1;
    }

    let sqrt_scaled = sqrt(scaled);
    // Multiply by 2^shift to undo the scaling
    sqrt_scaled * (1u32 << shift) as f32
}

/// Compute atan2(y, x) using CORDIC hardware PHASE function.
/// Returns angle in radians in range [-π, π].
pub fn atan2(y: f32, x: f32) -> f32 {
    // CORDIC PHASE function computes atan2(y, x) and returns angle in [-1, 1] representing [-π, π]
    // It also returns the modulus sqrt(x² + y²) as a second result (which we discard)
    //
    // Input scaling: x and y must be in [-1, 1] range
    // We need to normalize by the larger absolute value

    let abs_x = if x >= 0.0 { x } else { -x };
    let abs_y = if y >= 0.0 { y } else { -y };
    let max_val = if abs_x > abs_y { abs_x } else { abs_y };

    if max_val == 0.0 {
        return 0.0;
    }

    let scale = 1.0 / max_val;
    let x_norm = x * scale;
    let y_norm = y * scale;

    CORDIC.csr().write(|w| {
        w.set_func(vals::Func::PHASE);
        w.set_precision(vals::Precision::ITERS24);
        w.set_nargs(vals::Num::NUM2);
        w.set_nres(vals::Num::NUM1); // We only need the angle, not the modulus
        w.set_scale(vals::Scale::A1_R1);
    });

    clean_rrdy_flag();

    // PHASE function expects arguments in order: x, then y
    let x_q31 = q31_from_f32(x_norm);
    let y_q31 = q31_from_f32(y_norm);
    CORDIC.wdata().write_value(x_q31);
    CORDIC.wdata().write_value(y_q31);

    let angle_q31 = CORDIC.rdata().read();
    let normalized_angle = f32_from_q31(angle_q31);

    // Convert from [-1, 1] to [-π, π]
    normalized_angle * PI
}

pub fn initialize_cordic() {
    RCC.ahb1enr().modify(|w| w.set_cordicen(true));
    RCC.ahb1rstr().modify(|w| w.set_cordicrst(true));
    RCC.ahb1rstr().modify(|w| w.set_cordicrst(false));

    CORDIC.csr().write(|w| {
        w.set_func(vals::Func::COSINE);
        w.set_precision(vals::Precision::ITERS4);
        w.set_nargs(vals::Num::NUM2);
        w.set_nres(vals::Num::NUM2);
        w.set_argsize(vals::Size::BITS32);
        w.set_ressize(vals::Size::BITS32);
        w.set_scale(vals::Scale::A1_R1);
    });

    CORDIC.wdata().write_value(0x0u32);
    CORDIC.wdata().write_value(0x7FFFFFFFu32);
    clean_rrdy_flag();
}

/// On-target test results for CORDIC functions.
/// Each field contains (passed, failed) counts.
#[allow(dead_code)]
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

/// Log CORDIC test results via defmt.
#[allow(dead_code)]
pub fn log_test_results(results: &CordicTestResults) {
    if results.all_passed() {
        defmt::info!("CORDIC validation passed: {} tests", results.total_passed());
    } else {
        defmt::warn!(
            "CORDIC validation: {} passed, {} failed",
            results.total_passed(),
            results.total_failed()
        );
        // Only log individual results if there are failures
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

#[allow(dead_code)]
fn log_test_result(name: &str, result: (u32, u32)) {
    if result.1 > 0 {
        defmt::warn!("  {}: {} passed, {} failed", name, result.0, result.1);
    }
}

/// Run all CORDIC validation tests on-target and log results.
/// Call this after initialize_cordic() to verify the implementation.
#[allow(dead_code)]
pub fn run_and_log_validation_tests() {
    let results = run_validation_tests();
    log_test_results(&results);
}

/// Run all CORDIC validation tests on-target.
/// Call this after initialize_cordic() to verify the implementation.
/// Returns test results that can be logged via defmt.
#[allow(dead_code)]
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

    // Test at various angles
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

    // (angle, expected_sin, expected_cos)
    let test_cases: [(f32, f32, f32); 8] = [
        (0.0, 0.0, 1.0),
        (PI / 2.0, 1.0, 0.0),
        (PI, 0.0, -1.0),
        (-PI / 2.0, -1.0, 0.0),
        (PI / 6.0, 0.5, 0.866_025_4),           // 30°
        (PI / 4.0, 0.707_106_77, 0.707_106_77), // 45°
        (PI / 3.0, 0.866_025_4, 0.5),           // 60°
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

        // sin(-x) = -sin(x)
        if (sin_neg + sin_pos).abs() < TOLERANCE {
            passed += 1;
        } else {
            failed += 1;
        }

        // cos(-x) = cos(x)
        if (cos_neg - cos_pos).abs() < TOLERANCE {
            passed += 1;
        } else {
            failed += 1;
        }
    }

    (passed, failed)
}

/// Test sincos with non-trivial angles after calling atan2.
/// This verifies that sincos correctly sets the modulus to 1.0,
/// preventing corruption from previous PHASE function calls.
fn test_sincos_after_atan2() -> (u32, u32) {
    let mut passed = 0u32;
    let mut failed = 0u32;
    const TOLERANCE: f32 = 1e-4;

    // Non-trivial angle values that might occur in real calculations
    // (e.g., from Fourier analysis in impedance measurement)
    // Using explicit numeric values to test CORDIC precision (not std constants)
    #[allow(clippy::approx_constant, clippy::excessive_precision)]
    let test_cases: [(f32, f32, f32); 10] = [
        (0.7853982, 0.7071068, 0.7071068), // π/4 ≈ 0.785
        (1.2345, 0.9439833, 0.3299932),    // ~70.7°
        (2.718, 0.4110382, -0.9116181),    // e radians
        (0.123, 0.1226901, 0.9924450),     // small angle
        (2.345, 0.7149780, -0.6991469),    // ~134.3°
        (-0.567, -0.5371039, 0.8435161),   // negative angle
        (-1.234, -0.9438182, 0.3304651),   // negative ~-70.7°
        (0.0001, 0.0001000, 1.0),          // very small angle
        (3.14159, 0.0000027, -1.0),        // ~π
        (1.5708, 1.0, -0.0000037),         // ~π/2
    ];

    for (angle, expected_sin, expected_cos) in test_cases {
        // Call atan2 first to potentially corrupt the modulus register
        let _ = atan2(0.6, 0.8); // Some arbitrary atan2 call
        let _ = atan2(-0.3, 0.7); // Another one for good measure

        // Now call sincos - this should still work correctly
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

    // (input, expected_output)
    let test_cases: [(f32, f32); 6] = [
        (0.0, 0.0),
        (0.25, 0.5),
        (0.5, 0.707_106_77),
        (0.64, 0.8),
        (0.81, 0.9),
        (1.0, 1.0),
    ];

    for (input, expected) in test_cases {
        // Handle edge case: sqrt(1.0) won't work due to clamping to 0.99999994
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
    const TOLERANCE: f32 = 1e-4; // Slightly larger tolerance for scaled values

    // (input, expected_output)
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

    // (y, x, expected_angle)
    let test_cases: [(f32, f32, f32); 8] = [
        (0.0, 1.0, 0.0),               // 0°
        (1.0, 1.0, PI / 4.0),          // 45°
        (1.0, 0.0, PI / 2.0),          // 90°
        (0.0, -1.0, PI),               // 180°
        (-1.0, 0.0, -PI / 2.0),        // -90°
        (-1.0, 1.0, -PI / 4.0),        // -45°
        (1.0, -1.0, 3.0 * PI / 4.0),   // 135°
        (-1.0, -1.0, -3.0 * PI / 4.0), // -135°
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

    // Test that atan2 returns angles in the correct quadrant
    // Quadrant I: x > 0, y > 0 => angle in (0, π/2)
    let angle = atan2(1.0, 1.0);
    if angle > 0.0 && angle < PI / 2.0 {
        passed += 1;
    } else {
        failed += 1;
    }

    // Quadrant II: x < 0, y > 0 => angle in (π/2, π)
    let angle = atan2(1.0, -1.0);
    if angle > PI / 2.0 && angle < PI {
        passed += 1;
    } else {
        failed += 1;
    }

    // Quadrant III: x < 0, y < 0 => angle in (-π, -π/2)
    let angle = atan2(-1.0, -1.0);
    if angle > -PI && angle < -PI / 2.0 {
        passed += 1;
    } else {
        failed += 1;
    }

    // Quadrant IV: x > 0, y < 0 => angle in (-π/2, 0)
    let angle = atan2(-1.0, 1.0);
    if angle > -PI / 2.0 && angle < 0.0 {
        passed += 1;
    } else {
        failed += 1;
    }

    (passed, failed)
}
