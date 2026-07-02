use core::f32::consts::{PI, SQRT_2};

use embassy_stm32::pac::cordic::vals;
use embassy_stm32::pac::{CORDIC, RCC};

const Q31_SCALE: f32 = 2_147_483_648.0;
/// Maximum Q31 representable value (slightly less than 1.0)
const Q31_MAX_INPUT: f32 = 0.99999994;

#[inline]
fn q31_from_f32(x: f32) -> u32 {
    let y = x.clamp(-1.0, Q31_MAX_INPUT);
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

    let arg_q31 = q31_from_f32(scaled_input.clamp(0.0, Q31_MAX_INPUT));
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
