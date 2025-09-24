use core::f32::consts::PI;

use embassy_stm32::pac::cordic::vals;
use embassy_stm32::pac::{CORDIC, RCC};

const F32_MAX: f32 = 2_147_483_648.0;

#[inline]
fn q31_from_f32(x: f32) -> u32 {
    let y = x.clamp(-1.0, 0.99999994);
    (y * F32_MAX) as i32 as u32
}

#[inline]
fn f32_from_q31(x: u32) -> f32 {
    let i = i32::from_ne_bytes(x.to_ne_bytes());
    (i as f32) / F32_MAX
}

pub fn sincos(a: f32) -> (f32, f32) {
    CORDIC.csr().write(|w| {
        w.set_func(vals::Func::SINE);
        w.set_precision(vals::Precision::ITERS24);
        w.set_nargs(vals::Num::NUM1);
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
