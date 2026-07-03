use core::f32::consts::TAU;

use crate::{
    cordic::atan2,
    encoder_correction::{EncoderCorrection, LUT_SIZE, wrap_0_tau, wrap_pm_pi},
};

pub fn circular_mean(a: f32, b: f32) -> f32 {
    wrap_0_tau(a + 0.5 * wrap_pm_pi(b - a))
}

pub fn average_wrapped_samples(sum_sin: f32, sum_cos: f32) -> Option<f32> {
    if sum_sin == 0.0 && sum_cos == 0.0 {
        None
    } else {
        Some(wrap_0_tau(atan2(sum_sin, sum_cos)))
    }
}

pub fn build_lut_from_samples(samples: &[(f32, f32)]) -> Option<EncoderCorrection> {
    if samples.is_empty() {
        return None;
    }

    let mut sum = [0.0; LUT_SIZE];
    let mut count = [0u16; LUT_SIZE];

    for &(measured, reference) in samples {
        let wrapped_measured = wrap_0_tau(measured);
        let error = wrap_pm_pi(reference - wrapped_measured);
        let index = ((wrapped_measured * (LUT_SIZE as f32) / TAU) as usize) % LUT_SIZE;
        sum[index] += error;
        count[index] += 1;
    }

    let mut lut = [0.0; LUT_SIZE];
    for i in 0..LUT_SIZE {
        if count[i] > 0 {
            lut[i] = sum[i] / (count[i] as f32);
        }
    }

    fill_missing_bins(&mut lut, &count)?;

    Some(EncoderCorrection {
        valid: true,
        error_lut: lut,
    })
}

fn fill_missing_bins(lut: &mut [f32; LUT_SIZE], count: &[u16; LUT_SIZE]) -> Option<()> {
    let mut filled = 0usize;
    for &c in count {
        if c > 0 {
            filled += 1;
        }
    }
    if filled < 4 {
        return None;
    }

    for i in 0..LUT_SIZE {
        if count[i] > 0 {
            continue;
        }

        let prev = find_filled_bin(count, i, -1)?;
        let next = find_filled_bin(count, i, 1)?;

        let prev_value = lut[prev];
        let next_value = lut[next];
        let span = if next >= prev {
            next - prev
        } else {
            LUT_SIZE + next - prev
        };
        let offset = if i >= prev {
            i - prev
        } else {
            LUT_SIZE + i - prev
        };
        let frac = offset as f32 / span as f32;
        lut[i] = prev_value + frac * wrap_pm_pi(next_value - prev_value);
    }
    Some(())
}

fn find_filled_bin(count: &[u16; LUT_SIZE], start: usize, direction: isize) -> Option<usize> {
    for offset in 1..=LUT_SIZE {
        let index = if direction > 0 {
            (start + offset) % LUT_SIZE
        } else {
            (start + LUT_SIZE - (offset % LUT_SIZE)) % LUT_SIZE
        };
        if count[index] > 0 {
            return Some(index);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builds_lut_from_sparse_samples() {
        let samples = [
            (0.0, 0.05),
            (TAU * 0.25, TAU * 0.25 + 0.05),
            (TAU * 0.5, TAU * 0.5 + 0.05),
            (TAU * 0.75, TAU * 0.75 + 0.05),
        ];
        let correction = build_lut_from_samples(&samples).unwrap();
        assert!(correction.valid);
        let corrected = correction.correct_wrapped_angle(0.0);
        assert!((corrected - 0.05).abs() < 0.02);
    }
}
