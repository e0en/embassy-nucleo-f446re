use core::f32::consts::TAU;

/// Resolve wrapped output phase from wrapped primary and secondary encoder phases.
///
/// `angle_p` and `angle_s` must already be wrapped to `[0, TAU)`.
/// Returns the wrapped output phase in `[0, TAU)`.
pub fn resolve_output_phase(angle_p: f32, angle_s: f32, reduction_ratio: i32) -> f32 {
    debug_assert!((0.0..TAU).contains(&angle_p));
    debug_assert!((0.0..TAU).contains(&angle_s));
    debug_assert_ne!(reduction_ratio, 0);

    let reduction_ratio_f = reduction_ratio as f32;
    let reduction_ratio_abs = reduction_ratio.abs();
    let reduction_ratio_abs_f = reduction_ratio_abs as f32;
    let rev_p = angle_p * (reduction_ratio_f + 1.0) / TAU;
    let rev_s = -angle_s * reduction_ratio_f / TAU;
    let mut rev_diff = rev_s - rev_p;
    while rev_diff < 0.0 {
        rev_diff += reduction_ratio_abs_f;
    }

    let rev_count = round_positive(rev_diff).rem_euclid(reduction_ratio_abs);
    wrap(-(angle_p / reduction_ratio_f + rev_count as f32 * TAU / reduction_ratio_f))
}

#[inline]
fn round_positive(value: f32) -> i32 {
    (value + 0.5) as i32
}

#[inline]
fn wrap(angle: f32) -> f32 {
    let wrapped = angle % TAU;
    if wrapped < 0.0 {
        wrapped + TAU
    } else {
        wrapped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 1e-4;
    const NOISE_AMPLITUDE: f32 = 0.01;
    const NOISY_EPSILON: f32 = 0.02;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPSILON
    }

    fn primary_phase(output_phase: f32, reduction_ratio: i32) -> f32 {
        wrap(-output_phase * reduction_ratio as f32)
    }

    fn secondary_phase(output_phase: f32, reduction_ratio: i32) -> f32 {
        wrap(output_phase * (reduction_ratio as f32 + 1.0))
    }

    #[test]
    fn reconstructs_wrapped_output_phase_from_ideal_sensor_pair() {
        let reduction_ratio = 19;
        let mut output_phase = 0.0;
        while output_phase < TAU {
            let angle_p = primary_phase(output_phase, reduction_ratio);
            let angle_s = secondary_phase(output_phase, reduction_ratio);
            let estimate = resolve_output_phase(angle_p, angle_s, reduction_ratio);
            assert!(
                approx_eq(estimate, output_phase),
                "output_phase={output_phase}, angle_p={angle_p}, angle_s={angle_s}, estimate={estimate}"
            );
            output_phase += 0.01;
        }
    }

    #[test]
    fn resolves_phase_across_wrapped_primary_boundaries() {
        let reduction_ratio = 19;
        let base_output_phase = wrap(1.0);
        let mut previous = None;

        for step in -20..=20 {
            let output_phase = wrap(base_output_phase + step as f32 * 0.001);
            let angle_p = primary_phase(output_phase, reduction_ratio);
            let angle_s = secondary_phase(output_phase, reduction_ratio);
            let estimate = resolve_output_phase(angle_p, angle_s, reduction_ratio);

            assert!(
                approx_eq(estimate, output_phase),
                "output_phase={output_phase}, angle_p={angle_p}, angle_s={angle_s}, estimate={estimate}"
            );

            if let Some(previous_estimate) = previous {
                assert!(
                    estimate > previous_estimate,
                    "previous={previous_estimate}, current={estimate}"
                );
            }
            previous = Some(estimate);
        }
    }

    #[test]
    fn resolves_phase_at_wrap_boundaries() {
        let reduction_ratio = 19;
        for output_phase in [0.0, EPSILON, TAU - EPSILON] {
            let angle_p = primary_phase(output_phase, reduction_ratio);
            let angle_s = secondary_phase(output_phase, reduction_ratio);
            let estimate = resolve_output_phase(angle_p, angle_s, reduction_ratio);
            assert!(
                approx_eq(estimate, output_phase),
                "output_phase={output_phase}, angle_p={angle_p}, angle_s={angle_s}, estimate={estimate}"
            );
        }
    }

    #[test]
    fn tolerates_secondary_encoder_noise() {
        let reduction_ratio = 19;
        let mut output_phase = 0.0;
        let mut max_error = 0.0f32;

        while output_phase < TAU {
            let angle_p = primary_phase(output_phase, reduction_ratio);
            let ideal_angle_s = secondary_phase(output_phase, reduction_ratio);
            let noise = NOISE_AMPLITUDE * (output_phase * 17.0).sin();
            let angle_s = wrap(ideal_angle_s + noise);
            let estimate = resolve_output_phase(angle_p, angle_s, reduction_ratio);
            let error = (estimate - output_phase)
                .abs()
                .min(TAU - (estimate - output_phase).abs());

            max_error = max_error.max(error);
            output_phase += 0.01;
        }

        assert!(
            max_error < NOISY_EPSILON,
            "max_error={max_error}, tolerance={NOISY_EPSILON}"
        );
    }

    #[test]
    fn preserves_signed_reduction_ratio_semantics() {
        let reduction_ratio = -19;
        let mut output_phase = 0.0;

        while output_phase < TAU {
            let angle_p = primary_phase(output_phase, reduction_ratio);
            let angle_s = secondary_phase(output_phase, reduction_ratio);
            let estimate = resolve_output_phase(angle_p, angle_s, reduction_ratio);

            assert!(
                approx_eq(estimate, output_phase),
                "output_phase={output_phase}, angle_p={angle_p}, angle_s={angle_s}, estimate={estimate}"
            );

            output_phase += 0.01;
        }
    }
}
