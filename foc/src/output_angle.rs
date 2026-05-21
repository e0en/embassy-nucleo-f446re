use core::f32::consts::TAU;

/// Estimate output revolution count from wrapped primary and secondary encoder angles.
///
/// `angle_p` and `angle_s` must already be wrapped to `[0, TAU)`.
pub fn estimate_output_rev_count(angle_p: f32, angle_s: f32, reduction_ratio: i32) -> i32 {
    let reduction_ratio_f = reduction_ratio as f32;
    let rev_p = angle_p * (reduction_ratio_f + 1.0) / TAU;
    let rev_s = -angle_s * reduction_ratio_f / TAU;
    let mut rev_diff = rev_s - rev_p;
    while rev_diff < 0.0 {
        rev_diff += reduction_ratio_f;
    }

    round_positive(rev_diff) % reduction_ratio
}

#[inline]
fn round_positive(value: f32) -> i32 {
    (value + 0.5) as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    const REDUCTION_RATIO: i32 = 19;
    const EPSILON: f32 = 1e-4;
    const NOISE_AMPLITUDE: f32 = 0.01;
    const NOISY_EPSILON: f32 = 0.02;

    fn wrap(angle: f32) -> f32 {
        let wrapped = angle % TAU;
        if wrapped < 0.0 {
            wrapped + TAU
        } else {
            wrapped
        }
    }

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPSILON
    }

    fn reconstruct_output_angle(angle_p: f32, rev_count: i32) -> f32 {
        -(angle_p / REDUCTION_RATIO as f32 + rev_count as f32 * TAU / REDUCTION_RATIO as f32)
    }

    #[test]
    fn reconstructs_rev_count_from_ideal_sensor_pair() {
        let mut output_angle = -TAU;
        while output_angle <= 0.0 {
            let angle_p = wrap(-output_angle * REDUCTION_RATIO as f32);
            let angle_s = wrap(output_angle * (REDUCTION_RATIO as f32 + 1.0));
            let rev_count = estimate_output_rev_count(angle_p, angle_s, REDUCTION_RATIO);
            let estimate = reconstruct_output_angle(angle_p, rev_count);
            assert!(
                approx_eq(estimate, output_angle),
                "output_angle={output_angle}, angle_p={angle_p}, angle_s={angle_s}, rev_count={rev_count}, estimate={estimate}"
            );
            output_angle += 0.01;
        }
    }

    #[test]
    fn rev_count_is_stable_across_wrapped_primary_boundaries() {
        let base_output_angle = -1.0;
        let mut previous = None;

        for step in -20..=20 {
            let output_angle = base_output_angle + step as f32 * 0.001;
            let angle_p = wrap(-output_angle * REDUCTION_RATIO as f32);
            let angle_s = wrap(output_angle * (REDUCTION_RATIO as f32 + 1.0));
            let rev_count = estimate_output_rev_count(angle_p, angle_s, REDUCTION_RATIO);
            let estimate = reconstruct_output_angle(angle_p, rev_count);

            assert!(
                approx_eq(estimate, output_angle),
                "output_angle={output_angle}, angle_p={angle_p}, angle_s={angle_s}, rev_count={rev_count}, estimate={estimate}"
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
    fn wraps_rev_count_at_reduction_ratio_boundary() {
        let angle_p = 0.0;
        let angle_s = 0.0;
        let rev_count = estimate_output_rev_count(angle_p, angle_s, REDUCTION_RATIO);
        assert_eq!(rev_count, 0, "rev_count={rev_count}");
    }

    #[test]
    fn rev_count_tolerates_secondary_encoder_noise() {
        let mut output_angle = -TAU;
        let mut max_error = 0.0f32;

        while output_angle <= 0.0 {
            let angle_p = wrap(-output_angle * REDUCTION_RATIO as f32);
            let ideal_angle_s = wrap(output_angle * (REDUCTION_RATIO as f32 + 1.0));
            let noise = NOISE_AMPLITUDE * (output_angle * 17.0).sin();
            let angle_s = wrap(ideal_angle_s + noise);
            let rev_count = estimate_output_rev_count(angle_p, angle_s, REDUCTION_RATIO);
            let estimate = reconstruct_output_angle(angle_p, rev_count);
            let error = (estimate - output_angle).abs();

            max_error = max_error.max(error);
            output_angle += 0.01;
        }

        assert!(
            max_error < NOISY_EPSILON,
            "max_error={max_error}, tolerance={NOISY_EPSILON}"
        );
    }
}
