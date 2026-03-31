use crate::pid::PID;

/// Least-squares slope of v vs t
pub fn linear_regression_slope(t: &[f32], v: &[f32]) -> f32 {
    let n = t.len() as f32;
    let mut t_mean = 0.0;
    let mut v_mean = 0.0;
    for i in 0..t.len() {
        t_mean += t[i];
        v_mean += v[i];
    }
    t_mean /= n;
    v_mean /= n;

    let mut num = 0.0;
    let mut den = 0.0;
    for i in 0..t.len() {
        let dt = t[i] - t_mean;
        num += dt * (v[i] - v_mean);
        den += dt * dt;
    }
    if den.abs() < 1e-12 {
        return 0.0;
    }
    num / den
}

/// PI gains for velocity loop on an integrator plant G(s) = K_plant / s.
/// Pole placement with critically damped response (zeta = 1.0).
pub fn calculate_velocity_pi(k_plant: f32, bandwidth_hz: f32) -> PID {
    let omega_n = core::f32::consts::TAU * bandwidth_hz;
    let zeta = 1.0;
    PID {
        p: 2.0 * zeta * omega_n / k_plant,
        i: omega_n * omega_n / k_plant,
        d: 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear_regression_on_constant_velocity() {
        let t: Vec<f32> = (0..100).map(|i| i as f32 * 0.001).collect();
        let v: Vec<f32> = t.iter().map(|_| 5.0).collect();
        let slope = linear_regression_slope(&t, &v);
        assert!(slope.abs() < 1e-3, "slope={}", slope);
    }

    #[test]
    fn linear_regression_on_constant_acceleration() {
        let accel = 50.0;
        let t: Vec<f32> = (0..256).map(|i| i as f32 * 0.001).collect();
        let v: Vec<f32> = t.iter().map(|ti| accel * ti).collect();
        let slope = linear_regression_slope(&t, &v);
        assert!(
            (slope - accel).abs() < 0.1,
            "expected ~{}, got {}",
            accel,
            slope
        );
    }

    #[test]
    fn velocity_pi_gains_scale_with_bandwidth() {
        let k_plant = 10.0;
        let low = calculate_velocity_pi(k_plant, 5.0);
        let high = calculate_velocity_pi(k_plant, 10.0);
        assert!(high.p > low.p);
        assert!(high.i > low.i);
    }

    #[test]
    fn velocity_pi_gains_inversely_proportional_to_k_plant() {
        let bw = 10.0;
        let small = calculate_velocity_pi(5.0, bw);
        let large = calculate_velocity_pi(20.0, bw);
        assert!(small.p > large.p);
        assert!(small.i > large.i);
    }

    #[test]
    fn velocity_pi_known_values() {
        let gains = calculate_velocity_pi(10.0, 10.0);
        let omega_n = core::f32::consts::TAU * 10.0;
        let expected_kp = 2.0 * omega_n / 10.0;
        let expected_ki = omega_n * omega_n / 10.0;
        assert!(
            (gains.p - expected_kp).abs() < 0.01,
            "kp: expected {}, got {}",
            expected_kp,
            gains.p
        );
        assert!(
            (gains.i - expected_ki).abs() < 0.1,
            "ki: expected {}, got {}",
            expected_ki,
            gains.i
        );
        assert!((gains.d - 0.0).abs() < 1e-6);
    }
}
