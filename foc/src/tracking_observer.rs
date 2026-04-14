use core::f32::consts::TAU;

/// Second-order angle tracking observer (PLL) for velocity estimation.
///
/// Tracks measured angle and outputs smooth velocity via integration,
/// avoiding differentiation-based noise amplification.
///
/// Uses critically damped (zeta=1) pole placement:
///   Kp = 2 * wn
///   Ki = wn^2
///
/// where wn = 2*pi*bandwidth_hz.
///
/// The observer bandwidth should be well above the velocity PI bandwidth
/// to avoid adding phase lag, but low enough to reject encoder noise.
/// A typical ratio is 5-10x the velocity PI bandwidth.
pub struct TrackingObserver {
    pub angle: f32,
    pub velocity: f32,
    kp: f32,
    ki: f32,
    initialized: bool,
}

impl TrackingObserver {
    pub fn new(bandwidth_hz: f32) -> Self {
        let wn = TAU * bandwidth_hz;
        Self {
            angle: 0.0,
            velocity: 0.0,
            kp: 2.0 * wn,
            ki: wn * wn,
            initialized: false,
        }
    }

    /// Feed cumulative (unwrapped) measured angle, returns estimated velocity.
    pub fn update(&mut self, measured_angle: f32, dt: f32) -> f32 {
        if !self.initialized || dt <= 0.0 {
            self.angle = measured_angle;
            self.velocity = 0.0;
            self.initialized = true;
            return 0.0;
        }

        let error = measured_angle - self.angle;
        self.velocity += self.ki * error * dt;
        self.angle += (self.velocity + self.kp * error) * dt;
        self.velocity
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32, tolerance: f32) -> bool {
        (a - b).abs() < tolerance
    }

    #[test]
    fn tracks_constant_velocity() {
        let mut obs = TrackingObserver::new(20.0);
        let dt = 0.001;
        let target_velocity = 5.0;

        let mut angle = 0.0;
        let mut velocity = 0.0;
        for _ in 0..2000 {
            angle += target_velocity * dt;
            velocity = obs.update(angle, dt);
        }

        assert!(
            approx_eq(velocity, target_velocity, 0.01),
            "velocity={velocity}, expected={target_velocity}"
        );
    }

    #[test]
    fn smooth_output_with_quantized_input() {
        let mut obs = TrackingObserver::new(20.0);
        let dt = 0.00005; // 20kHz
        let target_velocity = 1.0;
        let encoder_resolution = TAU / 16384.0;

        let mut true_angle = 0.0;
        let mut max_velocity_deviation = 0.0f32;

        for i in 0..20000 {
            true_angle += target_velocity * dt;
            let quantized_angle = (true_angle / encoder_resolution).round() * encoder_resolution;
            let velocity = obs.update(quantized_angle, dt);

            if i > 10000 {
                max_velocity_deviation =
                    max_velocity_deviation.max((velocity - target_velocity).abs());
            }
        }

        assert!(
            max_velocity_deviation < 0.5,
            "max deviation={max_velocity_deviation}, too noisy"
        );
    }

    #[test]
    fn first_update_returns_zero() {
        let mut obs = TrackingObserver::new(20.0);
        let v = obs.update(1.0, 0.001);
        assert_eq!(v, 0.0);
    }
}
