#[derive(Copy, Clone)]
pub struct PID {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

pub struct PIDController {
    pub gains: PID,

    pub max_output: f32,
    pub max_rate: Option<f32>,

    pub integral: f32,
    pub last_error: f32,
    pub last_output: f32,
}

impl PIDController {
    pub fn new(pid: PID, max_output: f32, max_rate: Option<f32>) -> Self {
        PIDController {
            gains: pid,
            max_output,
            max_rate,
            integral: 0.0,
            last_error: 0.0,
            last_output: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = 0.0;
        self.last_output = 0.0;
    }

    pub fn set_max_output(&mut self, max_output: f32) {
        self.max_output = max_output;
        self.last_output = self.last_output.min(max_output).max(-self.max_output);
        self.integral = self.integral.min(self.max_output).max(-self.max_output);
    }

    pub fn update(&mut self, error: f32, dt_seconds: f32) -> f32 {
        let d_term = if dt_seconds > 0.0 {
            self.gains.d * (error - self.last_error) / dt_seconds
        } else {
            0.0
        };

        let candidate_integral = (self.integral
            + 0.5 * self.gains.i * (error + self.last_error) * dt_seconds)
            .min(self.max_output)
            .max(-self.max_output);

        let unclamped_output = self.gains.p * error + candidate_integral + d_term;
        let drives_further_into_positive_sat = unclamped_output > self.max_output && error > 0.0;
        let drives_further_into_negative_sat = unclamped_output < -self.max_output && error < 0.0;

        if !(drives_further_into_positive_sat || drives_further_into_negative_sat) {
            self.integral = candidate_integral;
        }

        let mut output = self.gains.p * error + self.integral + d_term;
        output = output.min(self.max_output).max(-self.max_output);

        if let Some(max_rate) = self.max_rate
            && dt_seconds > 0.0
        {
            let output_rate = (output - self.last_output) / dt_seconds;
            if output_rate > max_rate {
                output = self.last_output + max_rate * dt_seconds;
            } else if output_rate < -max_rate {
                output = self.last_output - max_rate * dt_seconds;
            }
        }

        self.last_error = error;
        self.last_output = output;
        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32, epsilon: f32) -> bool {
        (a - b).abs() < epsilon
    }

    #[test]
    fn proportional_gain_scales_error() {
        let pid = PID {
            p: 2.0,
            i: 0.0,
            d: 0.0,
        };
        let mut controller = PIDController::new(pid, 100.0, None);

        let output = controller.update(5.0, 0.01);

        assert!(approx_eq(output, 10.0, 0.001));
    }

    #[test]
    fn integral_accumulates_over_time() {
        let pid = PID {
            p: 0.0,
            i: 1.0,
            d: 0.0,
        };
        let mut controller = PIDController::new(pid, 100.0, None);

        controller.update(1.0, 0.1);
        let output = controller.update(1.0, 0.1);

        // Trapezoidal integration: 0.5 * 1.0 * (1.0 + 0.0) * 0.1 + 0.5 * 1.0 * (1.0 + 1.0) * 0.1
        assert!(approx_eq(output, 0.15, 0.001));
    }

    #[test]
    fn derivative_responds_to_error_change() {
        let pid = PID {
            p: 0.0,
            i: 0.0,
            d: 1.0,
        };
        let mut controller = PIDController::new(pid, 100.0, None);

        controller.update(0.0, 0.1);
        let output = controller.update(1.0, 0.1);

        // d_term = 1.0 * (1.0 - 0.0) / 0.1 = 10.0
        assert!(approx_eq(output, 10.0, 0.001));
    }

    #[test]
    fn output_clamped_to_max() {
        let pid = PID {
            p: 100.0,
            i: 0.0,
            d: 0.0,
        };
        let mut controller = PIDController::new(pid, 5.0, None);

        let output = controller.update(1.0, 0.01);

        assert!(approx_eq(output, 5.0, 0.001));
    }

    #[test]
    fn reset_clears_state() {
        let pid = PID {
            p: 1.0,
            i: 1.0,
            d: 0.0,
        };
        let mut controller = PIDController::new(pid, 100.0, None);

        controller.update(5.0, 0.1);
        controller.reset();

        assert!(approx_eq(controller.integral, 0.0, 0.001));
        assert!(approx_eq(controller.last_error, 0.0, 0.001));
        assert!(approx_eq(controller.last_output, 0.0, 0.001));
    }

    #[test]
    fn rate_limiting_constrains_output_change() {
        let pid = PID {
            p: 100.0,
            i: 0.0,
            d: 0.0,
        };
        let mut controller = PIDController::new(pid, 100.0, Some(10.0));

        let output = controller.update(1.0, 0.1);

        // max_rate = 10.0, dt = 0.1, so max change = 1.0 from 0.0
        assert!(approx_eq(output, 1.0, 0.001));
    }

    #[test]
    fn set_max_output_clamps_existing_values() {
        let pid = PID {
            p: 1.0,
            i: 1.0,
            d: 0.0,
        };
        let mut controller = PIDController::new(pid, 100.0, None);

        controller.update(50.0, 0.1);
        controller.set_max_output(10.0);

        assert!(controller.last_output <= 10.0);
        assert!(controller.integral <= 10.0);
    }

    #[test]
    fn integral_does_not_wind_up_further_into_positive_saturation() {
        let pid = PID {
            p: 3.0,
            i: 10.0,
            d: 0.0,
        };
        let mut controller = PIDController::new(pid, 5.0, None);
        controller.integral = 2.0;
        controller.last_error = 1.0;

        let output = controller.update(1.0, 0.1);

        assert!(approx_eq(output, 5.0, 0.001));
        assert!(approx_eq(controller.integral, 2.0, 0.001));
    }

    #[test]
    fn integral_unwinds_when_error_opposes_positive_saturation() {
        let pid = PID {
            p: 1.0,
            i: 10.0,
            d: 0.0,
        };
        let mut controller = PIDController::new(pid, 5.0, None);
        controller.integral = 5.0;
        controller.last_error = 1.0;

        let output = controller.update(-1.0, 0.1);

        assert!(approx_eq(output, 4.0, 0.001));
        assert!(approx_eq(controller.integral, 5.0, 0.001));

        let output = controller.update(-1.0, 0.1);

        assert!(approx_eq(output, 3.0, 0.001));
        assert!(approx_eq(controller.integral, 4.0, 0.001));
    }

    #[test]
    fn zero_dt_skips_rate_limiting_safely() {
        let pid = PID {
            p: 100.0,
            i: 0.0,
            d: 0.0,
        };
        let mut controller = PIDController::new(pid, 100.0, Some(10.0));

        let output = controller.update(1.0, 0.0);

        assert!(approx_eq(output, 100.0, 0.001));
        assert!(approx_eq(controller.last_output, 100.0, 0.001));
    }
}
