use core::f32::consts::PI;

const TAU: f32 = 2.0 * PI;

pub struct OutputTracker {
    previous_phase: Option<f32>,
    output_angle: f32,
}

impl OutputTracker {
    pub const fn new() -> Self {
        Self {
            previous_phase: None,
            output_angle: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.previous_phase = None;
        self.output_angle = 0.0;
    }

    pub fn update(&mut self, wrapped_output_phase: f32) -> f32 {
        match self.previous_phase {
            None => {
                self.previous_phase = Some(wrapped_output_phase);
                self.output_angle = wrapped_output_phase;
            }
            Some(previous_phase) => {
                self.output_angle += wrap_pm_pi(wrapped_output_phase - previous_phase);
                self.previous_phase = Some(wrapped_output_phase);
            }
        }

        self.output_angle
    }
}

fn wrap_pm_pi(angle: f32) -> f32 {
    let mut wrapped = angle % TAU;
    if wrapped <= -PI {
        wrapped += TAU;
    } else if wrapped > PI {
        wrapped -= TAU;
    }
    wrapped
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 1e-6;

    #[test]
    fn stays_continuous_across_wrap() {
        let mut tracker = OutputTracker::new();

        let first = tracker.update(TAU - 0.1);
        let second = tracker.update(0.05);

        assert!((first - (TAU - 0.1)).abs() < EPSILON);
        assert!((second - (TAU + 0.05)).abs() < EPSILON);
    }

    #[test]
    fn preserves_multi_turn_continuity() {
        let mut tracker = OutputTracker::new();

        let samples = [0.2, 1.0, 2.4, 3.8, 5.2, 0.15, 1.1, 2.2, 3.0];
        let mut last = 0.0;
        for sample in samples {
            last = tracker.update(sample);
        }

        let expected = TAU + 3.0;
        assert!((last - expected).abs() < EPSILON);
    }

    #[test]
    fn reset_restarts_continuity_from_next_sample() {
        let mut tracker = OutputTracker::new();

        let _ = tracker.update(TAU - 0.1);
        let _ = tracker.update(0.05);
        tracker.reset();
        let reset_angle = tracker.update(0.3);

        assert!((reset_angle - 0.3).abs() < EPSILON);
    }
}
