use crate::units::Second;

pub struct PID {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

pub struct PIDController {
    pub gains: PID,
    pub integral: f32,
    pub last_error: f32,
}

impl PIDController {
    pub fn new(pid: PID) -> Self {
        PIDController {
            gains: pid,
            integral: 0.0,
            last_error: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = 0.0;
    }

    pub fn update(&mut self, error: f32, dt: Second) -> f32 {
        let p_term = self.gains.p * error;
        let seconds = dt.0;

        self.integral += self.gains.i * error * seconds;

        let d_term = if seconds > 0.0 {
            self.gains.d * (error - self.last_error) / seconds
        } else {
            0.0
        };
        self.last_error = error;

        p_term + self.integral + d_term
    }
}
