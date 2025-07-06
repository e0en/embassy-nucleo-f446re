use crate::units::Second;

pub struct PID {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

pub struct PIDController {
    pub gains: PID,

    pub max_output: f32,
    pub max_rate: f32,

    pub integral: f32,
    pub last_error: f32,
    pub last_output: f32,
}

impl PIDController {
    pub fn new(pid: PID, max_output: f32, max_rate: f32) -> Self {
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

    pub fn update(&mut self, error: f32, dt: Second) -> f32 {
        let seconds = dt.0;

        self.integral += 0.5 * (error + self.last_error) * seconds;
        self.integral = self.integral.min(self.max_output).max(-self.max_output);

        let d_term = if seconds > 0.0 {
            (error - self.last_error) / seconds
        } else {
            0.0
        };

        let mut output =
            self.gains.p * error + self.gains.i * self.integral + self.gains.d * d_term;
        output = output.min(self.max_output).max(-self.max_output);

        let output_rate = (output - self.last_output) / seconds;
        if output_rate > self.max_rate {
            output = self.last_output + self.max_rate * seconds;
        } else if output_rate < -self.max_rate {
            output = self.last_output - self.max_rate * seconds;
        }

        self.last_error = error;
        self.last_output = output;
        output
    }
}
