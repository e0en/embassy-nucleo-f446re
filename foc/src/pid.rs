#[derive(Copy, Clone)]
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

    pub fn set_max_output(&mut self, max_output: f32) {
        self.max_output = max_output;
        self.last_output = self.last_output.min(max_output).max(-self.max_output);
        self.integral = self.integral.min(self.max_output).max(-self.max_output);
    }

    pub fn update(&mut self, error: f32, dt_seconds: f32) -> f32 {
        self.integral += 0.5 * self.gains.i * (error + self.last_error) * dt_seconds;
        self.integral = self.integral.min(self.max_output).max(-self.max_output);

        let d_term = if dt_seconds > 0.0 {
            self.gains.d * (error - self.last_error) / dt_seconds
        } else {
            0.0
        };

        let mut output = self.gains.p * error + self.integral + d_term;
        output = output.min(self.max_output).max(-self.max_output);

        let output_rate = (output - self.last_output) / dt_seconds;
        if output_rate > self.max_rate {
            output = self.last_output + self.max_rate * dt_seconds;
        } else if output_rate < -self.max_rate {
            output = self.last_output - self.max_rate * dt_seconds;
        }

        self.last_error = error;
        self.last_output = output;
        output
    }
}
