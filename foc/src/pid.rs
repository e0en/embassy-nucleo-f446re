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

    max_integral: f32,
}

impl PIDController {
    pub fn new(pid: PID, max_output: f32, max_rate: f32) -> Self {
        let max_integral = match pid.i {
            i if i <= 0.0 => max_output,
            i => max_output / i,
        };
        PIDController {
            gains: pid,
            max_output,
            max_rate,
            integral: 0.0,
            last_error: 0.0,
            last_output: 0.0,
            max_integral,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = 0.0;
        self.last_output = 0.0;
    }

    pub fn set_max_output(&mut self, max_output: f32) {
        self.max_integral = match self.gains.i {
            i if i <= 0.0 => max_output,
            i => max_output / i,
        };
        self.max_output = max_output;
        self.last_output = self.last_output.min(max_output);
        self.integral = self.integral.min(self.max_integral);
    }

    pub fn update(&mut self, error: f32, dt_seconds: f32) -> f32 {
        self.integral += 0.5 * (error + self.last_error) * dt_seconds;
        self.integral = self.integral.min(self.max_integral).max(-self.max_integral);

        let d_term = if dt_seconds > 0.0 {
            (error - self.last_error) / dt_seconds
        } else {
            0.0
        };

        let mut output =
            self.gains.p * error + self.gains.i * self.integral + self.gains.d * d_term;
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
