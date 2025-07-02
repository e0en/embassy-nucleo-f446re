use crate::units::Second;

pub struct LowPassFilter {
    time_constant: f32,
    last_value: f32,
}

impl LowPassFilter {
    pub fn new(time_constant: f32) -> Self {
        Self {
            time_constant,
            last_value: 0.0,
        }
    }

    pub fn apply(&mut self, value: f32, dt: Second) -> f32 {
        if dt.0 > 0.3 {
            self.last_value = value;
            value
        } else {
            let alpha = self.time_constant / (self.time_constant + dt.0);
            let new_value = alpha * self.last_value + (1.0 - alpha) * value;
            self.last_value = new_value;
            new_value
        }
    }
}
