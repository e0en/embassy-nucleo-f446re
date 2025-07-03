use crate::units::Second;

pub struct LowPassFilter {
    time_constant: f32,
    last_value: Option<f32>,
}

impl LowPassFilter {
    pub fn new(time_constant: f32) -> Self {
        Self {
            time_constant,
            last_value: None,
        }
    }

    pub fn apply(&mut self, value: f32, dt: Second) -> f32 {
        if dt.0 > 0.3 {
            self.last_value = Some(value);
            value
        } else {
            let new_value = match self.last_value {
                None => value,
                Some(v) => {
                    let alpha = self.time_constant / (self.time_constant + dt.0);
                    alpha * v + (1.0 - alpha) * value
                }
            };
            self.last_value = Some(new_value);
            new_value
        }
    }
}

#[cfg(test)]
mod test {
    use crate::units::Second;

    use super::LowPassFilter;

    #[test]
    fn test_first_value() {
        let mut f = LowPassFilter::new(0.1);
        let value = 1.0;
        let filtered = f.apply(value, Second(0.1));
        assert_eq!(filtered, value, "{filtered} != {value}");
    }

    #[test]
    fn test_long_update() {
        let mut f = LowPassFilter::new(0.1);
        let first_value = 1.0;
        let _ = f.apply(first_value, Second(0.0));
        let second_value = 2.0;
        let filtered = f.apply(second_value, Second(0.5));
        assert_eq!(filtered, second_value, "{filtered} != {second_value}");
    }

    #[test]
    fn test_decay() {
        let mut f = LowPassFilter::new(0.1);
        let first_value = 0.0;
        let _ = f.apply(first_value, Second(0.0));
        let second_value = 1.0;
        let filtered = f.apply(second_value, Second(0.1));
        let expected = second_value / 2.0;
        assert_eq!(filtered, expected, "{filtered} != {expected}");
    }
}
