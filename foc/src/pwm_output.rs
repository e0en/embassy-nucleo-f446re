#[derive(Debug, Clone, Copy)]
pub struct DutyCycle3Phase {
    pub t1: f32,
    pub t2: f32,
    pub t3: f32,
}

impl DutyCycle3Phase {
    pub fn zero() -> Self {
        Self {
            t1: 0.0,
            t2: 0.0,
            t3: 0.0,
        }
    }
}

pub struct Motor {
    pub pole_pairs: u8,
    pub max_current: f32, // Amperes
    pub max_voltage: f32, // Volts
    pub max_power: f32,   // Watts
    pub max_rpm: u16,
}

impl DutyCycle3Phase {
    pub fn new(t1: f32, t2: f32, t3: f32) -> Self {
        Self { t1, t2, t3 }
    }
}

pub trait PwmOutput {
    fn run(&mut self, signal: DutyCycle3Phase);
    fn stop(&mut self) {
        self.run(DutyCycle3Phase::new(0.0, 0.0, 0.0));
    }
}
