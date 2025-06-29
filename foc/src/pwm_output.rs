#[derive(Debug, Clone, Copy)]
pub struct DutyCycle3Phase {
    pub t1: f32,
    pub t2: f32,
    pub t3: f32,
}

pub struct Motor {
    pub pole_pairs: u8,
    pub max_current: f32, // Amperes
    pub max_voltage: f32, // Volts
    pub max_power: f32,   // Watts
    pub max_rpm: u16,
}

impl DutyCycle3Phase {
    pub fn new(phases: (f32, f32, f32)) -> Self {
        Self {
            t1: phases.0,
            t2: phases.1,
            t3: phases.2,
        }
    }
}

pub trait PwmOutput {
    fn run(&mut self, signal: DutyCycle3Phase);
}
