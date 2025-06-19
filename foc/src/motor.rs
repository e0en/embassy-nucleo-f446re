#[derive(Debug)]
pub struct DutyCycle3Phase {
    pub t1: f32,
    pub t2: f32,
    pub t3: f32,
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

pub trait Motor {
    fn run(signal: DutyCycle3Phase);
}
