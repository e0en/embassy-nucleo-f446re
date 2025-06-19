use crate::controller::DutyCycle3Phase;

pub trait Motor {
    fn run(signal: DutyCycle3Phase);
}
