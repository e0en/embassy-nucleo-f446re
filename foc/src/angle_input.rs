use crate::units::{Radian, Second};

pub struct AngleReading {
    pub angle: Radian,
    pub velocity: f32,
    pub dt: Second,
}

pub trait AngleInput {
    type ReadError;

    fn read_async(&mut self) -> impl Future<Output = Result<AngleReading, Self::ReadError>>;
}
