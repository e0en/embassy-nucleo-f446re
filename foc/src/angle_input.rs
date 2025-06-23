use crate::units::{Radian, RadianPerSecond, Second};

pub struct AngleReading {
    pub angle: Radian,
    pub velocity: RadianPerSecond,
    pub dt: Second,
}

pub trait AngleInput {
    type Bus;
    type ReadError;

    fn read_async(
        &mut self,
        comm: &mut Self::Bus,
    ) -> impl Future<Output = Result<AngleReading, Self::ReadError>> + Send;
}
