use crate::units::{Radian, RadianPerSecond, Second};

pub struct SensorReading {
    pub angle: Radian,
    pub velocity: RadianPerSecond,
    pub dt: Second,
}

pub trait Sensor {
    type Bus;
    fn read_async(&mut self, comm: &mut Self::Bus) -> impl Future<Output = SensorReading> + Send;
}
