use crate::units::{Radian, RadianPerSecond};

pub trait Sensor {
    type Bus;
    fn read_async(
        &mut self,
        comm: &mut Self::Bus,
    ) -> impl Future<Output = (Radian, RadianPerSecond, f32)> + Send;
}
