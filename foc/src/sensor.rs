use crate::units::{Radian, RadianPerSecond};

pub trait Sensor {
    fn read() -> (Radian, RadianPerSecond, f32);
}
