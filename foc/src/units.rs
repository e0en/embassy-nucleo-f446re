use core::ops::{Add, Div, Mul, Sub};

use libm::fmodf;

#[derive(Clone, Copy)]
pub struct Radian(pub f32);

#[derive(Clone, Copy)]
pub struct RadianPerSecond(pub f32);

#[derive(Clone, Copy)]
pub struct Second(pub f32);

impl Radian {
    pub fn normalize(&self) -> Self {
        Radian(fmodf(self.0, 2.0 * core::f32::consts::PI))
    }
}

impl Add<Self> for Radian {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Radian(self.0 + rhs.0)
    }
}

impl Sub<Self> for Radian {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Radian(self.0 - rhs.0)
    }
}

impl Add<f32> for Radian {
    type Output = Self;
    fn add(self, rhs: f32) -> Self::Output {
        Self(self.0 + rhs)
    }
}

impl Sub<f32> for Radian {
    type Output = Self;
    fn sub(self, rhs: f32) -> Self::Output {
        Self(self.0 - rhs)
    }
}

impl Mul<f32> for Radian {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self::Output {
        Self(self.0 * rhs)
    }
}

impl Div<f32> for Radian {
    type Output = Self;
    fn div(self, rhs: f32) -> Self::Output {
        Self(self.0 / rhs)
    }
}

impl Div<Second> for Radian {
    type Output = RadianPerSecond;
    fn div(self, rhs: Second) -> Self::Output {
        RadianPerSecond(self.0 / rhs.0)
    }
}

impl Add<Self> for RadianPerSecond {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub<Self> for RadianPerSecond {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl Add<f32> for RadianPerSecond {
    type Output = Self;
    fn add(self, rhs: f32) -> Self::Output {
        Self(self.0 + rhs)
    }
}

impl Sub<f32> for RadianPerSecond {
    type Output = Self;
    fn sub(self, rhs: f32) -> Self::Output {
        RadianPerSecond(self.0 - rhs)
    }
}

impl Mul<f32> for RadianPerSecond {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self::Output {
        RadianPerSecond(self.0 * rhs)
    }
}

impl Div<f32> for RadianPerSecond {
    type Output = Self;
    fn div(self, rhs: f32) -> Self::Output {
        Self(self.0 / rhs)
    }
}

impl Mul<Second> for RadianPerSecond {
    type Output = Radian;
    fn mul(self, rhs: Second) -> Self::Output {
        Radian(self.0 * rhs.0)
    }
}
