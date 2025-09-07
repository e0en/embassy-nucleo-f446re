use core::ops::{Add, AddAssign, Div, Mul, Sub, SubAssign};

use libm::{cosf, fmodf, sinf};

#[derive(Clone, Copy, PartialEq)]
pub struct Radian {
    pub angle: f32,
    pub sin_cos: Option<(f32, f32)>,
}

#[derive(Clone, Copy)]
pub struct RadianPerSecond(pub f32);

#[derive(Clone, Copy)]
pub struct Second(pub f32);

impl Radian {
    pub fn new(angle: f32) -> Self {
        Self {
            angle,
            sin_cos: None,
        }
    }
    pub fn get_sin_cos(&mut self) -> (f32, f32) {
        match self.sin_cos {
            None => {
                let (s, c) = (sinf(self.angle), cosf(self.angle));
                self.sin_cos = Some((s, c));
                (s, c)
            }
            Some((s, c)) => (s, c),
        }
    }
    pub fn normalize(&self) -> Self {
        Radian::new(fmodf(self.angle, 2.0 * core::f32::consts::PI))
    }
}

impl Add<Self> for Radian {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Radian::new(self.angle + rhs.angle)
    }
}

impl AddAssign<Self> for Radian {
    fn add_assign(&mut self, rhs: Self) {
        self.angle += rhs.angle;
        self.sin_cos = None;
    }
}

impl Sub<Self> for Radian {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Radian::new(self.angle - rhs.angle)
    }
}

impl SubAssign<Self> for Radian {
    fn sub_assign(&mut self, rhs: Self) {
        self.angle -= rhs.angle;
        self.sin_cos = None;
    }
}

impl Add<f32> for Radian {
    type Output = Self;
    fn add(self, rhs: f32) -> Self::Output {
        Self::new(self.angle + rhs)
    }
}

impl AddAssign<f32> for Radian {
    fn add_assign(&mut self, rhs: f32) {
        self.angle += rhs;
        self.sin_cos = None;
    }
}

impl Sub<f32> for Radian {
    type Output = Self;
    fn sub(self, rhs: f32) -> Self::Output {
        Self::new(self.angle - rhs)
    }
}

impl SubAssign<f32> for Radian {
    fn sub_assign(&mut self, rhs: f32) {
        self.angle -= rhs;
        self.sin_cos = None;
    }
}

impl Mul<f32> for Radian {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self::Output {
        Self::new(self.angle * rhs)
    }
}

impl Div<f32> for Radian {
    type Output = Self;
    fn div(self, rhs: f32) -> Self::Output {
        Self::new(self.angle / rhs)
    }
}

impl Div<Second> for Radian {
    type Output = RadianPerSecond;
    fn div(self, rhs: Second) -> Self::Output {
        RadianPerSecond(self.angle / rhs.0)
    }
}

impl Add<Self> for RadianPerSecond {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl AddAssign<Self> for RadianPerSecond {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
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
        Radian::new(self.0 * rhs.0)
    }
}
