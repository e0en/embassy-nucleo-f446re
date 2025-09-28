use core::ops::Sub;

#[derive(Clone, Copy)]
pub struct PhaseCurrent {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

impl PhaseCurrent {
    pub fn new(a: f32, b: f32, c: f32) -> Self {
        Self { a, b, c }
    }
}

impl Sub<PhaseCurrent> for PhaseCurrent {
    type Output = Self;
    fn sub(self, c: PhaseCurrent) -> Self::Output {
        PhaseCurrent::new(self.a - c.a, self.b - c.b, self.c - c.c)
    }
}
