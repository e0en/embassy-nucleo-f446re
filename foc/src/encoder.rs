pub struct EncoderReading {
    pub phase: f32,
    pub full_rotations: i32,
    pub cumulative_angle: f32,
    pub velocity: f32,
    pub dt: f32,
}

pub trait AngleInput {
    type ReadError;

    fn read_async(&mut self) -> impl Future<Output = Result<EncoderReading, Self::ReadError>>;
}
