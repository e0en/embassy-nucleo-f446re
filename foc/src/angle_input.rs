pub struct AngleReading {
    pub angle: f32,
    pub velocity: f32,
    pub dt: f32,
}

pub trait AngleInput {
    type ReadError;

    fn read_async(&mut self) -> impl Future<Output = Result<AngleReading, Self::ReadError>>;
}
