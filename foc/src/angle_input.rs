pub struct RawSensorReading {
    pub primary_phase: f32,
    pub secondary_phase: f32,
    pub rotor_velocity: f32,
    pub dt: f32,
}

pub struct SensorReading {
    pub output_phase: f32,
    pub rotor_phase: f32,
    pub rotor_velocity: f32,
    pub dt: f32,
}

pub struct ControlReading {
    pub output_angle: f32,
    pub output_velocity: f32,
    pub rotor_phase: f32,
    pub dt: f32,
}

pub trait AngleInput {
    type ReadError;

    fn read_async(&mut self) -> impl Future<Output = Result<SensorReading, Self::ReadError>>;
}
