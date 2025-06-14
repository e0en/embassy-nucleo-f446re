pub struct Motor {
    pub pole_pairs: u8,
    pub max_current: f32, // Amperes
    pub max_voltage: f32, // Volts
    pub max_power: f32,   // Watts
    pub max_rpm: u16,
    // TODO: add more parameters as needed
    // pub resistance: f32, // Ohms
}

pub struct PID {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

pub struct FOCParameters {
    pub align_voltage: f32, // Volts
    pub angle_pid: PID,
    pub velocity_pid: PID,
    pub velocity_output_limit: f32, // Volts per second
    pub velocity_time_filter: f32,  // Seconds,
    pub motor: Motor,
}

pub struct FOC {
    pub parameters: FOCParameters,
    pub angle: f32,           // Radians
    pub target_angle: f32,    // Radians
    pub sensor_angle: f32,    // Radians
    pub velocity: f32,        // Radians per second
    pub target_velocity: f32, // Radians per second
    pub sensor_velocity: f32, // Radians per second
    pub current: f32,         // Amperes
}

pub fn set_angle(angle: f32) {
    // set target velocity from angle difference
    // set target current from velocity difference
    // calculate FOC control signals
}
pub fn set_velocity(rpm: u16) {
    // set target current from velocity difference
    // calculate FOC control signals
}
pub fn set_torque(current: f32) {
    // calculate FOC control signals
}
