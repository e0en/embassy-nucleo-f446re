#[allow(dead_code)]
pub const SETUP: foc::controller::MotorSetup = foc::controller::MotorSetup {
    pole_pair_count: 7,
    phase_resistance: 9.0,
    max_velocity: 2180.0 * (2.0 * core::f32::consts::PI) / 60.0,
};

#[allow(dead_code)]
pub const CURRENT_PID: foc::pid::PID = foc::pid::PID {
    p: 0.0,
    i: 0.0,
    d: 0.0,
};

#[allow(dead_code)]
pub const ANGLE_PID_CS: foc::pid::PID = foc::pid::PID {
    p: 0.0,
    i: 0.0,
    d: 0.0,
};

#[allow(dead_code)]
pub const VELOCITY_PID_CS: foc::pid::PID = foc::pid::PID {
    p: 0.0,
    i: 0.0,
    d: 0.0,
};

#[allow(dead_code)]
pub const ANGLE_PID_NOCS: foc::pid::PID = foc::pid::PID {
    p: 4.0,
    i: 0.0,
    d: 0.0,
};

#[allow(dead_code)]
pub const VELOCITY_PID_NOCS: foc::pid::PID = foc::pid::PID {
    p: 0.4,
    i: 1.6,
    d: 0.0,
};
