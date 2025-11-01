#[allow(dead_code)]
pub const SETUP: foc::controller::MotorSetup = foc::controller::MotorSetup {
    pole_pair_count: 7,
    phase_resistance: 0.5,
    max_velocity: 400.0,
};

#[allow(dead_code)]
pub const CURRENT_PID: foc::pid::PID = foc::pid::PID {
    p: 1.0,
    i: 100.0,
    d: 0.0,
};

#[allow(dead_code)]
pub const ANGLE_PID: foc::pid::PID = foc::pid::PID {
    p: 16.0,
    i: 0.0,
    d: 0.0,
};

#[allow(dead_code)]
pub const VELOCITY_PID: foc::pid::PID = foc::pid::PID {
    p: 0.5,
    i: 0.001,
    d: 0.0,
};
