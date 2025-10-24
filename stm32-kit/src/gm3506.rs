pub const SETUP: foc::controller::MotorSetup = foc::controller::MotorSetup {
    pole_pair_count: 11,
    phase_resistance: 9.0,
    max_velocity: 2375.0 * (2.0 * core::f32::consts::PI) / 60.0,
};

pub const CURRENT_PID: foc::pid::PID = foc::pid::PID {
    p: 0.5,
    i: 1000.0,
    d: 0.0,
};

pub const ANGLE_PID_CS: foc::pid::PID = foc::pid::PID {
    p: 2.0,
    i: 0.0,
    d: 0.0,
};

pub const VELOCITY_PID_CS: foc::pid::PID = foc::pid::PID {
    p: 0.04,
    i: 0.1,
    d: 0.0,
};

pub const ANGLE_PID_NOCS: foc::pid::PID = foc::pid::PID {
    p: 16.0,
    i: 0.0,
    d: 0.0,
};

pub const VELOCITY_PID_NOCS: foc::pid::PID = foc::pid::PID {
    p: 0.2,
    i: 12.0,
    d: 0.0,
};
