#[allow(dead_code)]
pub const SETUP: foc::controller::MotorSetup = foc::controller::MotorSetup {
    pole_pair_count: 11,
    phase_resistance: 5.0,
    kv_rating: 0.0,
    max_velocity: 2375.0 * (2.0 * core::f32::consts::PI) / 60.0,
};

// Measured with a 100mm arm pressing a load cell: torque[Nm] ~= |i_q_ref| * 1.25.
#[allow(dead_code)]
pub const IQ_TO_TORQUE_NM: f32 = 1.25;

#[allow(dead_code)]
pub const CURRENT_PID: foc::pid::PID = foc::pid::PID {
    p: 5.0,
    i: 1000.0,
    d: 0.0,
};

#[allow(dead_code)]
pub const ANGLE_PID: foc::pid::PID = foc::pid::PID {
    p: 6.0,
    i: 0.0,
    d: 0.0,
};

#[allow(dead_code)]
pub const VELOCITY_PID: foc::pid::PID = foc::pid::PID {
    p: 0.02,
    i: 0.2,
    d: 0.0,
};
