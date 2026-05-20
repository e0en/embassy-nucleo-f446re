use core::f32::consts::TAU;

use foc::angle_input::RawSensorReading;

#[path = "../src/gear_sensor.rs"]
mod gear_sensor;

const EPSILON: f32 = 1e-5;

fn wrap_0_tau(angle: f32) -> f32 {
    let wrapped = angle % TAU;
    if wrapped < 0.0 {
        wrapped + TAU
    } else {
        wrapped
    }
}

fn primary_phase(output_phase: f32, reduction_ratio: i32) -> f32 {
    wrap_0_tau(-output_phase * reduction_ratio as f32)
}

fn secondary_phase(output_phase: f32, reduction_ratio: i32) -> f32 {
    wrap_0_tau(output_phase * (reduction_ratio as f32 + 1.0))
}

#[test]
fn returns_none_when_alignment_is_invalid() {
    let raw = RawSensorReading {
        primary_phase: 1.0,
        secondary_phase: 2.0,
        rotor_velocity: 3.0,
        dt: 0.001,
    };

    assert!(gear_sensor::resolve_sensor_reading(&raw, false, -19).is_none());
}

#[test]
fn resolves_output_phase_when_alignment_is_valid() {
    let output_phase = 1.234;
    let raw = RawSensorReading {
        primary_phase: primary_phase(output_phase, -19),
        secondary_phase: secondary_phase(output_phase, -19),
        rotor_velocity: 4.5,
        dt: 0.001,
    };

    let reading = gear_sensor::resolve_sensor_reading(&raw, true, -19).unwrap();

    assert!((reading.output_phase - output_phase).abs() < EPSILON);
    assert!((reading.rotor_phase - raw.primary_phase).abs() < EPSILON);
    assert!((reading.rotor_velocity - raw.rotor_velocity).abs() < EPSILON);
    assert!((reading.dt - raw.dt).abs() < EPSILON);
}

#[test]
fn preserves_signed_reduction_ratio_direction() {
    let output_phase = 2.468;
    let raw = RawSensorReading {
        primary_phase: primary_phase(output_phase, -19),
        secondary_phase: secondary_phase(output_phase, -19),
        rotor_velocity: -1.25,
        dt: 0.002,
    };

    let reading = gear_sensor::resolve_sensor_reading(&raw, true, -19).unwrap();

    assert!((reading.output_phase - output_phase).abs() < EPSILON);
}
