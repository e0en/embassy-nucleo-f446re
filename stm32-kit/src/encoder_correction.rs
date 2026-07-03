use core::cell::RefCell;
use core::f32::consts::{PI, TAU};
use core::sync::atomic::{AtomicU32, Ordering};

use critical_section::Mutex;
use embassy_time::Instant;
use foc::encoder::EncoderReading;
use foc::tracking_observer::TrackingObserver;

pub const LUT_SIZE: usize = 128;

static CORRECTION_VERSION: AtomicU32 = AtomicU32::new(1);
static RUNTIME_CORRECTION: Mutex<RefCell<EncoderCorrection>> =
    Mutex::new(RefCell::new(EncoderCorrection::identity()));

#[derive(Clone, Copy)]
pub struct EncoderCorrection {
    pub valid: bool,
    pub error_lut: [f32; LUT_SIZE],
}

impl EncoderCorrection {
    pub const fn identity() -> Self {
        Self {
            valid: false,
            error_lut: [0.0; LUT_SIZE],
        }
    }

    pub fn correct_wrapped_angle(&self, angle: f32) -> f32 {
        if !self.valid {
            return wrap_0_tau(angle);
        }

        let wrapped = wrap_0_tau(angle);
        let position = wrapped * (LUT_SIZE as f32) / TAU;
        let i0 = position as usize % LUT_SIZE;
        let i1 = (i0 + 1) % LUT_SIZE;
        let frac = position - i0 as f32;
        let error = self.error_lut[i0] + frac * wrap_pm_pi(self.error_lut[i1] - self.error_lut[i0]);
        wrap_0_tau(wrapped + error)
    }
}

pub fn runtime_version() -> u32 {
    CORRECTION_VERSION.load(Ordering::Relaxed)
}

pub fn runtime_snapshot() -> EncoderCorrection {
    critical_section::with(|cs| *RUNTIME_CORRECTION.borrow(cs).borrow())
}

pub fn set_runtime_correction(correction: EncoderCorrection) {
    critical_section::with(|cs| {
        *RUNTIME_CORRECTION.borrow(cs).borrow_mut() = correction;
    });
    CORRECTION_VERSION.fetch_add(1, Ordering::Relaxed);
}

pub struct AngleTracker {
    previous_wrapped_angle: Option<f32>,
    full_turns: i32,
    residual_angle: f32,
    observer_angle: f32,
    previous_time: Instant,
    observer: TrackingObserver,
}

impl AngleTracker {
    pub fn new(observer_bandwidth_hz: f32) -> Self {
        Self {
            previous_wrapped_angle: None,
            full_turns: 0,
            residual_angle: 0.0,
            observer_angle: 0.0,
            previous_time: Instant::from_secs(0),
            observer: TrackingObserver::new(observer_bandwidth_hz),
        }
    }

    fn cumulative_angle(&self) -> f32 {
        self.full_turns as f32 * TAU + self.residual_angle
    }

    pub fn is_uninitialized(&self) -> bool {
        self.previous_wrapped_angle.is_none()
    }

    pub fn seed(&mut self, wrapped_angle: f32, full_turns: i32, now: Instant) -> EncoderReading {
        let wrapped_angle = wrap_0_tau(wrapped_angle);
        self.previous_wrapped_angle = Some(wrapped_angle);
        self.previous_time = now;
        self.full_turns = full_turns;
        self.residual_angle = wrapped_angle;
        self.observer_angle = wrapped_angle;
        let _ = self.observer.update(self.observer_angle, 0.0);

        EncoderReading {
            phase: wrapped_angle,
            full_rotations: self.full_turns,
            cumulative_angle: self.cumulative_angle(),
            velocity: 0.0,
            dt: 0.0,
        }
    }

    pub fn update(&mut self, wrapped_angle: f32, now: Instant) -> EncoderReading {
        let wrapped_angle = wrap_0_tau(wrapped_angle);
        match self.previous_wrapped_angle {
            None => self.seed(wrapped_angle, 0, now),
            Some(previous_wrapped_angle) => {
                let dt_duration = now.checked_duration_since(self.previous_time);
                let dt = dt_duration
                    .map(|x| x.as_micros() as f32 / 1e6)
                    .unwrap_or(0.0);
                let delta = wrap_pm_pi(wrapped_angle - previous_wrapped_angle);
                self.residual_angle += delta;
                while self.residual_angle >= TAU {
                    self.residual_angle -= TAU;
                    self.full_turns += 1;
                }
                while self.residual_angle < 0.0 {
                    self.residual_angle += TAU;
                    self.full_turns -= 1;
                }
                self.observer_angle += delta;
                if self.observer_angle > PI {
                    self.observer_angle -= TAU;
                    self.observer.angle -= TAU;
                } else if self.observer_angle < -PI {
                    self.observer_angle += TAU;
                    self.observer.angle += TAU;
                }

                let velocity = self.observer.update(self.observer_angle, dt);
                self.previous_wrapped_angle = Some(wrapped_angle);
                self.previous_time = now;
                EncoderReading {
                    phase: wrapped_angle,
                    full_rotations: self.full_turns,
                    cumulative_angle: self.cumulative_angle(),
                    velocity,
                    dt,
                }
            }
        }
    }
}

pub fn wrap_0_tau(angle: f32) -> f32 {
    let mut wrapped = angle % TAU;
    if wrapped < 0.0 {
        wrapped += TAU;
    }
    wrapped
}

pub fn wrap_pm_pi(angle: f32) -> f32 {
    let wrapped = wrap_0_tau(angle + PI);
    wrapped - PI
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_correction_is_noop() {
        let correction = EncoderCorrection::identity();
        let angle = 5.9;
        assert!((correction.correct_wrapped_angle(angle) - angle).abs() < 1e-6);
    }

    #[test]
    fn lut_correction_interpolates_periodically() {
        let mut lut = [0.0; LUT_SIZE];
        lut[0] = 0.1;
        lut[1] = 0.2;
        let correction = EncoderCorrection {
            valid: true,
            error_lut: lut,
        };

        let corrected = correction.correct_wrapped_angle(TAU * 0.5 / LUT_SIZE as f32);
        assert!(corrected > 0.1);
    }

    #[test]
    fn angle_tracker_velocity_stays_stable_across_many_turns() {
        let mut tracker = AngleTracker::new(20.0);
        let dt = 0.001;
        let velocity = -80.0;
        let mut now = Instant::from_micros(0);
        let mut wrapped_angle = 0.0;
        let mut last_velocity = 0.0;

        let _ = tracker.update(wrapped_angle, now);

        for _ in 0..120_000 {
            now += embassy_time::Duration::from_micros(1_000);
            wrapped_angle = wrap_0_tau(wrapped_angle + velocity * dt);
            last_velocity = tracker.update(wrapped_angle, now).velocity;
        }

        assert!(
            (last_velocity - velocity).abs() < 0.5,
            "velocity drifted: measured={last_velocity}, expected={velocity}"
        );
    }

    #[test]
    fn angle_tracker_preserves_position_across_many_turns() {
        let mut tracker = AngleTracker::new(20.0);
        let dt = 0.001f32;
        let velocity = 80.0f32;
        let mut now = Instant::from_micros(0);
        let mut wrapped_angle = 0.0f32;
        let mut last_angle = 0.0f32;
        let mut expected_angle = 0.0f64;

        let _ = tracker.update(wrapped_angle, now);

        for _ in 0..120_000 {
            now += embassy_time::Duration::from_micros(1_000);
            wrapped_angle = wrap_0_tau(wrapped_angle + velocity * dt);
            last_angle = tracker.update(wrapped_angle, now).cumulative_angle;
            expected_angle += velocity as f64 * dt as f64;
        }

        assert!(
            (last_angle as f64 - expected_angle).abs() < 0.05,
            "angle drifted: measured={last_angle}, expected={expected_angle}"
        );
    }

    #[test]
    fn angle_tracker_seed_restores_startup_turn_count() {
        let mut tracker = AngleTracker::new(20.0);
        let now = Instant::from_micros(0);
        let wrapped_angle = PI;

        let seeded = tracker.seed(wrapped_angle, 9, now);

        assert_eq!(seeded.full_rotations, 9);
        assert!((seeded.cumulative_angle - (9.0 * TAU + wrapped_angle)).abs() < 1e-6);
        assert_eq!(seeded.velocity, 0.0);
        assert_eq!(seeded.dt, 0.0);
    }
}
