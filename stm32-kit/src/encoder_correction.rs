use core::cell::RefCell;
use core::f32::consts::{PI, TAU};
use core::sync::atomic::{AtomicU32, Ordering};

use critical_section::Mutex;
use embassy_time::Instant;
use foc::angle_input::AngleReading;
use foc::tracking_observer::TrackingObserver;

use crate::cordic::atan2;

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
    cumulative_angle: f32,
    observer_angle: f32,
    previous_time: Instant,
    observer: TrackingObserver,
}

impl AngleTracker {
    pub fn new(observer_bandwidth_hz: f32) -> Self {
        Self {
            previous_wrapped_angle: None,
            cumulative_angle: 0.0,
            observer_angle: 0.0,
            previous_time: Instant::from_secs(0),
            observer: TrackingObserver::new(observer_bandwidth_hz),
        }
    }

    pub fn update(&mut self, wrapped_angle: f32, now: Instant) -> AngleReading {
        let wrapped_angle = wrap_0_tau(wrapped_angle);
        match self.previous_wrapped_angle {
            None => {
                self.previous_wrapped_angle = Some(wrapped_angle);
                self.previous_time = now;
                self.cumulative_angle = wrapped_angle;
                self.observer_angle = wrapped_angle;
                AngleReading {
                    angle: self.cumulative_angle,
                    phase_angle: wrapped_angle,
                    velocity: 0.0,
                    dt: 0.0,
                }
            }
            Some(previous_wrapped_angle) => {
                let dt_duration = now.checked_duration_since(self.previous_time);
                let dt = dt_duration
                    .map(|x| x.as_micros() as f32 / 1e6)
                    .unwrap_or(0.0);
                let delta = wrap_pm_pi(wrapped_angle - previous_wrapped_angle);
                self.cumulative_angle += delta;
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
                AngleReading {
                    angle: self.cumulative_angle,
                    phase_angle: wrapped_angle,
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

pub fn circular_mean(a: f32, b: f32) -> f32 {
    wrap_0_tau(a + 0.5 * wrap_pm_pi(b - a))
}

pub fn average_wrapped_samples(sum_sin: f32, sum_cos: f32) -> Option<f32> {
    if sum_sin == 0.0 && sum_cos == 0.0 {
        None
    } else {
        Some(wrap_0_tau(atan2(sum_sin, sum_cos)))
    }
}

pub fn build_lut_from_samples(samples: &[(f32, f32)]) -> Option<EncoderCorrection> {
    if samples.is_empty() {
        return None;
    }

    let mut sum = [0.0; LUT_SIZE];
    let mut count = [0u16; LUT_SIZE];

    for &(measured, reference) in samples {
        let wrapped_measured = wrap_0_tau(measured);
        let error = wrap_pm_pi(reference - wrapped_measured);
        let index = ((wrapped_measured * (LUT_SIZE as f32) / TAU) as usize) % LUT_SIZE;
        sum[index] += error;
        count[index] += 1;
    }

    let mut lut = [0.0; LUT_SIZE];
    for i in 0..LUT_SIZE {
        if count[i] > 0 {
            lut[i] = sum[i] / (count[i] as f32);
        }
    }

    fill_missing_bins(&mut lut, &count)?;

    Some(EncoderCorrection {
        valid: true,
        error_lut: lut,
    })
}

fn fill_missing_bins(lut: &mut [f32; LUT_SIZE], count: &[u16; LUT_SIZE]) -> Option<()> {
    let mut filled = 0usize;
    for &c in count {
        if c > 0 {
            filled += 1;
        }
    }
    if filled < 4 {
        return None;
    }

    for i in 0..LUT_SIZE {
        if count[i] > 0 {
            continue;
        }

        let prev = find_filled_bin(count, i, -1)?;
        let next = find_filled_bin(count, i, 1)?;

        let prev_value = lut[prev];
        let next_value = lut[next];
        let span = if next >= prev {
            next - prev
        } else {
            LUT_SIZE + next - prev
        };
        let offset = if i >= prev {
            i - prev
        } else {
            LUT_SIZE + i - prev
        };
        let frac = offset as f32 / span as f32;
        lut[i] = prev_value + frac * wrap_pm_pi(next_value - prev_value);
    }
    Some(())
}

fn find_filled_bin(count: &[u16; LUT_SIZE], start: usize, direction: isize) -> Option<usize> {
    for offset in 1..=LUT_SIZE {
        let index = if direction > 0 {
            (start + offset) % LUT_SIZE
        } else {
            (start + LUT_SIZE - (offset % LUT_SIZE)) % LUT_SIZE
        };
        if count[index] > 0 {
            return Some(index);
        }
    }
    None
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
    fn builds_lut_from_sparse_samples() {
        let samples = [
            (0.0, 0.05),
            (TAU * 0.25, TAU * 0.25 + 0.05),
            (TAU * 0.5, TAU * 0.5 + 0.05),
            (TAU * 0.75, TAU * 0.75 + 0.05),
        ];
        let correction = build_lut_from_samples(&samples).unwrap();
        assert!(correction.valid);
        let corrected = correction.correct_wrapped_angle(0.0);
        assert!((corrected - 0.05).abs() < 0.02);
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
}
