use core::sync::atomic::{AtomicI32, AtomicU8, AtomicU32, Ordering};

use defmt::error;
use embassy_executor::task;
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, Sender},
};
use embassy_time::{Instant, Timer};
use foc::encoder::EncoderReading;

use crate::{
    as5047p::{self, As5047P},
    dual_encoder::{DualEncoder, DualEncoderReadError},
    encoder_correction::{AngleTracker, runtime_snapshot, runtime_version},
};

use super::VELOCITY_OBSERVER_BANDWIDTH;

struct SensorSnapshotBuffer {
    phase: AtomicU32,
    secondary_phase: AtomicU32,
    full_rotations: AtomicI32,
    cumulative_angle: AtomicU32,
    velocity: AtomicU32,
    dt: AtomicU32,
}

impl SensorSnapshotBuffer {
    const fn new() -> Self {
        Self {
            phase: AtomicU32::new(0),
            secondary_phase: AtomicU32::new(0),
            full_rotations: AtomicI32::new(0),
            cumulative_angle: AtomicU32::new(0),
            velocity: AtomicU32::new(0),
            dt: AtomicU32::new(0),
        }
    }

    fn write(&self, encoder_reading: &EncoderReading, secondary_phase: f32) {
        store_f32(&self.phase, encoder_reading.phase);
        store_f32(&self.secondary_phase, secondary_phase);
        self.full_rotations
            .store(encoder_reading.full_rotations, Ordering::Relaxed);
        store_f32(&self.cumulative_angle, encoder_reading.cumulative_angle);
        store_f32(&self.velocity, encoder_reading.velocity);
        store_f32(&self.dt, encoder_reading.dt);
    }

    fn read_encoder(&self) -> EncoderReading {
        EncoderReading {
            phase: load_f32(&self.phase),
            full_rotations: self.full_rotations.load(Ordering::Relaxed),
            cumulative_angle: load_f32(&self.cumulative_angle),
            velocity: load_f32(&self.velocity),
            dt: load_f32(&self.dt),
        }
    }
}

static SENSOR_SNAPSHOTS: [SensorSnapshotBuffer; 2] =
    [SensorSnapshotBuffer::new(), SensorSnapshotBuffer::new()];
static ACTIVE_SENSOR_SNAPSHOT: AtomicU8 = AtomicU8::new(0);

#[derive(Clone, Copy)]
#[cfg_attr(feature = "main-fw", allow(dead_code))]
enum EncoderTaskCommand {
    SetSecondaryZeroOffset(f32),
}

#[derive(Clone, Copy)]
enum EncoderTaskResponse {
    SecondaryZeroOffsetSet,
}

static ENCODER_TASK_COMMAND_CHANNEL: Channel<ThreadModeRawMutex, EncoderTaskCommand, 1> =
    Channel::new();
static ENCODER_TASK_RESPONSE_CHANNEL: Channel<ThreadModeRawMutex, EncoderTaskResponse, 1> =
    Channel::new();

fn store_f32(slot: &AtomicU32, value: f32) {
    slot.store(u32::from_le_bytes(value.to_le_bytes()), Ordering::Relaxed);
}

fn load_f32(slot: &AtomicU32) -> f32 {
    f32::from_le_bytes(slot.load(Ordering::Relaxed).to_le_bytes())
}

fn active_sensor_snapshot() -> &'static SensorSnapshotBuffer {
    let active_index = ACTIVE_SENSOR_SNAPSHOT.load(Ordering::Acquire) as usize;
    &SENSOR_SNAPSHOTS[active_index]
}

fn write_sensor_snapshot(encoder_reading: &EncoderReading, secondary_phase: f32) {
    let active_index = ACTIVE_SENSOR_SNAPSHOT.load(Ordering::Relaxed) as usize;
    let next_index = active_index ^ 1;
    let snapshot = &SENSOR_SNAPSHOTS[next_index];
    snapshot.write(encoder_reading, secondary_phase);

    // Writer runs in thread mode and reader runs in ADC ISR, so once the
    // index is published the ISR can only observe a fully written snapshot.
    ACTIVE_SENSOR_SNAPSHOT.store(next_index as u8, Ordering::Release);
}

async fn handle_encoder_task_command(
    command: EncoderTaskCommand,
    dual_encoder: &mut DualEncoder<As5047P<'static>, As5047P<'static>>,
    response_sender: &Sender<'static, ThreadModeRawMutex, EncoderTaskResponse, 1>,
) -> bool {
    match command {
        EncoderTaskCommand::SetSecondaryZeroOffset(secondary_zero_offset) => {
            dual_encoder
                .secondary_mut()
                .set_zero_offset(secondary_zero_offset);
            response_sender
                .send(EncoderTaskResponse::SecondaryZeroOffsetSet)
                .await;
            true
        }
    }
}

#[cfg_attr(feature = "main-fw", allow(dead_code))]
pub(crate) async fn apply_secondary_zero_offset(secondary_zero_offset: f32) -> bool {
    ENCODER_TASK_COMMAND_CHANNEL
        .sender()
        .send(EncoderTaskCommand::SetSecondaryZeroOffset(
            secondary_zero_offset,
        ))
        .await;
    match ENCODER_TASK_RESPONSE_CHANNEL.receiver().receive().await {
        EncoderTaskResponse::SecondaryZeroOffsetSet => true,
    }
}

#[task]
pub(crate) async fn encoder_task(
    mut dual_encoder: DualEncoder<As5047P<'static>, As5047P<'static>>,
) {
    let mut tracker = AngleTracker::new(VELOCITY_OBSERVER_BANDWIDTH);
    let mut correction_version = 0;
    let mut correction = runtime_snapshot();
    let command_receiver = ENCODER_TASK_COMMAND_CHANNEL.receiver();
    let response_sender = ENCODER_TASK_RESPONSE_CHANNEL.sender();

    loop {
        let latest_version = runtime_version();
        if latest_version != correction_version {
            correction = runtime_snapshot();
            correction_version = latest_version;
        }

        if let Ok(command) = command_receiver.try_receive()
            && handle_encoder_task_command(command, &mut dual_encoder, &response_sender).await
        {
            tracker = AngleTracker::new(VELOCITY_OBSERVER_BANDWIDTH);
            continue;
        }

        match dual_encoder.read_pair_async().await {
            Ok(dual_reading) => {
                let zeroed_angle = correction.correct_wrapped_angle(dual_reading.primary.phase);
                let now = Instant::now();
                let mut encoder_reading = if tracker.is_uninitialized() {
                    let startup_turns = dual_encoder.last_rev_count_estimate().unwrap_or(0);
                    tracker.seed(zeroed_angle, startup_turns, now)
                } else {
                    tracker.update(zeroed_angle, now)
                };
                encoder_reading.phase = zeroed_angle;
                write_sensor_snapshot(&encoder_reading, dual_reading.secondary_phase);
            }
            Err(DualEncoderReadError::Primary(as5047p::Error::CommandFrame)) => {
                if dual_encoder
                    .primary_mut()
                    .read_and_reset_error_flag()
                    .await
                    .is_err()
                {
                    error!("ENC1 CommandFrame error correction failed");
                }
            }
            Err(DualEncoderReadError::Primary(as5047p::Error::Parity)) => {
                if dual_encoder
                    .primary_mut()
                    .read_and_reset_error_flag()
                    .await
                    .is_err()
                {
                    error!("ENC1 Parity error correction failed");
                }
            }
            Err(DualEncoderReadError::Primary(e)) => error!("Failed to read from ENC1: {:?}", e),
            Err(DualEncoderReadError::Secondary(as5047p::Error::CommandFrame)) => {
                if dual_encoder
                    .secondary_mut()
                    .read_and_reset_error_flag()
                    .await
                    .is_err()
                {
                    error!("ENC2 CommandFrame error correction failed");
                }
            }
            Err(DualEncoderReadError::Secondary(as5047p::Error::Parity)) => {
                if dual_encoder
                    .secondary_mut()
                    .read_and_reset_error_flag()
                    .await
                    .is_err()
                {
                    error!("ENC2 Parity error correction failed");
                }
            }
            Err(DualEncoderReadError::Secondary(e)) => {
                error!("Failed to read from ENC2: {:?}", e)
            }
        }

        Timer::after_micros(50).await;
    }
}

pub(crate) fn read_sensor() -> EncoderReading {
    active_sensor_snapshot().read_encoder()
}

#[cfg(feature = "tuner-fw")]
pub(crate) fn read_sensor_pair() -> (EncoderReading, f32) {
    let snapshot = active_sensor_snapshot();
    (snapshot.read_encoder(), load_f32(&snapshot.secondary_phase))
}
