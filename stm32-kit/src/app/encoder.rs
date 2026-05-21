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
    full_rotations: AtomicI32,
    cumulative_angle: AtomicU32,
    velocity: AtomicU32,
    dt: AtomicU32,
}

impl SensorSnapshotBuffer {
    const fn new() -> Self {
        Self {
            phase: AtomicU32::new(0),
            full_rotations: AtomicI32::new(0),
            cumulative_angle: AtomicU32::new(0),
            velocity: AtomicU32::new(0),
            dt: AtomicU32::new(0),
        }
    }
}

static SENSOR_SNAPSHOTS: [SensorSnapshotBuffer; 2] =
    [SensorSnapshotBuffer::new(), SensorSnapshotBuffer::new()];
static ACTIVE_SENSOR_SNAPSHOT: AtomicU8 = AtomicU8::new(0);

#[derive(Clone, Copy)]
enum EncoderTaskCommand {
    CaptureZeroOffset,
}

#[derive(Clone, Copy)]
enum EncoderTaskResponse {
    Captured(f32, f32),
    CaptureFailed,
}

static ENCODER_TASK_COMMAND_CHANNEL: Channel<ThreadModeRawMutex, EncoderTaskCommand, 1> =
    Channel::new();
static ENCODER_TASK_RESPONSE_CHANNEL: Channel<ThreadModeRawMutex, EncoderTaskResponse, 1> =
    Channel::new();

fn write_sensor_snapshot(encoder_reading: &EncoderReading) {
    let active_index = ACTIVE_SENSOR_SNAPSHOT.load(Ordering::Relaxed) as usize;
    let next_index = active_index ^ 1;
    let snapshot = &SENSOR_SNAPSHOTS[next_index];

    snapshot.phase.store(
        u32::from_le_bytes(encoder_reading.phase.to_le_bytes()),
        Ordering::Relaxed,
    );
    snapshot
        .full_rotations
        .store(encoder_reading.full_rotations, Ordering::Relaxed);
    snapshot.cumulative_angle.store(
        u32::from_le_bytes(encoder_reading.cumulative_angle.to_le_bytes()),
        Ordering::Relaxed,
    );
    snapshot.velocity.store(
        u32::from_le_bytes(encoder_reading.velocity.to_le_bytes()),
        Ordering::Relaxed,
    );
    snapshot.dt.store(
        u32::from_le_bytes(encoder_reading.dt.to_le_bytes()),
        Ordering::Relaxed,
    );

    // Writer runs in thread mode and reader runs in ADC ISR, so once the
    // index is published the ISR can only observe a fully written snapshot.
    ACTIVE_SENSOR_SNAPSHOT.store(next_index as u8, Ordering::Release);
}

#[derive(defmt::Format)]
enum ZeroOffsetWriteError {
    Primary(as5047p::Error),
    Secondary(as5047p::Error),
}

async fn write_zero_offsets(
    dual_encoder: &mut DualEncoder<As5047P<'static>, As5047P<'static>>,
) -> Result<(f32, f32), ZeroOffsetWriteError> {
    dual_encoder
        .primary_mut()
        .set_current_angle_as_zero()
        .await
        .map_err(ZeroOffsetWriteError::Primary)?;
    dual_encoder
        .secondary_mut()
        .set_current_angle_as_zero()
        .await
        .map_err(ZeroOffsetWriteError::Secondary)?;
    Ok((
        dual_encoder.primary_mut().zero_offset(),
        dual_encoder.secondary_mut().zero_offset(),
    ))
}

async fn handle_encoder_task_command(
    command: EncoderTaskCommand,
    dual_encoder: &mut DualEncoder<As5047P<'static>, As5047P<'static>>,
    response_sender: &Sender<'static, ThreadModeRawMutex, EncoderTaskResponse, 1>,
) -> bool {
    match command {
        EncoderTaskCommand::CaptureZeroOffset => match write_zero_offsets(dual_encoder).await {
            Ok((primary_zero_offset, secondary_zero_offset)) => {
                response_sender
                    .send(EncoderTaskResponse::Captured(
                        primary_zero_offset,
                        secondary_zero_offset,
                    ))
                    .await;
                true
            }
            Err(ZeroOffsetWriteError::Primary(e)) => {
                error!("Failed to set ENC1 zero offset: {:?}", e);
                response_sender
                    .send(EncoderTaskResponse::CaptureFailed)
                    .await;
                false
            }
            Err(ZeroOffsetWriteError::Secondary(e)) => {
                error!("Failed to set ENC2 zero offset: {:?}", e);
                response_sender
                    .send(EncoderTaskResponse::CaptureFailed)
                    .await;
                false
            }
        },
    }
}

pub(crate) async fn request_zero_offset_capture() -> Option<(f32, f32)> {
    ENCODER_TASK_COMMAND_CHANNEL
        .sender()
        .send(EncoderTaskCommand::CaptureZeroOffset)
        .await;
    match ENCODER_TASK_RESPONSE_CHANNEL.receiver().receive().await {
        EncoderTaskResponse::Captured(primary_zero_offset, secondary_zero_offset) => {
            Some((primary_zero_offset, secondary_zero_offset))
        }
        EncoderTaskResponse::CaptureFailed => None,
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
                let mut encoder_reading = tracker.update(zeroed_angle, Instant::now());
                encoder_reading.phase = zeroed_angle;
                write_sensor_snapshot(&encoder_reading);
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
    let active_index = ACTIVE_SENSOR_SNAPSHOT.load(Ordering::Acquire) as usize;
    let snapshot = &SENSOR_SNAPSHOTS[active_index];
    let raw_phase = snapshot.phase.load(Ordering::Relaxed);
    let full_rotations = snapshot.full_rotations.load(Ordering::Relaxed);
    let raw_cumulative_angle = snapshot.cumulative_angle.load(Ordering::Relaxed);
    let raw_velocity = snapshot.velocity.load(Ordering::Relaxed);
    let raw_dt = snapshot.dt.load(Ordering::Relaxed);

    EncoderReading {
        phase: f32::from_le_bytes(raw_phase.to_le_bytes()),
        full_rotations,
        cumulative_angle: f32::from_le_bytes(raw_cumulative_angle.to_le_bytes()),
        velocity: f32::from_le_bytes(raw_velocity.to_le_bytes()),
        dt: f32::from_le_bytes(raw_dt.to_le_bytes()),
    }
}
