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
    as5047p::{self, As5047P, raw_to_angle},
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
static ANGLE_2: AtomicU32 = AtomicU32::new(0);

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

fn write_sensor_snapshot(encoder_reading: EncoderReading) {
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
    sensor: &mut As5047P<'static>,
    secondary_sensor: &mut As5047P<'static>,
) -> Result<(f32, f32), ZeroOffsetWriteError> {
    sensor
        .set_current_angle_as_zero()
        .await
        .map_err(ZeroOffsetWriteError::Primary)?;
    secondary_sensor
        .set_current_angle_as_zero()
        .await
        .map_err(ZeroOffsetWriteError::Secondary)?;
    Ok((sensor.zero_offset(), secondary_sensor.zero_offset()))
}

async fn handle_encoder_task_command(
    command: EncoderTaskCommand,
    sensor: &mut As5047P<'static>,
    secondary_sensor: &mut As5047P<'static>,
    response_sender: &Sender<'static, ThreadModeRawMutex, EncoderTaskResponse, 1>,
) -> bool {
    match command {
        EncoderTaskCommand::CaptureZeroOffset => {
            match write_zero_offsets(sensor, secondary_sensor).await {
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
            }
        }
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
    mut sensor: As5047P<'static>,
    mut secondary_sensor: As5047P<'static>,
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
            && handle_encoder_task_command(
                command,
                &mut sensor,
                &mut secondary_sensor,
                &response_sender,
            )
            .await
        {
            tracker = AngleTracker::new(VELOCITY_OBSERVER_BANDWIDTH);
            continue;
        }

        match sensor.read_raw_angle().await {
            Ok(raw_angle) => {
                let zeroed_angle = correction.correct_wrapped_angle(raw_to_angle(raw_angle));
                let mut encoder_reading = tracker.update(zeroed_angle, Instant::now());
                encoder_reading.phase = zeroed_angle;
                write_sensor_snapshot(encoder_reading);
            }
            Err(as5047p::Error::CommandFrame) => {
                if sensor.read_and_reset_error_flag().await.is_err() {
                    error!("CommandFrame error correction failed");
                }
            }
            Err(as5047p::Error::Parity) => {
                if sensor.read_and_reset_error_flag().await.is_err() {
                    error!("Parity error correction failed");
                }
            }
            Err(e) => error!("Failed to read from sensor: {:?}", e),
        }

        match secondary_sensor.read_raw_angle().await {
            Ok(raw_angle) => {
                let angle_2 = raw_to_angle(raw_angle);
                ANGLE_2.store(u32::from_le_bytes(angle_2.to_le_bytes()), Ordering::Relaxed);
            }
            Err(as5047p::Error::CommandFrame) => {
                if secondary_sensor.read_and_reset_error_flag().await.is_err() {
                    error!("ENC2 CommandFrame error correction failed");
                }
            }
            Err(as5047p::Error::Parity) => {
                if secondary_sensor.read_and_reset_error_flag().await.is_err() {
                    error!("ENC2 Parity error correction failed");
                }
            }
            Err(e) => error!("Failed to read from ENC2: {:?}", e),
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

pub(crate) fn read_secondary_angle() -> f32 {
    f32::from_le_bytes(ANGLE_2.load(Ordering::Relaxed).to_le_bytes())
}
