use defmt::{info, warn};
use embassy_executor::task;
use embassy_stm32::can::{self as stm32_can, CanRx, CanTx};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex},
    channel::{Channel, Sender},
};
use embedded_can::Id;

use can_message::message::{
    Command, CommandMessage, ParameterIndex, ParameterValue, ResponseBody, ResponseMessage,
};

use crate::{
    can::{convert_response_message, parse_command_frame},
    foc_isr, read_sensor,
};

use super::{
    ACTUATOR_REDUCTION_RATIO,
    fault::{has_active_fault, set_gate_driver_enabled, take_motor_disarmed_by_fault},
    persistence::{
        CAN_ID, CAN_PROPERTIES, save_can_id_to_flash, save_output_zero_offset_to_flash,
        save_runtime_config,
    },
};

macro_rules! dispatch_set_param {
    ($cmd:expr, $( $variant:ident => $setter:ident ),* $(,)?) => {
        match $cmd {
            $(
                Command::SetParameter(ParameterValue::$variant(v)) => {
                    foc_isr::with_foc(|foc| foc.$setter(*v));
                }
            )*
            _ => {}
        }
    };
}

#[inline]
fn input_to_output_shaft(value: f32) -> f32 {
    value / ACTUATOR_REDUCTION_RATIO
}

macro_rules! dispatch_get_param {
    ($p:expr, $( $variant:ident => $value:expr ),* $(,)?) => {
        match $p {
            $(
                ParameterIndex::$variant => ParameterValue::$variant($value),
            )*
        }
    };
}

#[derive(Clone, Copy)]
pub(crate) struct QueuedResponse {
    pub(crate) host_can_id: u8,
    pub(crate) body: ResponseBody,
}

static COMMAND_CHANNEL: Channel<ThreadModeRawMutex, CommandMessage, 32> = Channel::new();
/// Response channel uses CriticalSectionRawMutex because it's accessed from ADC interrupt
pub(crate) static RESPONSE_CHANNEL: Channel<CriticalSectionRawMutex, QueuedResponse, 64> =
    Channel::new();

pub(crate) fn can_id_filter(can_id: u8) -> stm32_can::filter::ExtendedFilter {
    stm32_can::filter::ExtendedFilter {
        filter: stm32_can::filter::FilterType::BitMask {
            filter: can_id as u32,
            mask: 0xFF,
        },
        action: stm32_can::filter::Action::StoreInFifo1,
    }
}

pub(crate) async fn update_can_hardware_filter(can_id: u8) {
    let guard = CAN_PROPERTIES.lock().await;
    if let Some(properties) = guard.as_ref() {
        properties.set_extended_filter(
            stm32_can::filter::ExtendedFilterSlot::_0,
            can_id_filter(can_id),
        );
    }
}

#[task]
pub(crate) async fn command_task() {
    let command_receiver = COMMAND_CHANNEL.receiver();
    let response_sender = RESPONSE_CHANNEL.sender();

    loop {
        let message = command_receiver.receive().await;
        let command = message.command;

        match command {
            Command::SetFeedbackInterval(period) => {
                foc_isr::set_feedback_host_can_id(message.host_can_id);
                foc_isr::set_feedback_period(period);
            }
            Command::SetFeedbackType(typ) => {
                foc_isr::set_feedback_host_can_id(message.host_can_id);
                foc_isr::set_feedback_type(typ);
            }
            Command::Recalibrate => {
                info!("Recalibrate command received, clearing config and resetting");
                super::persistence::clear_config_and_reset("recalibration").await;
            }
            Command::SetZeroPosition => {
                info!("SetZeroPosition command received");
                let current_input_angle = read_sensor().cumulative_angle;
                let current_output_angle = input_to_output_shaft(current_input_angle);
                foc_isr::set_output_zero_offset(current_output_angle);
                save_output_zero_offset_to_flash(current_output_angle).await;
                let _ = foc_isr::with_foc(|foc| {
                    foc.set_target_angle(current_input_angle);
                    foc.state.angle_error = 0.0;
                    foc.state.angle_integral = 0.0;
                    foc.state.velocity_error = 0.0;
                    foc.state.velocity_integral = 0.0;
                });
            }
            Command::RunMotorTuning => {
                info!("RunMotorTuning command received, clearing config and resetting");
                super::persistence::clear_config_and_reset("full motor tuning").await;
            }
            Command::SaveParameters | Command::SaveConfig => {
                info!("Save-to-flash command received");
                save_runtime_config().await;
            }
            _ => {
                handle_command(&command, message.host_can_id, &response_sender).await;
            }
        }
    }
}

async fn handle_command(
    command: &Command,
    host_can_id: u8,
    response_sender: &Sender<'static, CriticalSectionRawMutex, QueuedResponse, 64>,
) {
    match command {
        Command::Enable => {
            if has_active_fault() {
                warn!("Enable rejected while fault is active");
            } else {
                if take_motor_disarmed_by_fault() {
                    set_gate_driver_enabled(true).await;
                }
                foc_isr::with_foc(|foc| foc.enable());
            }
        }
        Command::Stop => {
            foc_isr::with_foc(|foc| foc.stop());
        }
        Command::SetParameter(ParameterValue::RunMode(m)) => {
            info!("received runmode {}", *m as u8);
            foc_isr::with_foc(|foc| foc.set_run_mode(decode_run_mode(m)));
        }
        Command::SetParameter(ParameterValue::AngleRef(angle)) => {
            foc_isr::with_foc(|foc| foc.set_target_angle(foc_isr::user_to_input_angle(*angle)));
        }
        Command::SetParameter(ParameterValue::SpeedRef(velocity)) => {
            foc_isr::with_foc(|foc| foc.set_target_velocity(*velocity * ACTUATOR_REDUCTION_RATIO));
        }
        Command::SetParameter(ParameterValue::SpeedLimit(velocity_limit)) => {
            foc_isr::with_foc(|foc| {
                foc.set_velocity_limit(*velocity_limit * ACTUATOR_REDUCTION_RATIO)
            });
        }
        _ => {
            dispatch_set_param!(command,
                IqRef => set_target_torque,
                CurrentLimit => set_current_limit,
                TorqueLimit => set_torque_limit,
                CurrentKp => set_current_kp,
                CurrentKi => set_current_ki,
                SpeedKp => set_velocity_kp,
                SpeedKi => set_velocity_ki,
                Iq => set_target_torque,
                AngleKp => set_angle_kp,
                Spring => set_spring,
                Damping => set_damping,
                VqRef => set_target_voltage,
            );
        }
    }

    if let Command::RequestStatus(_) = command {
        foc_isr::set_feedback_host_can_id(host_can_id);
    }

    if let Command::GetParameter(p) = command {
        info!("parameter requested {}", *p as u16);
        let pv = foc_isr::with_foc(|foc| {
            dispatch_get_param!(p,
                RunMode => encode_run_mode(&foc.mode),
                IqRef => foc.user_frame_i_ref(),
                Angle => foc_isr::input_to_user_angle(foc.state.angle),
                AngleRef => foc_isr::input_to_user_angle(foc.target.angle),
                Speed => input_to_output_shaft(foc.state.velocity),
                SpeedRef => input_to_output_shaft(foc.target.velocity),
                SpeedLimit => input_to_output_shaft(foc.velocity_limit.unwrap_or(0.0)),
                TorqueLimit => foc.torque_limit.unwrap_or(0.0),
                Iq => foc.user_frame_i_q(),
                AngleKp => foc.angle_pid.gains.p,
                SpeedKp => foc.velocity_pid.gains.p,
                SpeedKi => foc.velocity_pid.gains.i,
                CurrentKp => foc.current_q_pid.gains.p,
                CurrentKi => foc.current_q_pid.gains.i,
                CurrentFilter => foc.current_q_filter.time_constant,
                CurrentLimit => foc.current_limit.unwrap_or(0.0),
                Spring => foc.target.spring,
                Damping => foc.target.damping,
                VqRef => foc.target.voltage,
            )
        });
        if let Some(pv) = pv {
            response_sender
                .send(QueuedResponse {
                    host_can_id,
                    body: ResponseBody::ParameterValue(pv),
                })
                .await;
        }
    }
}

#[task]
pub(crate) async fn can_tx_task(mut can_tx: CanTx<'static>) {
    let response_receiver = RESPONSE_CHANNEL.receiver();

    loop {
        let r = response_receiver.receive().await;
        let can_id = CAN_ID.load(core::sync::atomic::Ordering::Relaxed);
        let message = ResponseMessage::new(can_id, r.host_can_id, r.body);
        if let Ok(frame) = convert_response_message(message) {
            can_tx.write(&frame).await;
        }
    }
}

#[task]
pub(crate) async fn can_rx_task(mut can_rx: CanRx<'static>) {
    let command_sender = COMMAND_CHANNEL.sender();

    loop {
        if let Ok(m) = can_rx.read().await {
            if let Ok(message) = parse_command_frame(m.frame) {
                let can_id = CAN_ID.load(core::sync::atomic::Ordering::Relaxed);
                if message.motor_can_id != can_id {
                    continue;
                }
                match message.command {
                    Command::SetCanId(new_can_id) => {
                        CAN_ID.store(new_can_id, core::sync::atomic::Ordering::Relaxed);
                        update_can_hardware_filter(new_can_id).await;
                        info!("CAN ID changed to {}", new_can_id);
                        save_can_id_to_flash(new_can_id).await;
                    }
                    cmd => {
                        if command_sender
                            .try_send(CommandMessage {
                                motor_can_id: message.motor_can_id,
                                host_can_id: message.host_can_id,
                                command: cmd,
                            })
                            .is_err()
                        {
                            warn!("failed to send cmd");
                        }
                    }
                };
            } else {
                let id = match m.frame.header().id() {
                    Id::Standard(x) => x.as_raw() as u32,
                    Id::Extended(x) => x.as_raw(),
                };
                warn!("parse failed: {}#{}", id, m.frame.data());
            }
        }
    }
}

fn decode_run_mode(m: &can_message::message::RunMode) -> foc::controller::RunMode {
    match m {
        can_message::message::RunMode::Impedance => foc::controller::RunMode::Impedance,
        can_message::message::RunMode::Angle => foc::controller::RunMode::Angle,
        can_message::message::RunMode::Velocity => foc::controller::RunMode::Velocity,
        can_message::message::RunMode::Torque => foc::controller::RunMode::Torque,
        can_message::message::RunMode::Voltage => foc::controller::RunMode::Voltage,
    }
}

fn encode_run_mode(m: &foc::controller::RunMode) -> can_message::message::RunMode {
    match m {
        foc::controller::RunMode::Impedance => can_message::message::RunMode::Impedance,
        foc::controller::RunMode::Angle => can_message::message::RunMode::Angle,
        foc::controller::RunMode::Velocity => can_message::message::RunMode::Velocity,
        foc::controller::RunMode::Torque => can_message::message::RunMode::Torque,
        foc::controller::RunMode::Voltage => can_message::message::RunMode::Voltage,
    }
}
