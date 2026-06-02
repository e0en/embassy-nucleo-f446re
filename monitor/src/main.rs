#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
use std::collections::{HashMap, VecDeque};
use std::fs::File;
use std::io::{BufRead, BufReader, prelude::Write};
use std::process::{Command as ProcessCommand, Stdio};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::thread::sleep;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use std::{f32, thread};

use can_message::message::{
    CanMessage, Command, CommandMessage, DebugValueKind, FeedbackType, MotionControl,
    ParameterIndex, ParameterValue, ResponseBody, ResponseMessage, RunMode,
};
use eframe::egui;
use socketcan::socket::{CanSocket, Socket};
use socketcan::{CanFrame, EmbeddedFrame, ExtendedId};

const WINDOW_WIDTH: f32 = 1024.0;
const WINDOW_HEIGHT: f32 = 768.0;
const MAX_PLOT_POINTS: usize = 100_000;
const UI_REPAINT_INTERVAL: Duration = Duration::from_millis(16);
const CHIRP_COMMAND_INTERVAL: Duration = Duration::from_millis(10);
const CHIRP_START_FREQ_HZ: f32 = 0.2;
const CHIRP_END_FREQ_HZ: f32 = 10.0;
const HOST_CAN_ID: u8 = 0x00;
const DEFAULT_MOTOR_CAN_ID: u8 = 0x0F;
const LOADCELL_MONITOR_COMMAND: &str =
    "Developer/esp32-c3-nau7802/pc-client/target/release/pc-client";
const GRAM_FORCE_TO_NEWTON: f32 = 0.009_806_65;

fn main() -> eframe::Result {
    env_logger::init();

    let (command_sender, command_receiver) = channel::<CommandMessage>();
    let (command_event_sender, command_event_receiver) = channel::<CommandEvent>();
    let (status_sender, status_receiver) = channel::<ResponseMessage>();
    let (sequence_control_sender, sequence_control_receiver) = channel::<SequenceControl>();
    let (sequence_event_sender, sequence_event_receiver) = channel::<SequenceEvent>();
    let (loadcell_sender, loadcell_receiver) = channel::<LoadCellEvent>();

    let rx_socket = CanSocket::open("can0").unwrap();
    let tx_socket = CanSocket::open("can0").unwrap();
    let sequence_command_sender = command_sender.clone();

    thread::spawn(move || {
        forward_status(status_sender, rx_socket);
    });
    thread::spawn(move || {
        forward_command(command_receiver, command_event_sender, tx_socket);
    });
    thread::spawn(move || {
        forward_sequence(
            sequence_control_receiver,
            sequence_event_sender,
            sequence_command_sender,
        );
    });
    thread::spawn(move || {
        forward_loadcell(loadcell_sender);
    });

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([WINDOW_WIDTH, WINDOW_HEIGHT]),
        ..Default::default()
    };
    eframe::run_native(
        "BLDC monitor",
        options,
        Box::new(|_| {
            Ok(Box::<MyApp>::new(MyApp::new(
                command_sender,
                command_event_receiver,
                status_receiver,
                sequence_control_sender,
                sequence_event_receiver,
                loadcell_receiver,
            )))
        }),
    )
}

fn forward_command(
    command_recv: Receiver<CommandMessage>,
    command_event_send: Sender<CommandEvent>,
    socket: CanSocket,
) {
    loop {
        for mut command_msg in command_recv.try_iter() {
            command_msg.host_can_id = HOST_CAN_ID;
            let command_event = command_event_from_message(&command_msg);
            if let Ok(can_msg) = CanMessage::try_from(command_msg) {
                match can_message_to_can_frame(&can_msg) {
                    Ok(frame) => {
                        if let Err(e) = socket.write_frame(&frame) {
                            println!("Failed to send CAN frame: {}", e);
                        } else if let Some(command_event) = command_event {
                            let _ = command_event_send.send(command_event);
                        }
                    }
                    Err(e) => {
                        println!("Failed to create CAN frame: {}", e);
                    }
                }
            }
        }
        thread::sleep(Duration::from_millis(1));
    }
}

fn forward_status(status_send: Sender<ResponseMessage>, socket: CanSocket) {
    println!("CAN receiver started");

    loop {
        match socket.read_frame() {
            Ok(frame) => {
                let can_msg = can_frame_to_can_message(&frame);
                if let Ok(status_msg) = ResponseMessage::try_from(can_msg)
                    && response_matches_host(&status_msg)
                {
                    let _ = status_send.send(status_msg);
                }
            }
            Err(e) => {
                println!("Failed to read CAN frame: {}", e);
                thread::sleep(Duration::from_millis(10));
            }
        }
    }
}

fn forward_sequence(
    control_recv: Receiver<SequenceControl>,
    event_send: Sender<SequenceEvent>,
    command_send: Sender<CommandMessage>,
) {
    let mut runtime_sequences: HashMap<u8, RuntimeSequence> = HashMap::new();

    loop {
        for control in control_recv.try_iter() {
            match control {
                SequenceControl::Start { can_id, sequence } => {
                    runtime_sequences.insert(can_id, RuntimeSequence::from(sequence));
                    send_sequence_command(
                        &command_send,
                        can_id,
                        Command::SetParameter(ParameterValue::RunMode(sequence.run_mode())),
                    );

                    if let Some((target, value)) = sequence.initial_command_value() {
                        send_sequence_command(
                            &command_send,
                            can_id,
                            sequence_parameter(target, value),
                        );
                        let _ = event_send.send(SequenceEvent::Updated {
                            can_id,
                            target,
                            value,
                        });
                    }
                }
                SequenceControl::Stop { can_id } => {
                    if let Some(runtime) = runtime_sequences.remove(&can_id) {
                        let target = runtime.target();
                        send_sequence_command(
                            &command_send,
                            can_id,
                            sequence_parameter(target, 0.0),
                        );
                        let _ = event_send.send(SequenceEvent::Updated {
                            can_id,
                            target,
                            value: 0.0,
                        });
                        let _ = event_send.send(SequenceEvent::Finished { can_id });
                    }
                }
            }
        }

        let now = Instant::now();
        let mut finished_can_ids = Vec::new();

        for (&can_id, runtime) in &mut runtime_sequences {
            match runtime.tick(now) {
                RuntimeTick::Idle => {}
                RuntimeTick::Send { target, value } => {
                    send_sequence_command(&command_send, can_id, sequence_parameter(target, value));
                    let _ = event_send.send(SequenceEvent::Updated {
                        can_id,
                        target,
                        value,
                    });
                }
                RuntimeTick::Finished { target } => {
                    send_sequence_command(&command_send, can_id, sequence_parameter(target, 0.0));
                    let _ = event_send.send(SequenceEvent::Updated {
                        can_id,
                        target,
                        value: 0.0,
                    });
                    let _ = event_send.send(SequenceEvent::Finished { can_id });
                    finished_can_ids.push(can_id);
                }
            }
        }

        for can_id in finished_can_ids {
            runtime_sequences.remove(&can_id);
        }

        thread::sleep(Duration::from_millis(1));
    }
}

fn forward_loadcell(event_send: Sender<LoadCellEvent>) {
    let home = std::env::var("HOME").unwrap_or_else(|_| ".".to_owned());
    let command_path = std::path::Path::new(&home).join(LOADCELL_MONITOR_COMMAND);
    let mut child = match ProcessCommand::new(command_path)
        .arg("monitor")
        .stdout(Stdio::piped())
        .spawn()
    {
        Ok(child) => child,
        Err(e) => {
            let _ = event_send.send(LoadCellEvent::Error(format!("loadcell monitor: {e}")));
            return;
        }
    };

    let Some(stdout) = child.stdout.take() else {
        let _ = event_send.send(LoadCellEvent::Error(
            "loadcell monitor stdout unavailable".to_owned(),
        ));
        return;
    };

    for line in BufReader::new(stdout).lines() {
        match line {
            Ok(line) => match line.trim().parse::<f32>() {
                Ok(grams) => {
                    let _ = event_send.send(LoadCellEvent::Reading { grams });
                }
                Err(_) => {
                    let _ = event_send.send(LoadCellEvent::Error(format!(
                        "loadcell parse error: {line}"
                    )));
                }
            },
            Err(e) => {
                let _ = event_send.send(LoadCellEvent::Error(format!("loadcell read: {e}")));
                break;
            }
        }
    }
}

fn send_sequence_command(command_send: &Sender<CommandMessage>, can_id: u8, command: Command) {
    let _ = command_send.send(CommandMessage {
        host_can_id: HOST_CAN_ID,
        motor_can_id: can_id,
        command,
    });
}

fn command_event_from_message(command_msg: &CommandMessage) -> Option<CommandEvent> {
    if let Command::MotionControl(command) = &command_msg.command {
        return Some(CommandEvent::MotionSent {
            can_id: command_msg.motor_can_id,
            angle: command.angle,
            velocity: command.velocity,
            torque: command.torque,
        });
    }

    let target_value = match &command_msg.command {
        Command::SetParameter(ParameterValue::AngleRef(value)) => {
            Some((ChirpTarget::Angle, *value))
        }
        Command::SetParameter(ParameterValue::SpeedRef(value)) => {
            Some((ChirpTarget::Velocity, *value))
        }
        Command::SetParameter(ParameterValue::TorqueRef(value)) => {
            Some((ChirpTarget::Torque, *value))
        }
        _ => None,
    }?;

    Some(CommandEvent::RefSent {
        can_id: command_msg.motor_can_id,
        target: target_value.0,
        value: target_value.1,
    })
}

struct MyApp {
    command_send: Sender<CommandMessage>,
    command_event_recv: Receiver<CommandEvent>,
    status_recv: Receiver<ResponseMessage>,
    sequence_control_send: Sender<SequenceControl>,
    sequence_event_recv: Receiver<SequenceEvent>,
    loadcell_recv: Receiver<LoadCellEvent>,
    latest_loadcell_grams: Option<f32>,
    loadcell_sequence: u64,
    loadcell_status: String,
    tabs: Vec<MotorTab>,
    selected_tab: usize,
    new_tab_can_id_string: String,
    file_dialog: egui_file_dialog::FileDialog,
    pending_export: Option<PendingExport>,
}

#[derive(Clone, Debug)]
enum LoadCellEvent {
    Reading { grams: f32 },
    Error(String),
}

struct MotorTab {
    can_id: u8,
    rename_can_id_string: String,
    feedback_interval_string: String,

    angle_string: String,
    velocity_string: String,
    torque_string: String,
    vq_string: String,

    angle_kp_string: String,
    speed_kp_string: String,
    speed_ki_string: String,

    iq_kp_string: String,
    iq_ki_string: String,

    spring_string: String,
    damping_string: String,
    motion_angle_string: String,
    motion_velocity_string: String,
    motion_torque_string: String,
    motion_kp_string: String,
    motion_kd_string: String,
    cal_iq_start_string: String,
    cal_iq_end_string: String,
    cal_iq_step_string: String,
    cal_settle_ms_string: String,
    cal_sample_count_string: String,
    cal_arm_mm_string: String,
    cal_repeat_count_string: String,
    cal_stability_std_g_string: String,
    cal_stability_samples_string: String,
    cal_stability_timeout_ms_string: String,
    cal_outlier_sigma_string: String,
    cal_release_threshold_g_string: String,
    cal_release_timeout_ms_string: String,
    cal_release_step_rad_string: String,
    cal_release_max_rad_string: String,
    cal_contact_threshold_g_string: String,

    angle: f32,
    velocity: f32,
    torque: f32,
    vq: f32,
    sent_angle_ref: f32,
    sent_velocity_ref: f32,
    sent_torque_ref: f32,

    angle_kp: f32,
    speed_kp: f32,
    speed_ki: f32,

    iq_kp: f32,
    iq_ki: f32,

    spring: f32,
    damping: f32,
    motion_angle: f32,
    motion_velocity: f32,
    motion_torque: f32,
    motion_kp: f32,
    motion_kd: f32,
    cal_iq_start: f32,
    cal_iq_end: f32,
    cal_iq_step: f32,
    cal_settle_ms: usize,
    cal_sample_count: usize,
    cal_arm_mm: f32,
    cal_repeat_count: usize,
    cal_stability_std_g: f32,
    cal_stability_samples: usize,
    cal_stability_timeout_ms: usize,
    cal_outlier_sigma: f32,
    cal_release_threshold_g: f32,
    cal_release_timeout_ms: usize,
    cal_release_step_rad: f32,
    cal_release_max_rad: f32,
    cal_contact_threshold_g: f32,
    chirp_amplitude: f32,
    chirp_duration: f32,
    chirp_target: ChirpTarget,
    step_value: f32,
    step_target: ChirpTarget,

    run_mode: RunMode,
    plot_type: PlotType,
    plot_points_1: VecDeque<egui_plot::PlotPoint>,
    plot_points_2: VecDeque<egui_plot::PlotPoint>,
    is_plotting: bool,
    t0: Instant,
    chirp_amplitude_string: String,
    chirp_duration_string: String,
    step_value_string: String,
    active_sequence: Option<ActiveSequence>,
    active_capture: Option<SequenceCapture>,
    last_capture: Option<SequenceCapture>,
    active_torque_calibration: Option<TorqueCalibration>,
    last_torque_calibration: Option<TorqueCalibrationResult>,
}

#[derive(Clone, Copy, Debug)]
enum ActiveSequence {
    Chirp {
        target: ChirpTarget,
        amplitude: f32,
        duration: Duration,
    },
    Step {
        target: ChirpTarget,
        value: f32,
    },
}

#[derive(Clone, Copy, Debug)]
enum SequenceControl {
    Start {
        can_id: u8,
        sequence: ActiveSequence,
    },
    Stop {
        can_id: u8,
    },
}

#[derive(Clone, Copy, Debug)]
enum SequenceEvent {
    Updated {
        can_id: u8,
        target: ChirpTarget,
        value: f32,
    },
    Finished {
        can_id: u8,
    },
}

enum PendingExport {
    Plot(Vec<egui_plot::PlotPoint>),
    Sequence(SequenceCapture),
    TorqueCalibration(TorqueCalibrationResult),
}

#[derive(Clone, Copy, Debug)]
enum CommandEvent {
    RefSent {
        can_id: u8,
        target: ChirpTarget,
        value: f32,
    },
    MotionSent {
        can_id: u8,
        angle: f32,
        velocity: f32,
        torque: f32,
    },
}

#[derive(Clone)]
struct SequenceCapture {
    sequence: ActiveSequence,
    rows: Vec<SequenceCaptureRow>,
}

#[derive(Clone)]
struct TorqueCalibrationResult {
    arm_mm: f32,
    tare_g: f32,
    fit_zero_slope_nm_per_iq: Option<f32>,
    fit_affine_slope_nm_per_iq: Option<f32>,
    fit_affine_intercept_nm: Option<f32>,
    fit_zero_rmse_nm: Option<f32>,
    fit_affine_rmse_nm: Option<f32>,
    fit_zero_r2: Option<f32>,
    fit_affine_r2: Option<f32>,
    rows: Vec<TorqueCalibrationRow>,
}

#[derive(Clone, Copy)]
struct TorqueCalibrationRow {
    iq_ref: f32,
    repeat: usize,
    direction: CalibrationDirection,
    raw_mean_g: f32,
    tare_corrected_mean_g: f32,
    std_g: f32,
    rejected_samples: usize,
    sample_count: usize,
    torque_nm: f32,
}

#[derive(Clone)]
struct TorqueCalibration {
    steps: Vec<TorqueCalibrationStep>,
    step_index: usize,
    min_settle_duration: Duration,
    sample_count: usize,
    arm_mm: f32,
    tare_g: f32,
    stability_std_g: f32,
    stability_samples: usize,
    stability_timeout: Duration,
    outlier_sigma: f32,
    release_threshold_g: f32,
    release_timeout: Duration,
    release_step_rad: f32,
    release_max_rad: f32,
    contact_threshold_g: f32,
    release_start_angle: f32,
    release_target_angle: f32,
    release_direction: f32,
    release_last_abs_g: Option<f32>,
    approach_target_angle: f32,
    phase: TorqueCalibrationPhase,
    rows: Vec<TorqueCalibrationRow>,
    samples: Vec<f32>,
    stability_window: VecDeque<f32>,
    last_loadcell_sequence: u64,
}

#[derive(Clone, Copy)]
enum TorqueCalibrationPhase {
    TareCommand,
    TareSettling {
        until: Instant,
    },
    TareSampling,
    ReleaseCommand,
    WaitingForRelease {
        timeout_at: Instant,
        last_step_at: Instant,
    },
    ApproachCommand,
    WaitingForContact {
        timeout_at: Instant,
        last_step_at: Instant,
    },
    CommandStep,
    Settling {
        min_until: Instant,
        timeout_at: Instant,
    },
    Sampling,
}

#[derive(Clone, Copy)]
struct TorqueCalibrationStep {
    iq_ref: f32,
    repeat: usize,
    direction: CalibrationDirection,
}

#[derive(Clone, Copy)]
enum CalibrationDirection {
    Descending,
    Ascending,
}

#[derive(Clone, Copy)]
enum SequenceCaptureRow {
    Command {
        timestamp_ms: u128,
        target: ChirpTarget,
        value: f32,
    },
    Status {
        timestamp_ms: u128,
        angle: f32,
        velocity: f32,
        torque: f32,
        temperature: f32,
    },
    Current {
        timestamp_ms: u128,
        i_q: f32,
        i_d: f32,
    },
    DebugValue {
        timestamp_ms: u128,
        kind: DebugValueKind,
        value: f32,
    },
}

struct RuntimeChirp {
    target: ChirpTarget,
    amplitude: f32,
    duration: Duration,
    t0: Instant,
    last_command_at: Option<Instant>,
}

enum RuntimeSequence {
    Chirp(RuntimeChirp),
    Step { target: ChirpTarget },
}

enum SequenceUiRequest {
    Start(ActiveSequence),
    Stop,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ChirpTarget {
    Angle,
    Velocity,
    Torque,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PlotType {
    Angle,
    Velocity,
    Torque,
    Current,
    SpeedError,
    TorqueRef,
    VelocityIntegral,
}

impl MotorTab {
    fn new(can_id: u8) -> Self {
        let angle = 0.0;
        let velocity = 0.0;
        let torque = 0.0;
        let vq = 0.0;

        let angle_kp = 0.0;
        let speed_kp = 0.0;
        let speed_ki = 0.0;
        let iq_kp = 0.0;
        let iq_ki = 0.0;

        let spring = 0.0;
        let damping = 0.0;
        let motion_angle = 0.0;
        let motion_velocity = 0.0;
        let motion_torque = 0.0;
        let motion_kp = 0.0;
        let motion_kd = 0.0;
        let cal_iq_start = -0.1;
        let cal_iq_end = -2.0;
        let cal_iq_step = -0.1;
        let cal_settle_ms = 500;
        let cal_sample_count = 10;
        let cal_arm_mm = 100.0;
        let cal_repeat_count = 3;
        let cal_stability_std_g = 2.0;
        let cal_stability_samples = 10;
        let cal_stability_timeout_ms = 3000;
        let cal_outlier_sigma = 3.0;
        let cal_release_threshold_g = 5.0;
        let cal_release_timeout_ms = 30000;
        let cal_release_step_rad = 0.02;
        let cal_release_max_rad = 0.5;
        let cal_contact_threshold_g = 30.0;
        let chirp_amplitude = 5.0;
        let chirp_duration = 30.0;
        let step_value = 5.0;

        Self {
            can_id,
            rename_can_id_string: can_id.to_string(),
            feedback_interval_string: "20".to_owned(),

            angle_string: angle.to_string(),
            velocity_string: velocity.to_string(),
            torque_string: torque.to_string(),
            vq_string: vq.to_string(),

            angle_kp_string: angle_kp.to_string(),
            speed_kp_string: speed_kp.to_string(),
            speed_ki_string: speed_ki.to_string(),

            iq_kp_string: iq_kp.to_string(),
            iq_ki_string: iq_ki.to_string(),

            spring_string: spring.to_string(),
            damping_string: damping.to_string(),
            motion_angle_string: motion_angle.to_string(),
            motion_velocity_string: motion_velocity.to_string(),
            motion_torque_string: motion_torque.to_string(),
            motion_kp_string: motion_kp.to_string(),
            motion_kd_string: motion_kd.to_string(),
            cal_iq_start_string: cal_iq_start.to_string(),
            cal_iq_end_string: cal_iq_end.to_string(),
            cal_iq_step_string: cal_iq_step.to_string(),
            cal_settle_ms_string: cal_settle_ms.to_string(),
            cal_sample_count_string: cal_sample_count.to_string(),
            cal_arm_mm_string: cal_arm_mm.to_string(),
            cal_repeat_count_string: cal_repeat_count.to_string(),
            cal_stability_std_g_string: cal_stability_std_g.to_string(),
            cal_stability_samples_string: cal_stability_samples.to_string(),
            cal_stability_timeout_ms_string: cal_stability_timeout_ms.to_string(),
            cal_outlier_sigma_string: cal_outlier_sigma.to_string(),
            cal_release_threshold_g_string: cal_release_threshold_g.to_string(),
            cal_release_timeout_ms_string: cal_release_timeout_ms.to_string(),
            cal_release_step_rad_string: cal_release_step_rad.to_string(),
            cal_release_max_rad_string: cal_release_max_rad.to_string(),
            cal_contact_threshold_g_string: cal_contact_threshold_g.to_string(),

            angle,
            velocity,
            torque,
            vq,
            sent_angle_ref: angle,
            sent_velocity_ref: velocity,
            sent_torque_ref: torque,

            angle_kp,
            speed_kp,
            speed_ki,

            iq_kp,
            iq_ki,

            spring,
            damping,
            motion_angle,
            motion_velocity,
            motion_torque,
            motion_kp,
            motion_kd,
            cal_iq_start,
            cal_iq_end,
            cal_iq_step,
            cal_settle_ms,
            cal_sample_count,
            cal_arm_mm,
            cal_repeat_count,
            cal_stability_std_g,
            cal_stability_samples,
            cal_stability_timeout_ms,
            cal_outlier_sigma,
            cal_release_threshold_g,
            cal_release_timeout_ms,
            cal_release_step_rad,
            cal_release_max_rad,
            cal_contact_threshold_g,
            chirp_amplitude,
            chirp_duration,
            chirp_target: ChirpTarget::Velocity,
            step_value,
            step_target: ChirpTarget::Velocity,

            run_mode: RunMode::Impedance,
            plot_type: PlotType::Angle,
            plot_points_1: VecDeque::with_capacity(MAX_PLOT_POINTS),
            plot_points_2: VecDeque::with_capacity(MAX_PLOT_POINTS),
            is_plotting: false,
            t0: Instant::now(),
            chirp_amplitude_string: chirp_amplitude.to_string(),
            chirp_duration_string: chirp_duration.to_string(),
            step_value_string: step_value.to_string(),
            active_sequence: None,
            active_capture: None,
            last_capture: None,
            active_torque_calibration: None,
            last_torque_calibration: None,
        }
    }

    fn current_feedback_type(&self) -> FeedbackType {
        match self.plot_type {
            PlotType::Current => FeedbackType::Current,
            PlotType::SpeedError => FeedbackType::SpeedError,
            PlotType::TorqueRef => FeedbackType::TorqueRef,
            PlotType::VelocityIntegral => FeedbackType::VelocityIntegral,
            PlotType::Angle | PlotType::Velocity | PlotType::Torque => FeedbackType::Status,
        }
    }

    fn clear_plot(&mut self) {
        self.plot_points_1.clear();
        self.plot_points_2.clear();
        self.t0 = Instant::now();
    }

    fn chirp_active(&self) -> bool {
        matches!(self.active_sequence, Some(ActiveSequence::Chirp { .. }))
    }

    fn step_active(&self) -> bool {
        matches!(self.active_sequence, Some(ActiveSequence::Step { .. }))
    }

    fn excitation_active(&self) -> bool {
        self.chirp_active() || self.step_active()
    }

    fn active_sequence_target(&self) -> Option<ChirpTarget> {
        self.active_sequence.map(ActiveSequence::target)
    }

    fn set_sequence_value(&mut self, target: ChirpTarget, value: f32) {
        match target {
            ChirpTarget::Angle => {
                self.angle = value;
                self.angle_string = value.to_string();
            }
            ChirpTarget::Velocity => {
                self.velocity = value;
                self.velocity_string = value.to_string();
            }
            ChirpTarget::Torque => {
                self.torque = value;
                self.torque_string = value.to_string();
            }
        }
    }

    fn push_plot_points(&mut self, y1: f64, y2: Option<f64>) {
        let Some(now) = Instant::now().checked_duration_since(self.t0) else {
            return;
        };
        let x = now.as_micros() as f64 / 1e6;
        push_point(&mut self.plot_points_1, egui_plot::PlotPoint::new(x, y1));
        if let Some(y2) = y2 {
            push_point(&mut self.plot_points_2, egui_plot::PlotPoint::new(x, y2));
        }
    }

    fn apply_response(&mut self, response: &ResponseBody) {
        match response {
            ResponseBody::MotorStatus(x) => {
                if !self.is_plotting
                    || matches!(
                        self.plot_type,
                        PlotType::Current
                            | PlotType::SpeedError
                            | PlotType::TorqueRef
                            | PlotType::VelocityIntegral
                    )
                {
                    return;
                }
                let (feedback, reference) = match self.plot_type {
                    PlotType::Angle => (x.angle as f64, self.sent_angle_ref as f64),
                    PlotType::Velocity => (x.velocity as f64, self.sent_velocity_ref as f64),
                    PlotType::Torque => (x.torque as f64, self.sent_torque_ref as f64),
                    PlotType::Current
                    | PlotType::SpeedError
                    | PlotType::TorqueRef
                    | PlotType::VelocityIntegral => return,
                };
                self.push_plot_points(feedback, Some(reference));
            }
            ResponseBody::MotorCurrent(x) => {
                if !self.is_plotting || self.plot_type != PlotType::Current {
                    return;
                }
                self.push_plot_points(x.i_q as f64, Some(x.i_d as f64));
            }
            ResponseBody::DebugValue(x) => {
                if !self.is_plotting {
                    return;
                }
                let expected_kind = match self.plot_type {
                    PlotType::SpeedError => Some(DebugValueKind::SpeedError),
                    PlotType::TorqueRef => Some(DebugValueKind::TorqueRef),
                    PlotType::VelocityIntegral => Some(DebugValueKind::VelocityIntegral),
                    _ => None,
                };
                if expected_kind != Some(x.kind) {
                    return;
                }
                self.push_plot_points(x.value as f64, None);
            }
            ResponseBody::ParameterValue(pv) => match pv {
                ParameterValue::RunMode(x) => self.run_mode = *x,
                ParameterValue::AngleRef(x) => {
                    self.angle = *x;
                    self.angle_string = x.to_string();
                }
                ParameterValue::SpeedRef(x) => {
                    self.velocity = *x;
                    self.velocity_string = x.to_string();
                }
                ParameterValue::TorqueRef(x) => {
                    self.torque = *x;
                    self.torque_string = x.to_string();
                }
                ParameterValue::AngleKp(x) => {
                    self.angle_kp = *x;
                    self.angle_kp_string = x.to_string();
                }
                ParameterValue::SpeedKp(x) => {
                    self.speed_kp = *x;
                    self.speed_kp_string = x.to_string();
                }
                ParameterValue::SpeedKi(x) => {
                    self.speed_ki = *x;
                    self.speed_ki_string = x.to_string();
                }
                ParameterValue::CurrentKp(x) => {
                    self.iq_kp = *x;
                    self.iq_kp_string = x.to_string();
                }
                ParameterValue::CurrentKi(x) => {
                    self.iq_ki = *x;
                    self.iq_ki_string = x.to_string();
                }
                ParameterValue::Spring(x) => {
                    self.spring = *x;
                    self.spring_string = x.to_string();
                }
                ParameterValue::Damping(x) => {
                    self.damping = *x;
                    self.damping_string = x.to_string();
                }
                ParameterValue::VqRef(x) => {
                    self.vq = *x;
                    self.vq_string = x.to_string();
                }
                _ => (),
            },
        }
    }

    fn start_capture(&mut self, sequence: ActiveSequence) {
        self.active_capture = Some(SequenceCapture {
            sequence,
            rows: Vec::new(),
        });
        self.last_capture = None;
    }

    fn finish_capture(&mut self) {
        if let Some(capture) = self.active_capture.take() {
            self.last_capture = Some(capture);
        }
    }

    fn record_sequence_command(&mut self, timestamp_ms: u128, target: ChirpTarget, value: f32) {
        if let Some(capture) = &mut self.active_capture {
            capture.rows.push(SequenceCaptureRow::Command {
                timestamp_ms,
                target,
                value,
            });
        }
    }

    fn record_feedback(&mut self, response: &ResponseBody, timestamp_ms: u128) {
        let Some(capture) = &mut self.active_capture else {
            return;
        };

        let row = match response {
            ResponseBody::MotorStatus(x) => SequenceCaptureRow::Status {
                timestamp_ms,
                angle: x.angle,
                velocity: x.velocity,
                torque: x.torque,
                temperature: x.temperature,
            },
            ResponseBody::MotorCurrent(x) => SequenceCaptureRow::Current {
                timestamp_ms,
                i_q: x.i_q,
                i_d: x.i_d,
            },
            ResponseBody::DebugValue(x) => SequenceCaptureRow::DebugValue {
                timestamp_ms,
                kind: x.kind,
                value: x.value,
            },
            ResponseBody::ParameterValue(_) => return,
        };
        capture.rows.push(row);
    }

    fn has_sequence_capture(&self) -> bool {
        self.active_capture
            .as_ref()
            .is_some_and(|capture| !capture.rows.is_empty())
            || self
                .last_capture
                .as_ref()
                .is_some_and(|capture| !capture.rows.is_empty())
    }

    fn sequence_capture(&self) -> Option<SequenceCapture> {
        self.active_capture
            .as_ref()
            .or(self.last_capture.as_ref())
            .cloned()
    }

    fn has_torque_calibration(&self) -> bool {
        self.active_torque_calibration
            .as_ref()
            .is_some_and(|calibration| !calibration.rows.is_empty())
            || self
                .last_torque_calibration
                .as_ref()
                .is_some_and(|calibration| !calibration.rows.is_empty())
    }

    fn torque_calibration(&self) -> Option<TorqueCalibrationResult> {
        if let Some(calibration) = &self.active_torque_calibration {
            return Some(torque_calibration_result(calibration));
        }
        self.last_torque_calibration.clone()
    }

    fn set_sent_reference(&mut self, target: ChirpTarget, value: f32) {
        match target {
            ChirpTarget::Angle => self.sent_angle_ref = value,
            ChirpTarget::Velocity => self.sent_velocity_ref = value,
            ChirpTarget::Torque => self.sent_torque_ref = value,
        }
    }
}

impl MyApp {
    fn new(
        command_send: Sender<CommandMessage>,
        command_event_recv: Receiver<CommandEvent>,
        status_recv: Receiver<ResponseMessage>,
        sequence_control_send: Sender<SequenceControl>,
        sequence_event_recv: Receiver<SequenceEvent>,
        loadcell_recv: Receiver<LoadCellEvent>,
    ) -> Self {
        let app = Self {
            command_send,
            command_event_recv,
            status_recv,
            sequence_control_send,
            sequence_event_recv,
            loadcell_recv,
            latest_loadcell_grams: None,
            loadcell_sequence: 0,
            loadcell_status: "waiting for loadcell".to_owned(),
            tabs: vec![MotorTab::new(DEFAULT_MOTOR_CAN_ID)],
            selected_tab: 0,
            new_tab_can_id_string: DEFAULT_MOTOR_CAN_ID.to_string(),
            file_dialog: egui_file_dialog::FileDialog::new(),
            pending_export: None,
        };
        app.request_initial_parameters(DEFAULT_MOTOR_CAN_ID);
        app
    }

    fn selected_tab(&self) -> Option<&MotorTab> {
        self.tabs.get(self.selected_tab)
    }

    fn selected_tab_mut(&mut self) -> Option<&mut MotorTab> {
        self.tabs.get_mut(self.selected_tab)
    }

    fn has_tab_for_can_id(&self, can_id: u8) -> bool {
        self.tabs.iter().any(|tab| tab.can_id == can_id)
    }

    fn send_command_to(&self, can_id: u8, command: Command) {
        let _ = self.command_send.send(CommandMessage {
            host_can_id: HOST_CAN_ID,
            motor_can_id: can_id,
            command,
        });
    }

    fn request_initial_parameters(&self, can_id: u8) {
        self.send_command_to(can_id, Command::GetParameter(ParameterIndex::RunMode));
        sleep(Duration::from_millis(1));
        self.send_command_to(can_id, Command::GetParameter(ParameterIndex::AngleRef));
        sleep(Duration::from_millis(1));
        self.send_command_to(can_id, Command::GetParameter(ParameterIndex::SpeedRef));
        sleep(Duration::from_millis(1));
        self.send_command_to(can_id, Command::GetParameter(ParameterIndex::TorqueRef));
        sleep(Duration::from_millis(1));
        self.send_command_to(can_id, Command::GetParameter(ParameterIndex::VqRef));
        sleep(Duration::from_millis(1));
        self.send_command_to(can_id, Command::GetParameter(ParameterIndex::AngleKp));
        sleep(Duration::from_millis(1));
        self.send_command_to(can_id, Command::GetParameter(ParameterIndex::SpeedKp));
        sleep(Duration::from_millis(1));
        self.send_command_to(can_id, Command::GetParameter(ParameterIndex::SpeedKi));
        sleep(Duration::from_millis(1));
        self.send_command_to(can_id, Command::GetParameter(ParameterIndex::CurrentKp));
        sleep(Duration::from_millis(1));
        self.send_command_to(can_id, Command::GetParameter(ParameterIndex::CurrentKi));
        sleep(Duration::from_millis(1));
        self.send_command_to(can_id, Command::GetParameter(ParameterIndex::Spring));
        sleep(Duration::from_millis(1));
        self.send_command_to(can_id, Command::GetParameter(ParameterIndex::Damping));
        sleep(Duration::from_millis(1));
        self.send_command_to(can_id, Command::GetParameter(ParameterIndex::SpeedLimit));
        sleep(Duration::from_millis(1));
        self.send_command_to(can_id, Command::GetParameter(ParameterIndex::TorqueLimit));
        sleep(Duration::from_millis(1));
        self.send_command_to(can_id, Command::GetParameter(ParameterIndex::CurrentLimit));
        sleep(Duration::from_millis(1));
    }

    fn add_tab(&mut self, can_id: u8) {
        if self.has_tab_for_can_id(can_id) {
            if let Some(index) = self.tabs.iter().position(|tab| tab.can_id == can_id) {
                self.selected_tab = index;
            }
            return;
        }
        self.tabs.push(MotorTab::new(can_id));
        self.selected_tab = self.tabs.len() - 1;
        self.request_initial_parameters(can_id);
    }

    fn remove_selected_tab(&mut self) {
        if self.tabs.len() <= 1 {
            return;
        }
        if let Some(tab) = self.tabs.get(self.selected_tab)
            && tab.active_sequence.is_some()
        {
            self.stop_sequence(tab.can_id);
        }
        self.tabs.remove(self.selected_tab);
        if self.selected_tab >= self.tabs.len() {
            self.selected_tab = self.tabs.len() - 1;
        }
    }

    fn rename_selected_motor(&mut self, new_can_id: u8) {
        let Some(current_can_id) = self.selected_tab().map(|tab| tab.can_id) else {
            return;
        };
        if new_can_id != current_can_id && self.has_tab_for_can_id(new_can_id) {
            return;
        }
        if self
            .selected_tab()
            .is_some_and(|tab| tab.active_sequence.is_some())
        {
            self.stop_sequence(current_can_id);
        }

        self.send_command_to(current_can_id, Command::SetCanId(new_can_id));
        if let Some(tab) = self.selected_tab_mut() {
            tab.can_id = new_can_id;
            tab.rename_can_id_string = new_can_id.to_string();
            tab.clear_plot();
            tab.active_sequence = None;
        }
        self.request_initial_parameters(new_can_id);
    }

    fn handle_incoming_messages(&mut self) {
        for message in self.status_recv.try_iter() {
            if let Some(tab) = self
                .tabs
                .iter_mut()
                .find(|tab| tab.can_id == message.motor_can_id)
            {
                if tab.active_capture.is_some() {
                    tab.record_feedback(&message.body, current_timestamp_ms());
                }
                tab.apply_response(&message.body);
            }
        }
    }

    fn handle_command_events(&mut self) {
        for event in self.command_event_recv.try_iter() {
            match event {
                CommandEvent::RefSent {
                    can_id,
                    target,
                    value,
                } => {
                    if let Some(tab) = self.tabs.iter_mut().find(|tab| tab.can_id == can_id) {
                        tab.set_sent_reference(target, value);
                    }
                }
                CommandEvent::MotionSent {
                    can_id,
                    angle,
                    velocity,
                    torque,
                } => {
                    if let Some(tab) = self.tabs.iter_mut().find(|tab| tab.can_id == can_id) {
                        tab.sent_angle_ref = angle;
                        tab.sent_velocity_ref = velocity;
                        tab.sent_torque_ref = torque;
                    }
                }
            }
        }
    }

    fn handle_sequence_events(&mut self) {
        for event in self.sequence_event_recv.try_iter() {
            match event {
                SequenceEvent::Updated {
                    can_id,
                    target,
                    value,
                } => {
                    if let Some(tab) = self.tabs.iter_mut().find(|tab| tab.can_id == can_id) {
                        tab.set_sequence_value(target, value);
                        tab.record_sequence_command(current_timestamp_ms(), target, value);
                    }
                }
                SequenceEvent::Finished { can_id } => {
                    if let Some(tab) = self.tabs.iter_mut().find(|tab| tab.can_id == can_id) {
                        tab.active_sequence = None;
                        tab.finish_capture();
                    }
                }
            }
        }
    }

    fn handle_loadcell_events(&mut self) {
        for event in self.loadcell_recv.try_iter() {
            match event {
                LoadCellEvent::Reading { grams } => {
                    self.latest_loadcell_grams = Some(grams);
                    self.loadcell_sequence = self.loadcell_sequence.wrapping_add(1);
                    self.loadcell_status = format!("{grams:.2} g");
                }
                LoadCellEvent::Error(message) => {
                    self.loadcell_status = message;
                }
            }
        }
    }

    fn tick_torque_calibrations(&mut self) {
        let now = Instant::now();
        let latest_loadcell_grams = self.latest_loadcell_grams;
        let loadcell_sequence = self.loadcell_sequence;
        let mut commands = Vec::new();

        for tab in &mut self.tabs {
            let Some(calibration) = &mut tab.active_torque_calibration else {
                continue;
            };

            match calibration.phase {
                TorqueCalibrationPhase::TareCommand => {
                    commands.push((
                        tab.can_id,
                        Command::SetParameter(ParameterValue::CurrentRef(0.0)),
                    ));
                    calibration.phase = TorqueCalibrationPhase::TareSettling {
                        until: now + calibration.min_settle_duration,
                    };
                }
                TorqueCalibrationPhase::TareSettling { until } => {
                    if now >= until {
                        calibration.samples.clear();
                        calibration.last_loadcell_sequence = loadcell_sequence;
                        calibration.phase = TorqueCalibrationPhase::TareSampling;
                    }
                }
                TorqueCalibrationPhase::TareSampling => {
                    if calibration.last_loadcell_sequence != loadcell_sequence
                        && let Some(grams) = latest_loadcell_grams
                    {
                        calibration.samples.push(grams);
                        calibration.last_loadcell_sequence = loadcell_sequence;
                    }

                    if calibration.samples.len() >= calibration.sample_count {
                        calibration.tare_g = mean(&calibration.samples);
                        calibration.samples.clear();
                        calibration.phase = TorqueCalibrationPhase::ReleaseCommand;
                    }
                }
                TorqueCalibrationPhase::ReleaseCommand => {
                    commands.push((
                        tab.can_id,
                        Command::SetParameter(ParameterValue::CurrentRef(0.0)),
                    ));
                    commands.push((
                        tab.can_id,
                        Command::SetParameter(ParameterValue::RunMode(RunMode::Angle)),
                    ));
                    calibration.release_start_angle = tab.angle;
                    calibration.release_target_angle = tab.angle;
                    calibration.release_direction = 1.0;
                    calibration.release_last_abs_g = latest_loadcell_grams.map(f32::abs);
                    calibration.phase = TorqueCalibrationPhase::WaitingForRelease {
                        timeout_at: now + calibration.release_timeout,
                        last_step_at: now,
                    };
                }
                TorqueCalibrationPhase::WaitingForRelease {
                    timeout_at,
                    last_step_at,
                } => {
                    let released = latest_loadcell_grams
                        .map(|grams| grams.abs() <= calibration.release_threshold_g)
                        .unwrap_or(false);
                    if released || now >= timeout_at {
                        calibration.phase = TorqueCalibrationPhase::ApproachCommand;
                    } else if now.saturating_duration_since(last_step_at)
                        >= calibration.min_settle_duration
                    {
                        if let Some(abs_g) = latest_loadcell_grams.map(f32::abs) {
                            if let Some(last_abs_g) = calibration.release_last_abs_g
                                && abs_g > last_abs_g
                            {
                                calibration.release_direction = -calibration.release_direction;
                                calibration.release_target_angle = calibration.release_start_angle;
                            }
                            calibration.release_last_abs_g = Some(abs_g);
                        }
                        let next = calibration.release_target_angle
                            + calibration.release_direction * calibration.release_step_rad.abs();
                        let offset = next - calibration.release_start_angle;
                        if offset.abs() > calibration.release_max_rad.abs() {
                            calibration.release_direction = -calibration.release_direction;
                            calibration.release_target_angle = calibration.release_start_angle;
                        } else {
                            calibration.release_target_angle = next;
                        }
                        commands.push((
                            tab.can_id,
                            Command::SetParameter(ParameterValue::AngleRef(
                                calibration.release_target_angle,
                            )),
                        ));
                        calibration.phase = TorqueCalibrationPhase::WaitingForRelease {
                            timeout_at,
                            last_step_at: now,
                        };
                    }
                }
                TorqueCalibrationPhase::ApproachCommand => {
                    calibration.approach_target_angle = calibration.release_target_angle;
                    calibration.phase = TorqueCalibrationPhase::WaitingForContact {
                        timeout_at: now + calibration.release_timeout,
                        last_step_at: now,
                    };
                }
                TorqueCalibrationPhase::WaitingForContact {
                    timeout_at,
                    last_step_at,
                } => {
                    let contacted = latest_loadcell_grams
                        .map(|grams| grams.abs() >= calibration.contact_threshold_g)
                        .unwrap_or(false);
                    if contacted || now >= timeout_at {
                        calibration.phase = TorqueCalibrationPhase::CommandStep;
                    } else if now.saturating_duration_since(last_step_at)
                        >= calibration.min_settle_duration
                    {
                        let next = calibration.approach_target_angle
                            - calibration.release_direction * calibration.release_step_rad.abs();
                        let offset = next - calibration.release_start_angle;
                        if offset.abs() <= calibration.release_max_rad.abs() {
                            calibration.approach_target_angle = next;
                            commands.push((
                                tab.can_id,
                                Command::SetParameter(ParameterValue::AngleRef(
                                    calibration.approach_target_angle,
                                )),
                            ));
                        }
                        calibration.phase = TorqueCalibrationPhase::WaitingForContact {
                            timeout_at,
                            last_step_at: now,
                        };
                    }
                }
                TorqueCalibrationPhase::CommandStep => {
                    if let Some(step) = calibration.steps.get(calibration.step_index).copied() {
                        commands.push((
                            tab.can_id,
                            Command::SetParameter(ParameterValue::RunMode(RunMode::Torque)),
                        ));
                        commands.push((
                            tab.can_id,
                            Command::SetParameter(ParameterValue::CurrentRef(step.iq_ref)),
                        ));
                        calibration.stability_window.clear();
                        calibration.phase = TorqueCalibrationPhase::Settling {
                            min_until: now + calibration.min_settle_duration,
                            timeout_at: now + calibration.stability_timeout,
                        };
                    } else {
                        commands.push((
                            tab.can_id,
                            Command::SetParameter(ParameterValue::CurrentRef(0.0)),
                        ));
                        commands.push((tab.can_id, Command::Stop));
                        tab.last_torque_calibration = Some(torque_calibration_result(calibration));
                        tab.active_torque_calibration = None;
                    }
                }
                TorqueCalibrationPhase::Settling {
                    min_until,
                    timeout_at,
                } => {
                    if calibration.last_loadcell_sequence != loadcell_sequence
                        && let Some(grams) = latest_loadcell_grams
                    {
                        push_sample(&mut calibration.stability_window, grams);
                        calibration.last_loadcell_sequence = loadcell_sequence;
                    }

                    let stable = calibration.stability_window.len()
                        >= calibration.stability_samples
                        && stddev_deque(&calibration.stability_window)
                            <= calibration.stability_std_g;
                    if now >= min_until && (stable || now >= timeout_at) {
                        calibration.samples.clear();
                        calibration.last_loadcell_sequence = loadcell_sequence;
                        calibration.phase = TorqueCalibrationPhase::Sampling;
                    }
                }
                TorqueCalibrationPhase::Sampling => {
                    if calibration.last_loadcell_sequence != loadcell_sequence
                        && let Some(grams) = latest_loadcell_grams
                    {
                        calibration.samples.push(grams);
                        calibration.last_loadcell_sequence = loadcell_sequence;
                    }

                    if calibration.samples.len() >= calibration.sample_count {
                        let step = calibration.steps[calibration.step_index];
                        let filtered =
                            reject_outliers(&calibration.samples, calibration.outlier_sigma);
                        let raw_mean = mean(&calibration.samples);
                        let corrected_mean = mean(&filtered) - calibration.tare_g;
                        let std = stddev(&filtered, mean(&filtered));
                        calibration.rows.push(TorqueCalibrationRow {
                            iq_ref: step.iq_ref,
                            repeat: step.repeat,
                            direction: step.direction,
                            raw_mean_g: raw_mean,
                            tare_corrected_mean_g: corrected_mean,
                            std_g: std,
                            rejected_samples: calibration
                                .samples
                                .len()
                                .saturating_sub(filtered.len()),
                            sample_count: filtered.len(),
                            torque_nm: grams_to_torque_nm(corrected_mean, calibration.arm_mm),
                        });
                        calibration.step_index += 1;
                        calibration.phase = TorqueCalibrationPhase::ReleaseCommand;
                    }
                }
            }
        }

        for (can_id, command) in commands {
            self.send_command_to(can_id, command);
        }
    }

    fn start_torque_calibration(&mut self, can_id: u8) {
        let Some(tab) = self.tabs.iter_mut().find(|tab| tab.can_id == can_id) else {
            return;
        };
        let Some(steps) = torque_calibration_steps(tab) else {
            return;
        };
        let release_angle = tab.angle;

        tab.last_torque_calibration = None;
        tab.active_torque_calibration = Some(TorqueCalibration {
            steps,
            step_index: 0,
            min_settle_duration: Duration::from_millis(tab.cal_settle_ms as u64),
            sample_count: tab.cal_sample_count,
            arm_mm: tab.cal_arm_mm,
            tare_g: 0.0,
            stability_std_g: tab.cal_stability_std_g,
            stability_samples: tab.cal_stability_samples,
            stability_timeout: Duration::from_millis(tab.cal_stability_timeout_ms as u64),
            outlier_sigma: tab.cal_outlier_sigma,
            release_threshold_g: tab.cal_release_threshold_g,
            release_timeout: Duration::from_millis(tab.cal_release_timeout_ms as u64),
            release_step_rad: tab.cal_release_step_rad,
            release_max_rad: tab.cal_release_max_rad,
            contact_threshold_g: tab.cal_contact_threshold_g,
            release_start_angle: release_angle,
            release_target_angle: release_angle,
            release_direction: 1.0,
            release_last_abs_g: None,
            approach_target_angle: release_angle,
            phase: TorqueCalibrationPhase::TareCommand,
            rows: Vec::new(),
            samples: Vec::new(),
            stability_window: VecDeque::with_capacity(tab.cal_stability_samples),
            last_loadcell_sequence: self.loadcell_sequence,
        });

        self.send_command_to(
            can_id,
            Command::SetParameter(ParameterValue::RunMode(RunMode::Angle)),
        );
        self.send_command_to(
            can_id,
            Command::SetParameter(ParameterValue::AngleRef(release_angle)),
        );
        self.send_command_to(
            can_id,
            Command::SetParameter(ParameterValue::CurrentRef(0.0)),
        );
        self.send_command_to(can_id, Command::Enable);
    }

    fn stop_torque_calibration(&mut self, can_id: u8) {
        if let Some(tab) = self.tabs.iter_mut().find(|tab| tab.can_id == can_id) {
            if let Some(calibration) = tab.active_torque_calibration.take() {
                tab.last_torque_calibration = Some(torque_calibration_result(&calibration));
            }
        }
        self.send_command_to(
            can_id,
            Command::SetParameter(ParameterValue::CurrentRef(0.0)),
        );
        self.send_command_to(can_id, Command::Stop);
    }

    fn start_sequence(&self, can_id: u8, sequence: ActiveSequence) {
        let _ = self
            .sequence_control_send
            .send(SequenceControl::Start { can_id, sequence });
    }

    fn stop_sequence(&self, can_id: u8) {
        let _ = self
            .sequence_control_send
            .send(SequenceControl::Stop { can_id });
    }
}

impl eframe::App for MyApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.handle_command_events();
        self.handle_incoming_messages();
        self.handle_sequence_events();
        self.handle_loadcell_events();
        self.tick_torque_calibrations();
        self.render_ui(ui);
        ui.ctx().request_repaint_after(UI_REPAINT_INTERVAL);
    }
}

impl MyApp {
    fn render_ui(&mut self, parent_ui: &mut egui::Ui) {
        egui::CentralPanel::default().show_inside(parent_ui, |ui| {
            ui.heading("BLDC Monitor");

            ui.horizontal(|ui| {
                ui.label("Motor tabs");
                let name_label = ui.label("CAN ID");
                ui.text_edit_singleline(&mut self.new_tab_can_id_string)
                    .labelled_by(name_label.id);
                let parsed_new_tab_id = self.new_tab_can_id_string.parse::<u8>().ok();
                let can_add_tab = parsed_new_tab_id
                    .map(|can_id| !self.has_tab_for_can_id(can_id))
                    .unwrap_or(false);
                if ui
                    .add_enabled(can_add_tab, egui::Button::new("Add tab"))
                    .clicked()
                    && let Some(can_id) = parsed_new_tab_id
                {
                    self.add_tab(can_id);
                }
                if ui
                    .add_enabled(self.tabs.len() > 1, egui::Button::new("Remove tab"))
                    .clicked()
                {
                    self.remove_selected_tab();
                }
            });

            ui.horizontal_wrapped(|ui| {
                for (index, tab) in self.tabs.iter().enumerate() {
                    let label = format!("0x{:02X}", tab.can_id);
                    if ui
                        .selectable_label(index == self.selected_tab, label)
                        .clicked()
                    {
                        self.selected_tab = index;
                    }
                }
            });

            ui.separator();

            let Some(selected_index) = self.tabs.get(self.selected_tab).map(|_| self.selected_tab)
            else {
                return;
            };
            let current_can_id = self.tabs[selected_index].can_id;

            let other_can_ids: Vec<u8> = self
                .tabs
                .iter()
                .enumerate()
                .filter_map(|(index, tab)| (index != selected_index).then_some(tab.can_id))
                .collect();

            let mut queued_commands: Vec<Command> = Vec::new();
            let mut request_initial_parameters = false;
            let mut rename_to: Option<u8> = None;
            let mut export_requested = false;
            let mut sequence_export_requested = false;
            let mut calibration_export_requested = false;
            let mut sequence_request: Option<SequenceUiRequest> = None;
            let mut start_calibration = false;
            let mut stop_calibration = false;

            {
                let tab = &mut self.tabs[selected_index];

                ui.horizontal(|ui| {
                    ui.label(format!("Active motor: 0x{:02X}", tab.can_id));
                    let name_label = ui.label("Rename to CAN ID");
                    ui.text_edit_singleline(&mut tab.rename_can_id_string)
                        .labelled_by(name_label.id);
                    let parsed_rename_id = tab.rename_can_id_string.parse::<u8>().ok();
                    let can_rename = parsed_rename_id
                        .map(|new_id| new_id == tab.can_id || !other_can_ids.contains(&new_id))
                        .unwrap_or(false);
                    if ui
                        .add_enabled(can_rename, egui::Button::new("Set on motor"))
                        .clicked()
                    {
                        rename_to = parsed_rename_id;
                    }
                });

                let old_mode = tab.run_mode;
                ui.label("Run mode");
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut tab.run_mode, RunMode::Impedance, "Impedance");
                    ui.selectable_value(&mut tab.run_mode, RunMode::Angle, "Angle");
                    ui.selectable_value(&mut tab.run_mode, RunMode::Velocity, "Velocity");
                    ui.selectable_value(&mut tab.run_mode, RunMode::Torque, "Torque");
                    ui.selectable_value(&mut tab.run_mode, RunMode::Voltage, "Voltage");
                });
                if old_mode != tab.run_mode {
                    queued_commands
                        .push(Command::SetParameter(ParameterValue::RunMode(tab.run_mode)));
                }

                ui.separator();
                ui.label("Feedback");
                ui.horizontal(|ui| {
                    if ui.button("Start feedback").clicked() {
                        queued_commands.push(Command::SetFeedbackType(tab.current_feedback_type()));
                        queued_commands.push(Command::RequestStatus(HOST_CAN_ID));
                    }

                    if ui.button("Send status request").clicked() {
                        queued_commands.push(Command::RequestStatus(HOST_CAN_ID));
                    }

                    if ui.button("Refresh parameters").clicked() {
                        request_initial_parameters = true;
                    }

                    if ui.button("Apply plot type").clicked() {
                        queued_commands.push(Command::SetFeedbackType(tab.current_feedback_type()));
                    }
                });

                ui.horizontal(|ui| {
                    let name_label = ui.label("Feedback interval");
                    ui.text_edit_singleline(&mut tab.feedback_interval_string)
                        .labelled_by(name_label.id);

                    let parsed_interval = tab.feedback_interval_string.parse::<u8>().ok();
                    if ui
                        .add_enabled(parsed_interval.is_some(), egui::Button::new("Set interval"))
                        .clicked()
                        && let Some(period) = parsed_interval
                    {
                        queued_commands.push(Command::SetFeedbackInterval(period));
                    }

                    ui.label("raw ticks");
                });

                egui::Grid::new(format!("params-{}", tab.can_id))
                    .num_columns(3)
                    .spacing([40.0, 4.0])
                    .show(ui, |ui| {
                        param_row(
                            ui,
                            "Angle Ref",
                            &mut tab.angle,
                            &mut tab.angle_string,
                            &mut queued_commands,
                            ParameterValue::AngleRef,
                        );
                        param_row(
                            ui,
                            "Velocity Ref",
                            &mut tab.velocity,
                            &mut tab.velocity_string,
                            &mut queued_commands,
                            ParameterValue::SpeedRef,
                        );
                        param_row(
                            ui,
                            "Torque Ref [Nm]",
                            &mut tab.torque,
                            &mut tab.torque_string,
                            &mut queued_commands,
                            ParameterValue::TorqueRef,
                        );
                        param_row(
                            ui,
                            "V_q Ref",
                            &mut tab.vq,
                            &mut tab.vq_string,
                            &mut queued_commands,
                            ParameterValue::VqRef,
                        );
                        param_row(
                            ui,
                            "Angle K_p",
                            &mut tab.angle_kp,
                            &mut tab.angle_kp_string,
                            &mut queued_commands,
                            ParameterValue::AngleKp,
                        );
                        param_row(
                            ui,
                            "Speed K_p",
                            &mut tab.speed_kp,
                            &mut tab.speed_kp_string,
                            &mut queued_commands,
                            ParameterValue::SpeedKp,
                        );
                        param_row(
                            ui,
                            "Speed K_i",
                            &mut tab.speed_ki,
                            &mut tab.speed_ki_string,
                            &mut queued_commands,
                            ParameterValue::SpeedKi,
                        );
                        param_row(
                            ui,
                            "Current K_p",
                            &mut tab.iq_kp,
                            &mut tab.iq_kp_string,
                            &mut queued_commands,
                            ParameterValue::CurrentKp,
                        );
                        param_row(
                            ui,
                            "Current K_i",
                            &mut tab.iq_ki,
                            &mut tab.iq_ki_string,
                            &mut queued_commands,
                            ParameterValue::CurrentKi,
                        );
                        param_row(
                            ui,
                            "Spring",
                            &mut tab.spring,
                            &mut tab.spring_string,
                            &mut queued_commands,
                            ParameterValue::Spring,
                        );
                        param_row(
                            ui,
                            "Damping",
                            &mut tab.damping,
                            &mut tab.damping_string,
                            &mut queued_commands,
                            ParameterValue::Damping,
                        );
                    });

                ui.separator();
                ui.label("Motion Control");
                egui::Grid::new(format!("motion-{}", tab.can_id))
                    .num_columns(2)
                    .spacing([40.0, 4.0])
                    .show(ui, |ui| {
                        motion_field(
                            ui,
                            "Position [rad]",
                            &mut tab.motion_angle,
                            &mut tab.motion_angle_string,
                        );
                        motion_field(
                            ui,
                            "Velocity [rad/s]",
                            &mut tab.motion_velocity,
                            &mut tab.motion_velocity_string,
                        );
                        motion_field(
                            ui,
                            "Torque [Nm]",
                            &mut tab.motion_torque,
                            &mut tab.motion_torque_string,
                        );
                        motion_field(
                            ui,
                            "Kp [Nm/rad]",
                            &mut tab.motion_kp,
                            &mut tab.motion_kp_string,
                        );
                        motion_field(
                            ui,
                            "Kd [Nm/(rad/s)]",
                            &mut tab.motion_kd,
                            &mut tab.motion_kd_string,
                        );
                    });
                let motion_valid = [
                    tab.motion_angle,
                    tab.motion_velocity,
                    tab.motion_torque,
                    tab.motion_kp,
                    tab.motion_kd,
                ]
                .iter()
                .all(|value| value.is_finite());
                if ui
                    .add_enabled(motion_valid, egui::Button::new("Send motion"))
                    .clicked()
                {
                    queued_commands.push(Command::MotionControl(MotionControl {
                        angle: tab.motion_angle,
                        velocity: tab.motion_velocity,
                        torque: tab.motion_torque,
                        kp: tab.motion_kp,
                        kd: tab.motion_kd,
                    }));
                }

                ui.separator();
                ui.label("Current-Torque Calibration");
                ui.horizontal(|ui| {
                    ui.label("Loadcell");
                    ui.monospace(&self.loadcell_status);
                });
                egui::Grid::new(format!("torque-cal-{}", tab.can_id))
                    .num_columns(2)
                    .spacing([40.0, 4.0])
                    .show(ui, |ui| {
                        motion_field(
                            ui,
                            "Iq start",
                            &mut tab.cal_iq_start,
                            &mut tab.cal_iq_start_string,
                        );
                        motion_field(
                            ui,
                            "Iq end",
                            &mut tab.cal_iq_end,
                            &mut tab.cal_iq_end_string,
                        );
                        motion_field(
                            ui,
                            "Iq step",
                            &mut tab.cal_iq_step,
                            &mut tab.cal_iq_step_string,
                        );
                        integer_field(
                            ui,
                            "Settle [ms]",
                            &mut tab.cal_settle_ms,
                            &mut tab.cal_settle_ms_string,
                        );
                        integer_field(
                            ui,
                            "Samples",
                            &mut tab.cal_sample_count,
                            &mut tab.cal_sample_count_string,
                        );
                        motion_field(
                            ui,
                            "Arm [mm]",
                            &mut tab.cal_arm_mm,
                            &mut tab.cal_arm_mm_string,
                        );
                        integer_field(
                            ui,
                            "Repeats",
                            &mut tab.cal_repeat_count,
                            &mut tab.cal_repeat_count_string,
                        );
                        motion_field(
                            ui,
                            "Stable std [g]",
                            &mut tab.cal_stability_std_g,
                            &mut tab.cal_stability_std_g_string,
                        );
                        integer_field(
                            ui,
                            "Stable samples",
                            &mut tab.cal_stability_samples,
                            &mut tab.cal_stability_samples_string,
                        );
                        integer_field(
                            ui,
                            "Stable timeout [ms]",
                            &mut tab.cal_stability_timeout_ms,
                            &mut tab.cal_stability_timeout_ms_string,
                        );
                        motion_field(
                            ui,
                            "Outlier sigma",
                            &mut tab.cal_outlier_sigma,
                            &mut tab.cal_outlier_sigma_string,
                        );
                        motion_field(
                            ui,
                            "Release threshold [g]",
                            &mut tab.cal_release_threshold_g,
                            &mut tab.cal_release_threshold_g_string,
                        );
                        integer_field(
                            ui,
                            "Release timeout [ms]",
                            &mut tab.cal_release_timeout_ms,
                            &mut tab.cal_release_timeout_ms_string,
                        );
                        motion_field(
                            ui,
                            "Release step [rad]",
                            &mut tab.cal_release_step_rad,
                            &mut tab.cal_release_step_rad_string,
                        );
                        motion_field(
                            ui,
                            "Release max [rad]",
                            &mut tab.cal_release_max_rad,
                            &mut tab.cal_release_max_rad_string,
                        );
                        motion_field(
                            ui,
                            "Contact threshold [g]",
                            &mut tab.cal_contact_threshold_g,
                            &mut tab.cal_contact_threshold_g_string,
                        );
                    });
                let calibration_steps = torque_calibration_steps(tab);
                let calibration_active = tab.active_torque_calibration.is_some();
                let calibration_ready =
                    calibration_steps.is_some() && self.latest_loadcell_grams.is_some();
                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(
                            calibration_ready && !calibration_active,
                            egui::Button::new("Start calibration"),
                        )
                        .clicked()
                    {
                        start_calibration = true;
                    }
                    if ui
                        .add_enabled(calibration_active, egui::Button::new("Stop calibration"))
                        .clicked()
                    {
                        stop_calibration = true;
                    }
                    if let Some(steps) = &calibration_steps {
                        ui.small(format!("{} negative steps", steps.len()));
                    } else {
                        ui.small("Iq start/end/step must generate only negative points");
                    }
                });
                if let Some(calibration) = &tab.active_torque_calibration {
                    ui.small(format!(
                        "{} step {}/{} rows {} tare {:.2} g",
                        calibration.phase.label(),
                        calibration.step_index + 1,
                        calibration.steps.len(),
                        calibration.rows.len(),
                        calibration.tare_g
                    ));
                }
                if let Some(result) = tab.torque_calibration()
                    && let Some(k) = result.fit_zero_slope_nm_per_iq
                {
                    ui.small(format!(
                        "fit: T = {:.4} * |Iq| Nm, RMSE {:.4} Nm",
                        k,
                        result.fit_zero_rmse_nm.unwrap_or(0.0)
                    ));
                }
                let calibration_export_label = if tab.active_torque_calibration.is_some() {
                    "Export active calibration"
                } else {
                    "Export last calibration"
                };
                if ui
                    .add_enabled(
                        tab.has_torque_calibration(),
                        egui::Button::new(calibration_export_label),
                    )
                    .clicked()
                {
                    calibration_export_requested = true;
                }

                ui.horizontal(|ui| {
                    if ui.button("disable").clicked() {
                        if tab.active_sequence.is_some() {
                            sequence_request = Some(SequenceUiRequest::Stop);
                            tab.active_sequence = None;
                        }
                        queued_commands.push(Command::Stop);
                    }
                    if ui.button("enable").clicked() {
                        queued_commands.push(Command::Enable);
                    }
                });

                ui.separator();
                ui.label("Chirp");
                ui.add_enabled_ui(!tab.step_active(), |ui| {
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut tab.chirp_target, ChirpTarget::Angle, "Angle");
                        ui.selectable_value(
                            &mut tab.chirp_target,
                            ChirpTarget::Velocity,
                            "Velocity",
                        );
                        ui.selectable_value(&mut tab.chirp_target, ChirpTarget::Torque, "Torque");
                    });
                    ui.horizontal(|ui| {
                        let amplitude_label = ui.label("Amplitude");
                        ui.text_edit_singleline(&mut tab.chirp_amplitude_string)
                            .labelled_by(amplitude_label.id);
                        let amplitude_valid = tab
                            .chirp_amplitude_string
                            .parse::<f32>()
                            .map(|value| {
                                tab.chirp_amplitude = value;
                                value.is_finite() && value > 0.0
                            })
                            .unwrap_or(false);

                        let duration_label = ui.label("Duration [s]");
                        ui.text_edit_singleline(&mut tab.chirp_duration_string)
                            .labelled_by(duration_label.id);
                        let duration_valid = tab
                            .chirp_duration_string
                            .parse::<f32>()
                            .map(|value| {
                                tab.chirp_duration = value;
                                value.is_finite() && value > 0.0
                            })
                            .unwrap_or(false);

                        let can_start_chirp =
                            amplitude_valid && duration_valid && !tab.excitation_active();
                        if ui
                            .add_enabled(can_start_chirp, egui::Button::new("Start chirp"))
                            .clicked()
                        {
                            let sequence = ActiveSequence::Chirp {
                                target: tab.chirp_target,
                                amplitude: tab.chirp_amplitude,
                                duration: Duration::from_secs_f32(tab.chirp_duration),
                            };
                            tab.run_mode = sequence.run_mode();
                            tab.set_sequence_value(tab.chirp_target, 0.0);
                            tab.active_sequence = Some(sequence);
                            tab.start_capture(sequence);
                            sequence_request = Some(SequenceUiRequest::Start(sequence));
                        }

                        if ui
                            .add_enabled(tab.chirp_active(), egui::Button::new("Stop chirp"))
                            .clicked()
                        {
                            let target = tab.active_sequence_target().unwrap_or(tab.chirp_target);
                            tab.active_sequence = None;
                            tab.set_sequence_value(target, 0.0);
                            sequence_request = Some(SequenceUiRequest::Stop);
                        }
                    });
                });
                ui.small(format!(
                    "{:.1} -> {:.1} Hz logarithmic sweep on {}",
                    CHIRP_START_FREQ_HZ,
                    CHIRP_END_FREQ_HZ,
                    tab.chirp_target.label()
                ));

                ui.separator();
                ui.label("Step");
                ui.add_enabled_ui(!tab.chirp_active(), |ui| {
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut tab.step_target, ChirpTarget::Angle, "Angle");
                        ui.selectable_value(
                            &mut tab.step_target,
                            ChirpTarget::Velocity,
                            "Velocity",
                        );
                        ui.selectable_value(&mut tab.step_target, ChirpTarget::Torque, "Torque");
                    });
                    ui.horizontal(|ui| {
                        let value_label = ui.label("Value");
                        ui.text_edit_singleline(&mut tab.step_value_string)
                            .labelled_by(value_label.id);
                        let step_valid = tab
                            .step_value_string
                            .parse::<f32>()
                            .map(|value| {
                                tab.step_value = value;
                                value.is_finite()
                            })
                            .unwrap_or(false);

                        if ui
                            .add_enabled(
                                step_valid && !tab.excitation_active(),
                                egui::Button::new("Send step"),
                            )
                            .clicked()
                        {
                            let sequence = ActiveSequence::Step {
                                target: tab.step_target,
                                value: tab.step_value,
                            };
                            tab.run_mode = sequence.run_mode();
                            tab.set_sequence_value(tab.step_target, tab.step_value);
                            tab.active_sequence = Some(sequence);
                            tab.start_capture(sequence);
                            sequence_request = Some(SequenceUiRequest::Start(sequence));
                        }

                        if ui
                            .add_enabled(tab.step_active(), egui::Button::new("Clear step"))
                            .clicked()
                        {
                            let target = tab.active_sequence_target().unwrap_or(tab.step_target);
                            tab.set_sequence_value(target, 0.0);
                            tab.active_sequence = None;
                            sequence_request = Some(SequenceUiRequest::Stop);
                        }
                    });
                });
                ui.small(format!("Send a held step on {}", tab.step_target.label()));

                ui.separator();
                ui.label("Maintenance");
                ui.horizontal(|ui| {
                    if ui.button("Set zero").clicked() {
                        queued_commands.push(Command::SetZeroPosition);
                    }
                    ui.small("Use current primary/secondary encoder angles as zero");
                });
                ui.horizontal(|ui| {
                    if ui.button("Save settings").clicked() {
                        queued_commands.push(Command::SaveParameters);
                    }
                    ui.small("Persist current motor settings to flash");
                });
                ui.horizontal(|ui| {
                    if ui.button("Retune motor").clicked() {
                        queued_commands.push(Command::RunMotorTuning);
                    }
                    ui.small("Clear stored calibration and reboot into tuning");
                });

                let before = tab.plot_type;
                ui.label("Plot Type");
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut tab.plot_type, PlotType::Angle, "Angle");
                    ui.selectable_value(&mut tab.plot_type, PlotType::Velocity, "Velocity");
                    ui.selectable_value(&mut tab.plot_type, PlotType::Torque, "Torque");
                    ui.selectable_value(&mut tab.plot_type, PlotType::Current, "Current");
                    ui.selectable_value(&mut tab.plot_type, PlotType::SpeedError, "Speed Error");
                    ui.selectable_value(&mut tab.plot_type, PlotType::TorqueRef, "Torque Ref");
                    ui.selectable_value(
                        &mut tab.plot_type,
                        PlotType::VelocityIntegral,
                        "Speed Int",
                    );
                });
                if before != tab.plot_type {
                    queued_commands.push(Command::SetFeedbackType(tab.current_feedback_type()));
                    tab.clear_plot();
                }
                if ui
                    .button(if tab.is_plotting { "Stop plot" } else { "Plot" })
                    .clicked()
                {
                    if !tab.is_plotting {
                        tab.clear_plot();
                    }
                    tab.is_plotting = !tab.is_plotting;
                }

                let export_txt = format!("Export {} samples", tab.plot_points_1.len());
                if ui
                    .add_enabled(!tab.plot_points_1.is_empty(), egui::Button::new(export_txt))
                    .clicked()
                {
                    export_requested = true;
                }

                let sequence_export_label = if tab.active_capture.is_some() {
                    "Export active test capture"
                } else {
                    "Export last test capture"
                };
                if ui
                    .add_enabled(
                        tab.has_sequence_capture(),
                        egui::Button::new(sequence_export_label),
                    )
                    .clicked()
                {
                    sequence_export_requested = true;
                }

                let points_1: Vec<_> = tab.plot_points_1.iter().copied().collect();
                let points_2: Vec<_> = tab.plot_points_2.iter().copied().collect();
                let (line_1_name, line_2_name) = tab.plot_type.plot_line_names();
                egui_plot::Plot::new(format!("plot-{}", tab.can_id)).show(ui, |plot_ui| {
                    let line_1 =
                        egui_plot::Line::new("y1", egui_plot::PlotPoints::Owned(points_1.clone()))
                            .name(line_1_name)
                            .style(egui_plot::LineStyle::Solid)
                            .color(egui::Color32::RED)
                            .width(2.0);
                    plot_ui.line(line_1);

                    if let Some(line_2_name) = line_2_name {
                        let line_2 = egui_plot::Line::new(
                            "y2",
                            egui_plot::PlotPoints::Owned(points_2.clone()),
                        )
                        .name(line_2_name)
                        .style(egui_plot::LineStyle::Solid)
                        .color(egui::Color32::GREEN)
                        .width(2.0);
                        plot_ui.line(line_2);
                    }
                });
            }

            for command in queued_commands {
                self.send_command_to(current_can_id, command);
            }

            if request_initial_parameters {
                self.request_initial_parameters(current_can_id);
            }

            if let Some(new_can_id) = rename_to {
                self.rename_selected_motor(new_can_id);
            }

            if let Some(sequence_request) = sequence_request {
                match sequence_request {
                    SequenceUiRequest::Start(sequence) => {
                        self.start_sequence(current_can_id, sequence)
                    }
                    SequenceUiRequest::Stop => self.stop_sequence(current_can_id),
                }
            }

            if start_calibration {
                self.start_torque_calibration(current_can_id);
            }

            if stop_calibration {
                self.stop_torque_calibration(current_can_id);
            }

            if export_requested {
                self.pending_export = self
                    .selected_tab()
                    .map(|tab| PendingExport::Plot(tab.plot_points_1.iter().copied().collect()));
                self.file_dialog = egui_file_dialog::FileDialog::new();
                self.file_dialog.save_file();
            }

            if sequence_export_requested {
                self.pending_export = self
                    .selected_tab()
                    .and_then(|tab| tab.sequence_capture().map(PendingExport::Sequence));
                self.file_dialog = egui_file_dialog::FileDialog::new();
                self.file_dialog.save_file();
            }

            if calibration_export_requested {
                self.pending_export = self.selected_tab().and_then(|tab| {
                    tab.torque_calibration()
                        .map(PendingExport::TorqueCalibration)
                });
                self.file_dialog = egui_file_dialog::FileDialog::new();
                self.file_dialog.save_file();
            }

            let picked_path = self
                .file_dialog
                .update(ui.ctx())
                .picked()
                .map(|path| path.to_path_buf());
            if let Some(path) = picked_path
                && let Some(export) = self.pending_export.take()
                && let Ok(mut file) = File::create(path)
            {
                match export {
                    PendingExport::Plot(points) => {
                        for point in &points {
                            let line = format!("{},{}\n", point.x, point.y);
                            let _ = file.write_all(line.as_bytes());
                        }
                    }
                    PendingExport::Sequence(capture) => {
                        let _ = write_sequence_capture_csv(&mut file, &capture);
                    }
                    PendingExport::TorqueCalibration(result) => {
                        let _ = write_torque_calibration_csv(&mut file, &result);
                    }
                }
            }
        });
    }
}

fn param_row(
    ui: &mut egui::Ui,
    label: &str,
    value: &mut f32,
    text: &mut String,
    queued_commands: &mut Vec<Command>,
    make_param: fn(f32) -> ParameterValue,
) {
    let name_label = ui.label(label);
    ui.text_edit_singleline(text).labelled_by(name_label.id);
    let is_valid = text.parse::<f32>().map(|n| *value = n).is_ok();
    if ui.add_enabled(is_valid, egui::Button::new("Set")).clicked() {
        queued_commands.push(Command::SetParameter(make_param(*value)));
    }
    ui.end_row();
}

fn motion_field(ui: &mut egui::Ui, label: &str, value: &mut f32, text: &mut String) {
    let name_label = ui.label(label);
    ui.text_edit_singleline(text).labelled_by(name_label.id);
    if let Ok(parsed) = text.parse::<f32>() {
        *value = parsed;
    }
    ui.end_row();
}

fn integer_field(ui: &mut egui::Ui, label: &str, value: &mut usize, text: &mut String) {
    let name_label = ui.label(label);
    ui.text_edit_singleline(text).labelled_by(name_label.id);
    if let Ok(parsed) = text.parse::<usize>() {
        *value = parsed;
    }
    ui.end_row();
}

impl TorqueCalibrationPhase {
    fn label(self) -> &'static str {
        match self {
            TorqueCalibrationPhase::TareCommand => "tare command",
            TorqueCalibrationPhase::TareSettling { .. } => "tare settle",
            TorqueCalibrationPhase::TareSampling => "tare sampling",
            TorqueCalibrationPhase::ReleaseCommand => "release command",
            TorqueCalibrationPhase::WaitingForRelease { .. } => "releasing",
            TorqueCalibrationPhase::ApproachCommand => "approach command",
            TorqueCalibrationPhase::WaitingForContact { .. } => "approaching",
            TorqueCalibrationPhase::CommandStep => "command",
            TorqueCalibrationPhase::Settling { .. } => "settling",
            TorqueCalibrationPhase::Sampling => "sampling",
        }
    }
}

impl CalibrationDirection {
    fn label(self) -> &'static str {
        match self {
            CalibrationDirection::Descending => "descending",
            CalibrationDirection::Ascending => "ascending",
        }
    }
}

fn torque_calibration_steps(tab: &MotorTab) -> Option<Vec<TorqueCalibrationStep>> {
    if !(tab.cal_iq_start.is_finite()
        && tab.cal_iq_end.is_finite()
        && tab.cal_iq_step.is_finite()
        && tab.cal_arm_mm.is_finite()
        && tab.cal_stability_std_g.is_finite()
        && tab.cal_outlier_sigma.is_finite()
        && tab.cal_release_threshold_g.is_finite()
        && tab.cal_release_step_rad.is_finite()
        && tab.cal_release_max_rad.is_finite()
        && tab.cal_contact_threshold_g.is_finite())
        || tab.cal_iq_start >= 0.0
        || tab.cal_iq_end >= 0.0
        || tab.cal_iq_step >= 0.0
        || tab.cal_sample_count == 0
        || tab.cal_repeat_count == 0
        || tab.cal_arm_mm <= 0.0
        || tab.cal_stability_std_g < 0.0
        || tab.cal_stability_samples == 0
        || tab.cal_stability_timeout_ms == 0
        || tab.cal_outlier_sigma <= 0.0
        || tab.cal_release_threshold_g < 0.0
        || tab.cal_release_timeout_ms == 0
        || tab.cal_release_step_rad <= 0.0
        || tab.cal_release_max_rad <= 0.0
        || tab.cal_contact_threshold_g <= tab.cal_release_threshold_g
    {
        return None;
    }

    let mut points = Vec::new();
    let mut iq_ref = tab.cal_iq_start;
    while iq_ref >= tab.cal_iq_end - 1e-6 {
        if iq_ref >= 0.0 {
            return None;
        }
        points.push(iq_ref);
        iq_ref += tab.cal_iq_step;
        if points.len() > 256 {
            return None;
        }
    }

    if points.is_empty() {
        return None;
    }

    let mut steps = Vec::new();
    for repeat in 0..tab.cal_repeat_count {
        for &iq_ref in &points {
            steps.push(TorqueCalibrationStep {
                iq_ref,
                repeat,
                direction: CalibrationDirection::Descending,
            });
        }
        for &iq_ref in points.iter().rev() {
            steps.push(TorqueCalibrationStep {
                iq_ref,
                repeat,
                direction: CalibrationDirection::Ascending,
            });
        }
    }

    Some(steps)
}

fn mean(values: &[f32]) -> f32 {
    if values.is_empty() {
        return 0.0;
    }
    values.iter().sum::<f32>() / values.len() as f32
}

fn stddev(values: &[f32], mean: f32) -> f32 {
    if values.len() < 2 {
        return 0.0;
    }
    let variance = values
        .iter()
        .map(|value| (*value - mean).powi(2))
        .sum::<f32>()
        / (values.len() - 1) as f32;
    variance.sqrt()
}

fn stddev_deque(values: &VecDeque<f32>) -> f32 {
    if values.len() < 2 {
        return 0.0;
    }
    let mean = values.iter().sum::<f32>() / values.len() as f32;
    let variance = values
        .iter()
        .map(|value| (*value - mean).powi(2))
        .sum::<f32>()
        / (values.len() - 1) as f32;
    variance.sqrt()
}

fn push_sample(values: &mut VecDeque<f32>, value: f32) {
    if values.len() == values.capacity() {
        values.pop_front();
    }
    values.push_back(value);
}

fn reject_outliers(values: &[f32], sigma: f32) -> Vec<f32> {
    if values.len() < 3 {
        return values.to_vec();
    }
    let mean = mean(values);
    let std = stddev(values, mean);
    if std <= f32::EPSILON {
        return values.to_vec();
    }
    let filtered: Vec<f32> = values
        .iter()
        .copied()
        .filter(|value| (*value - mean).abs() <= sigma * std)
        .collect();
    if filtered.is_empty() {
        values.to_vec()
    } else {
        filtered
    }
}

fn grams_to_torque_nm(grams: f32, arm_mm: f32) -> f32 {
    grams * GRAM_FORCE_TO_NEWTON * arm_mm / 1000.0
}

fn torque_calibration_result(calibration: &TorqueCalibration) -> TorqueCalibrationResult {
    let (zero_slope, zero_rmse, zero_r2) = zero_intercept_fit(&calibration.rows);
    let (affine_slope, affine_intercept, affine_rmse, affine_r2) = affine_fit(&calibration.rows);
    TorqueCalibrationResult {
        arm_mm: calibration.arm_mm,
        tare_g: calibration.tare_g,
        fit_zero_slope_nm_per_iq: zero_slope,
        fit_affine_slope_nm_per_iq: affine_slope,
        fit_affine_intercept_nm: affine_intercept,
        fit_zero_rmse_nm: zero_rmse,
        fit_affine_rmse_nm: affine_rmse,
        fit_zero_r2: zero_r2,
        fit_affine_r2: affine_r2,
        rows: calibration.rows.clone(),
    }
}

fn zero_intercept_fit(rows: &[TorqueCalibrationRow]) -> (Option<f32>, Option<f32>, Option<f32>) {
    if rows.is_empty() {
        return (None, None, None);
    }

    let denom = rows.iter().map(|row| row.iq_ref.abs().powi(2)).sum::<f32>();
    if denom <= f32::EPSILON {
        return (None, None, None);
    }

    let slope = rows
        .iter()
        .map(|row| row.iq_ref.abs() * row.torque_nm)
        .sum::<f32>()
        / denom;
    let rmse = fit_rmse(rows, slope, 0.0);
    let r2 = fit_r2(rows, slope, 0.0);
    (Some(slope), Some(rmse), Some(r2))
}

fn affine_fit(
    rows: &[TorqueCalibrationRow],
) -> (Option<f32>, Option<f32>, Option<f32>, Option<f32>) {
    if rows.len() < 2 {
        return (None, None, None, None);
    }

    let mean_x = rows.iter().map(|row| row.iq_ref.abs()).sum::<f32>() / rows.len() as f32;
    let mean_y = rows.iter().map(|row| row.torque_nm).sum::<f32>() / rows.len() as f32;
    let denom = rows
        .iter()
        .map(|row| (row.iq_ref.abs() - mean_x).powi(2))
        .sum::<f32>();
    if denom <= f32::EPSILON {
        return (None, None, None, None);
    }

    let slope = rows
        .iter()
        .map(|row| (row.iq_ref.abs() - mean_x) * (row.torque_nm - mean_y))
        .sum::<f32>()
        / denom;
    let intercept = mean_y - slope * mean_x;
    let rmse = fit_rmse(rows, slope, intercept);
    let r2 = fit_r2(rows, slope, intercept);
    (Some(slope), Some(intercept), Some(rmse), Some(r2))
}

fn fit_rmse(rows: &[TorqueCalibrationRow], slope: f32, intercept: f32) -> f32 {
    let mse = rows
        .iter()
        .map(|row| {
            let predicted = slope * row.iq_ref.abs() + intercept;
            (row.torque_nm - predicted).powi(2)
        })
        .sum::<f32>()
        / rows.len() as f32;
    mse.sqrt()
}

fn fit_r2(rows: &[TorqueCalibrationRow], slope: f32, intercept: f32) -> f32 {
    let mean_y = rows.iter().map(|row| row.torque_nm).sum::<f32>() / rows.len() as f32;
    let ss_res = rows
        .iter()
        .map(|row| {
            let predicted = slope * row.iq_ref.abs() + intercept;
            (row.torque_nm - predicted).powi(2)
        })
        .sum::<f32>();
    let ss_tot = rows
        .iter()
        .map(|row| (row.torque_nm - mean_y).powi(2))
        .sum::<f32>();
    if ss_tot <= f32::EPSILON {
        return 1.0;
    }
    1.0 - ss_res / ss_tot
}

fn push_point(buffer: &mut VecDeque<egui_plot::PlotPoint>, point: egui_plot::PlotPoint) {
    if buffer.len() == MAX_PLOT_POINTS {
        buffer.pop_front();
    }
    buffer.push_back(point);
}

impl ChirpTarget {
    fn label(self) -> &'static str {
        match self {
            ChirpTarget::Angle => "AngleRef",
            ChirpTarget::Velocity => "SpeedRef",
            ChirpTarget::Torque => "TorqueRef",
        }
    }

    fn run_mode(self) -> RunMode {
        match self {
            ChirpTarget::Angle => RunMode::Angle,
            ChirpTarget::Velocity => RunMode::Velocity,
            ChirpTarget::Torque => RunMode::Torque,
        }
    }
}

impl PlotType {
    fn plot_line_names(self) -> (&'static str, Option<&'static str>) {
        match self {
            PlotType::Angle => ("Angle", Some("AngleRef")),
            PlotType::Velocity => ("Velocity", Some("SpeedRef")),
            PlotType::Torque => ("Torque", Some("TorqueRef")),
            PlotType::Current => ("Iq", Some("Id")),
            PlotType::SpeedError => ("SpeedError", None),
            PlotType::TorqueRef => ("TorqueRef", None),
            PlotType::VelocityIntegral => ("VelocityIntegral", None),
        }
    }
}

impl ActiveSequence {
    fn run_mode(self) -> RunMode {
        self.target().run_mode()
    }

    fn target(self) -> ChirpTarget {
        match self {
            ActiveSequence::Chirp { target, .. } | ActiveSequence::Step { target, .. } => target,
        }
    }

    fn initial_command_value(self) -> Option<(ChirpTarget, f32)> {
        match self {
            ActiveSequence::Chirp { target, .. } => Some((target, 0.0)),
            ActiveSequence::Step { target, value } => Some((target, value)),
        }
    }

    fn csv_fields(self) -> (&'static str, &'static str, f32) {
        match self {
            ActiveSequence::Chirp {
                target, amplitude, ..
            } => ("chirp", target.label(), amplitude),
            ActiveSequence::Step { target, value } => ("step", target.label(), value),
        }
    }
}

enum RuntimeTick {
    Idle,
    Send { target: ChirpTarget, value: f32 },
    Finished { target: ChirpTarget },
}

impl From<ActiveSequence> for RuntimeSequence {
    fn from(sequence: ActiveSequence) -> Self {
        match sequence {
            ActiveSequence::Chirp {
                target,
                amplitude,
                duration,
            } => Self::Chirp(RuntimeChirp {
                target,
                amplitude,
                duration,
                t0: Instant::now(),
                last_command_at: None,
            }),
            ActiveSequence::Step { target, .. } => Self::Step { target },
        }
    }
}

impl RuntimeSequence {
    fn target(&self) -> ChirpTarget {
        match self {
            RuntimeSequence::Chirp(chirp) => chirp.target,
            RuntimeSequence::Step { target, .. } => *target,
        }
    }

    fn tick(&mut self, now: Instant) -> RuntimeTick {
        match self {
            RuntimeSequence::Step { .. } => RuntimeTick::Idle,
            RuntimeSequence::Chirp(chirp) => {
                let elapsed = now.saturating_duration_since(chirp.t0);
                if elapsed >= chirp.duration {
                    return RuntimeTick::Finished {
                        target: chirp.target,
                    };
                }

                if chirp.last_command_at.is_some_and(|last| {
                    now.saturating_duration_since(last) < CHIRP_COMMAND_INTERVAL
                }) {
                    return RuntimeTick::Idle;
                }

                chirp.last_command_at = Some(now);
                RuntimeTick::Send {
                    target: chirp.target,
                    value: chirp_velocity_command(
                        chirp.amplitude,
                        elapsed.as_secs_f32(),
                        chirp.duration.as_secs_f32(),
                    ),
                }
            }
        }
    }
}

fn sequence_parameter(target: ChirpTarget, value: f32) -> Command {
    let parameter = match target {
        ChirpTarget::Angle => ParameterValue::AngleRef(value),
        ChirpTarget::Velocity => ParameterValue::SpeedRef(value),
        ChirpTarget::Torque => ParameterValue::TorqueRef(value),
    };
    Command::SetParameter(parameter)
}

fn chirp_velocity_command(amplitude: f32, elapsed_secs: f32, duration_secs: f32) -> f32 {
    if !(amplitude.is_finite() && duration_secs.is_finite())
        || amplitude <= 0.0
        || duration_secs <= 0.0
    {
        return 0.0;
    }

    let sweep_ratio = CHIRP_END_FREQ_HZ / CHIRP_START_FREQ_HZ;
    let exponent = elapsed_secs / duration_secs;
    let phase = 2.0 * f32::consts::PI * CHIRP_START_FREQ_HZ * duration_secs / sweep_ratio.ln()
        * (sweep_ratio.powf(exponent) - 1.0);
    amplitude * phase.sin()
}

fn write_sequence_capture_csv(file: &mut File, capture: &SequenceCapture) -> std::io::Result<()> {
    file.write_all(
        b"sequence_kind,sequence_target,sequence_value,timestamp_ms,row_kind,command_target,command_value,angle,velocity,torque,temperature,i_q,i_d,debug_kind,debug_value\n",
    )?;

    for row in &capture.rows {
        let line = row.to_csv_line(capture.sequence);
        file.write_all(line.as_bytes())?;
    }

    Ok(())
}

fn write_torque_calibration_csv(
    file: &mut File,
    result: &TorqueCalibrationResult,
) -> std::io::Result<()> {
    let summary = format!(
        "arm_mm,tare_g,zero_slope_nm_per_iq,zero_rmse_nm,zero_r2,affine_slope_nm_per_iq,affine_intercept_nm,affine_rmse_nm,affine_r2\n{},{},{},{},{},{},{},{},{}\n\n",
        result.arm_mm,
        result.tare_g,
        optional_f32(result.fit_zero_slope_nm_per_iq),
        optional_f32(result.fit_zero_rmse_nm),
        optional_f32(result.fit_zero_r2),
        optional_f32(result.fit_affine_slope_nm_per_iq),
        optional_f32(result.fit_affine_intercept_nm),
        optional_f32(result.fit_affine_rmse_nm),
        optional_f32(result.fit_affine_r2),
    );
    file.write_all(summary.as_bytes())?;
    file.write_all(
        b"repeat,direction,iq_ref,raw_mean_g,tare_corrected_mean_g,std_g,sample_count,rejected_samples,arm_mm,torque_nm,nm_per_abs_iq\n",
    )?;

    for row in &result.rows {
        let nm_per_abs_iq = row.torque_nm / row.iq_ref.abs();
        let line = format!(
            "{},{},{},{},{},{},{},{},{},{},{}\n",
            row.repeat,
            row.direction.label(),
            row.iq_ref,
            row.raw_mean_g,
            row.tare_corrected_mean_g,
            row.std_g,
            row.sample_count,
            row.rejected_samples,
            result.arm_mm,
            row.torque_nm,
            nm_per_abs_iq
        );
        file.write_all(line.as_bytes())?;
    }

    Ok(())
}

fn optional_f32(value: Option<f32>) -> String {
    value.map(|value| value.to_string()).unwrap_or_default()
}

impl SequenceCaptureRow {
    fn to_csv_line(self, sequence: ActiveSequence) -> String {
        let (sequence_kind, sequence_target, sequence_value) = sequence.csv_fields();
        match self {
            SequenceCaptureRow::Command {
                timestamp_ms,
                target,
                value,
            } => format!(
                "{sequence_kind},{sequence_target},{sequence_value},{timestamp_ms},command,{},{},,,,,,,,\n",
                target.label(),
                value
            ),
            SequenceCaptureRow::Status {
                timestamp_ms,
                angle,
                velocity,
                torque,
                temperature,
            } => format!(
                "{sequence_kind},{sequence_target},{sequence_value},{timestamp_ms},status,,,{angle},{velocity},{torque},{temperature},,,,\n"
            ),
            SequenceCaptureRow::Current {
                timestamp_ms,
                i_q,
                i_d,
            } => format!(
                "{sequence_kind},{sequence_target},{sequence_value},{timestamp_ms},current,,,,,,,,{i_q},{i_d},,\n"
            ),
            SequenceCaptureRow::DebugValue {
                timestamp_ms,
                kind,
                value,
            } => format!(
                "{sequence_kind},{sequence_target},{sequence_value},{timestamp_ms},debug,,,,,,,,{},{}\n",
                debug_value_kind_label(kind),
                value
            ),
        }
    }
}

fn current_timestamp_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
}

fn debug_value_kind_label(kind: DebugValueKind) -> &'static str {
    match kind {
        DebugValueKind::SpeedError => "speed_error",
        DebugValueKind::TorqueRef => "torque_ref",
        DebugValueKind::VelocityIntegral => "velocity_integral",
    }
}

// Helper functions for CAN frame conversions
fn can_frame_to_can_message(frame: &CanFrame) -> CanMessage {
    let id = frame.id();
    let raw_id = match id {
        socketcan::Id::Standard(std_id) => std_id.as_raw() as u32,
        socketcan::Id::Extended(ext_id) => ext_id.as_raw(),
    };

    let mut data = [0u8; 8];
    let frame_data = frame.data();
    let len = frame_data.len().min(8);
    data[..len].copy_from_slice(&frame_data[..len]);

    CanMessage {
        id: raw_id,
        data,
        length: len as u8,
    }
}

fn can_message_to_can_frame(msg: &CanMessage) -> Result<CanFrame, socketcan::ConstructionError> {
    let id = ExtendedId::new(msg.id).ok_or(socketcan::ConstructionError::IDTooLarge)?;
    CanFrame::new(
        socketcan::Id::Extended(id),
        &msg.data[..msg.length as usize],
    )
    .ok_or(socketcan::ConstructionError::TooMuchData)
}

fn response_matches_host(message: &ResponseMessage) -> bool {
    message.host_can_id == HOST_CAN_ID
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn response_filter_accepts_matching_host_id() {
        let message = ResponseMessage {
            motor_can_id: DEFAULT_MOTOR_CAN_ID,
            host_can_id: HOST_CAN_ID,
            body: ResponseBody::MotorStatus(can_message::message::MotorStatus {
                angle: 0.0,
                velocity: 0.0,
                torque: 0.0,
                temperature: 0.0,
            }),
        };

        assert!(response_matches_host(&message));
    }

    #[test]
    fn response_filter_rejects_other_host_id() {
        let message = ResponseMessage {
            motor_can_id: DEFAULT_MOTOR_CAN_ID,
            host_can_id: HOST_CAN_ID + 1,
            body: ResponseBody::ParameterValue(ParameterValue::SpeedKp(1.0)),
        };

        assert!(!response_matches_host(&message));
    }

    #[test]
    fn push_point_discards_oldest_sample_at_capacity() {
        let mut points = VecDeque::with_capacity(MAX_PLOT_POINTS);
        for i in 0..=MAX_PLOT_POINTS {
            push_point(&mut points, egui_plot::PlotPoint::new(i as f64, i as f64));
        }

        assert_eq!(points.len(), MAX_PLOT_POINTS);
        assert_eq!(points.front().unwrap().x, 1.0);
        assert_eq!(points.back().unwrap().x, MAX_PLOT_POINTS as f64);
    }

    #[test]
    fn chirp_starts_at_zero() {
        assert_eq!(chirp_velocity_command(5.0, 0.0, 30.0), 0.0);
    }

    #[test]
    fn chirp_respects_amplitude_bound() {
        let sample = chirp_velocity_command(5.0, 7.5, 30.0);
        assert!(sample.abs() <= 5.0);
    }
}
