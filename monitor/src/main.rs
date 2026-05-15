#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
use std::fs::File;
use std::io::prelude::Write;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::thread::sleep;
use std::time::{Duration, Instant};
use std::{f32, thread};

use can_message::message::{
    CanMessage, Command, CommandMessage, FeedbackType, ParameterIndex, ParameterValue,
    ResponseBody, ResponseMessage, RunMode,
};
use eframe::egui;
use socketcan::socket::{CanSocket, Socket};
use socketcan::{CanFrame, EmbeddedFrame, ExtendedId};
use std::sync::Arc;
use std::sync::atomic::{AtomicU8, Ordering};

const WINDOW_WIDTH: f32 = 1024.0;
const WINDOW_HEIGHT: f32 = 768.0;
const MAX_PLOT_POINTS: usize = 100_000;
const UI_REPAINT_INTERVAL: Duration = Duration::from_millis(16);
const HOST_CAN_ID: u8 = 0x00;
const DEFAULT_MOTOR_CAN_ID: u8 = 0x0F;

fn main() -> eframe::Result {
    env_logger::init();

    let (command_sender, command_receiver) = channel::<CommandMessage>();
    let (status_sender, status_receiver) = channel::<ResponseMessage>();
    let target_motor_can_id = Arc::new(AtomicU8::new(DEFAULT_MOTOR_CAN_ID));

    let rx_socket = CanSocket::open("can0").unwrap();
    let tx_socket = CanSocket::open("can0").unwrap();

    let rx_target_motor_can_id = Arc::clone(&target_motor_can_id);
    thread::spawn(move || {
        forward_status(status_sender, rx_socket, rx_target_motor_can_id);
    });
    let tx_target_motor_can_id = Arc::clone(&target_motor_can_id);
    thread::spawn(move || {
        forward_command(command_receiver, tx_socket, tx_target_motor_can_id);
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
                status_receiver,
                target_motor_can_id,
            )))
        }),
    )
}

fn forward_command(
    command_recv: Receiver<CommandMessage>,
    socket: CanSocket,
    target_motor_can_id: Arc<AtomicU8>,
) {
    loop {
        for mut command_msg in command_recv.try_iter() {
            command_msg.host_can_id = HOST_CAN_ID;
            command_msg.motor_can_id = target_motor_can_id.load(Ordering::Relaxed);
            if let Ok(can_msg) = CanMessage::try_from(command_msg) {
                match can_message_to_can_frame(&can_msg) {
                    Ok(frame) => {
                        if let Err(e) = socket.write_frame(&frame) {
                            println!("Failed to send CAN frame: {}", e);
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

fn forward_status(
    status_send: Sender<ResponseMessage>,
    socket: CanSocket,
    target_motor_can_id: Arc<AtomicU8>,
) {
    println!("CAN receiver started");

    loop {
        match socket.read_frame() {
            Ok(frame) => {
                let can_msg = can_frame_to_can_message(&frame);
                if let Ok(status_msg) = ResponseMessage::try_from(can_msg) {
                    if response_matches_target(
                        &status_msg,
                        target_motor_can_id.load(Ordering::Relaxed),
                    ) {
                        let _ = status_send.send(status_msg);
                    }
                }
            }
            Err(e) => {
                println!("Failed to read CAN frame: {}", e);
                thread::sleep(Duration::from_millis(10));
            }
        }
    }
}

struct MyApp {
    command_send: Sender<CommandMessage>,
    status_recv: Receiver<ResponseMessage>,
    target_motor_can_id: Arc<AtomicU8>,
    target_motor_can_id_string: String,

    angle_string: String,
    velocity_string: String,
    iq_string: String,
    vq_string: String,

    angle_kp_string: String,
    speed_kp_string: String,
    speed_ki_string: String,

    iq_kp_string: String,
    iq_ki_string: String,

    spring_string: String,
    damping_string: String,

    angle: f32,
    velocity: f32,
    iq: f32,
    vq: f32,

    angle_kp: f32,
    speed_kp: f32,
    speed_ki: f32,

    iq_kp: f32,
    iq_ki: f32,

    spring: f32,
    damping: f32,

    run_mode: RunMode,

    frequency: f32,
    is_non_inverted: bool,
    last_nonzero_at: Instant,

    plot_type: PlotType,
    plot_points_1: [egui_plot::PlotPoint; MAX_PLOT_POINTS],
    plot_points_2: [egui_plot::PlotPoint; MAX_PLOT_POINTS],
    n_plot_points: usize,
    is_plotting: bool,

    file_dialog: egui_file_dialog::FileDialog,

    t0: Instant,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PlotType {
    Angle,
    Velocity,
    Torque,
    Current,
}

fn param_row(
    ui: &mut egui::Ui,
    label: &str,
    value: &mut f32,
    text: &mut String,
    command_send: &Sender<CommandMessage>,
    motor_can_id: u8,
    make_param: fn(f32) -> ParameterValue,
) {
    let name_label = ui.label(label);
    ui.text_edit_singleline(text).labelled_by(name_label.id);
    let is_valid = text.parse::<f32>().map(|n| *value = n).is_ok();
    if ui.add_enabled(is_valid, egui::Button::new("Set")).clicked() {
        let _ = command_send.send(CommandMessage {
            host_can_id: HOST_CAN_ID,
            motor_can_id,
            command: Command::SetParameter(make_param(*value)),
        });
    }
    ui.end_row();
}

impl MyApp {
    fn new(
        command_send: Sender<CommandMessage>,
        status_recv: Receiver<ResponseMessage>,
        target_motor_can_id: Arc<AtomicU8>,
    ) -> Self {
        let angle = 0.0;
        let velocity = 0.0;
        let iq = 0.0;
        let vq = 0.0;

        let angle_kp = 0.0;
        let speed_kp = 0.0;
        let speed_ki = 0.0;
        let iq_kp = 0.0;
        let iq_ki = 0.0;

        let spring = 0.0;
        let damping = 0.0;

        let app = Self {
            command_send,
            status_recv,
            target_motor_can_id,
            target_motor_can_id_string: format!("{DEFAULT_MOTOR_CAN_ID}"),

            angle_string: angle.to_string(),
            velocity_string: velocity.to_string(),
            iq_string: iq.to_string(),
            vq_string: vq.to_string(),

            angle_kp_string: angle_kp.to_string(),
            speed_kp_string: speed_kp.to_string(),
            speed_ki_string: speed_ki.to_string(),

            iq_kp_string: iq_kp.to_string(),
            iq_ki_string: iq_ki.to_string(),

            spring_string: spring.to_string(),
            damping_string: damping.to_string(),

            angle,
            velocity,
            iq,
            vq,

            angle_kp,
            speed_kp,
            speed_ki,

            iq_kp,
            iq_ki,

            spring,
            damping,

            frequency: 0.0,
            is_non_inverted: false,
            last_nonzero_at: Instant::now(),

            plot_type: PlotType::Angle,
            run_mode: RunMode::Impedance,
            plot_points_1: [egui_plot::PlotPoint::new(0.0, 0.0); MAX_PLOT_POINTS],
            plot_points_2: [egui_plot::PlotPoint::new(0.0, 0.0); MAX_PLOT_POINTS],
            n_plot_points: 0,
            is_plotting: false,

            file_dialog: egui_file_dialog::FileDialog::new(),
            t0: Instant::now(),
        };
        app.request_initial_parameters();
        app
    }

    fn current_motor_can_id(&self) -> u8 {
        self.target_motor_can_id.load(Ordering::Relaxed)
    }

    fn set_target_motor_can_id(&mut self, new_id: u8) {
        self.target_motor_can_id.store(new_id, Ordering::Relaxed);
        self.n_plot_points = 0;
        self.request_initial_parameters();
    }

    fn send_command(&self, command: Command) {
        let _ = self.command_send.send(CommandMessage {
            host_can_id: HOST_CAN_ID,
            motor_can_id: self.current_motor_can_id(),
            command,
        });
    }

    fn send_set_can_id(&mut self, new_id: u8) {
        self.send_command(Command::SetCanId(new_id));
        self.set_target_motor_can_id(new_id);
    }

    fn request_initial_parameters(&self) {
        self.send_command(Command::GetParameter(ParameterIndex::AngleKp));
        sleep(Duration::from_millis(1));
        self.send_command(Command::GetParameter(ParameterIndex::SpeedKp));
        sleep(Duration::from_millis(1));
        self.send_command(Command::GetParameter(ParameterIndex::SpeedKi));
        sleep(Duration::from_millis(1));
        self.send_command(Command::GetParameter(ParameterIndex::CurrentKp));
        sleep(Duration::from_millis(1));
        self.send_command(Command::GetParameter(ParameterIndex::CurrentKi));
        sleep(Duration::from_millis(1));
    }
}

impl eframe::App for MyApp {
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        for s in self.status_recv.try_iter() {
            match s.body {
                ResponseBody::MotorStatus(x) => {
                    if self.is_plotting {
                        if self.plot_type == PlotType::Current {
                            continue;
                        }
                        let y = match self.plot_type {
                            PlotType::Angle => x.angle,
                            PlotType::Velocity => x.velocity,
                            PlotType::Torque => x.torque,
                            PlotType::Current => panic!(),
                        };
                        if let Some(now) = Instant::now().checked_duration_since(self.t0) {
                            if s.host_can_id != 0 {
                                continue;
                            }
                            let x = now.as_micros() as f32 / 1e6;
                            let new_point = egui_plot::PlotPoint::new(x, y as f64);
                            self.plot_points_1[self.n_plot_points] = new_point;
                            self.n_plot_points += 1;
                            if self.n_plot_points == MAX_PLOT_POINTS {
                                self.n_plot_points = 0;
                            }
                        }
                    }
                }
                ResponseBody::MotorCurrent(x) => {
                    if self.is_plotting {
                        if self.plot_type != PlotType::Current {
                            continue;
                        }
                        let (y1, y2) = match self.plot_type {
                            PlotType::Current => (x.i_q, x.i_d),
                            _ => panic!(),
                        };

                        if let Some(now) = Instant::now().checked_duration_since(self.t0) {
                            if s.host_can_id != 0 {
                                continue;
                            }
                            let x = now.as_micros() as f32 / 1e6;
                            let p1 = egui_plot::PlotPoint::new(x, y1 as f64);
                            let p2 = egui_plot::PlotPoint::new(x, y2 as f64);
                            self.plot_points_1[self.n_plot_points] = p1;
                            self.plot_points_2[self.n_plot_points] = p2;
                            self.n_plot_points += 1;
                            if self.n_plot_points == MAX_PLOT_POINTS {
                                self.n_plot_points = 0;
                            }
                        }
                    }
                }
                ResponseBody::ParameterValue(pv) => match pv {
                    ParameterValue::RunMode(x) => self.run_mode = x,
                    ParameterValue::AngleRef(x) => {
                        self.angle = x;
                        self.angle_string = x.to_string();
                    }
                    ParameterValue::SpeedRef(x) => {
                        self.velocity = x;
                        self.velocity_string = x.to_string();
                    }
                    ParameterValue::IqRef(x) => {
                        self.iq = x;
                        self.iq_string = x.to_string();
                    }
                    ParameterValue::AngleKp(x) => {
                        self.angle_kp = x;
                        self.angle_kp_string = x.to_string();
                    }
                    ParameterValue::SpeedKp(x) => {
                        self.speed_kp = x;
                        self.speed_kp_string = x.to_string();
                    }
                    ParameterValue::SpeedKi(x) => {
                        self.speed_ki = x;
                        self.speed_ki_string = x.to_string();
                    }
                    ParameterValue::CurrentKp(x) => {
                        self.iq_kp = x;
                        self.iq_kp_string = x.to_string();
                    }
                    ParameterValue::CurrentKi(x) => {
                        self.iq_ki = x;
                        self.iq_ki_string = x.to_string();
                    }
                    ParameterValue::Spring(x) => {
                        self.spring = x;
                        self.spring_string = x.to_string();
                    }
                    ParameterValue::Damping(x) => {
                        self.damping = x;
                        self.damping_string = x.to_string();
                    }
                    _ => (),
                },
            }
        }
        ctx.request_repaint_after(UI_REPAINT_INTERVAL);
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show_inside(ui, |ui| {
            ui.heading("BLDC Monitor");

            ui.horizontal(|ui| {
                let name_label = ui.label("Target CAN ID");
                ui.text_edit_singleline(&mut self.target_motor_can_id_string)
                    .labelled_by(name_label.id);

                let parsed_id = self.target_motor_can_id_string.parse::<u8>().ok();
                if ui
                    .add_enabled(parsed_id.is_some(), egui::Button::new("Apply"))
                    .clicked()
                    && let Some(new_id) = parsed_id
                {
                    self.set_target_motor_can_id(new_id);
                }

                if ui
                    .add_enabled(parsed_id.is_some(), egui::Button::new("Set on motor"))
                    .clicked()
                    && let Some(new_id) = parsed_id
                {
                    self.send_set_can_id(new_id);
                }

                ui.label(format!("Current: {}", self.current_motor_can_id()));
            });

            let old_mode = self.run_mode;
            ui.label("Run mode");
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.run_mode, RunMode::Impedance, "Impedance");
                ui.selectable_value(&mut self.run_mode, RunMode::Angle, "Angle");
                ui.selectable_value(&mut self.run_mode, RunMode::Velocity, "Velocity");
                ui.selectable_value(&mut self.run_mode, RunMode::Torque, "Torque");
                ui.selectable_value(&mut self.run_mode, RunMode::Voltage, "Voltage");
            });
            if old_mode != self.run_mode {
                self.send_command(Command::SetParameter(ParameterValue::RunMode(
                    self.run_mode,
                )));
            }

            let current_motor_can_id = self.current_motor_can_id();
            egui::Grid::new("params")
                .num_columns(3)
                .spacing([40.0, 4.0])
                .show(ui, |ui| {
                    param_row(
                        ui,
                        "Angle Ref",
                        &mut self.angle,
                        &mut self.angle_string,
                        &self.command_send,
                        current_motor_can_id,
                        ParameterValue::AngleRef,
                    );
                    param_row(
                        ui,
                        "Velocity Ref",
                        &mut self.velocity,
                        &mut self.velocity_string,
                        &self.command_send,
                        current_motor_can_id,
                        ParameterValue::SpeedRef,
                    );
                    param_row(
                        ui,
                        "I_q Ref",
                        &mut self.iq,
                        &mut self.iq_string,
                        &self.command_send,
                        current_motor_can_id,
                        ParameterValue::IqRef,
                    );
                    param_row(
                        ui,
                        "V_q Ref",
                        &mut self.vq,
                        &mut self.vq_string,
                        &self.command_send,
                        current_motor_can_id,
                        ParameterValue::VqRef,
                    );
                    param_row(
                        ui,
                        "Angle K_p",
                        &mut self.angle_kp,
                        &mut self.angle_kp_string,
                        &self.command_send,
                        current_motor_can_id,
                        ParameterValue::AngleKp,
                    );
                    param_row(
                        ui,
                        "Speed K_p",
                        &mut self.speed_kp,
                        &mut self.speed_kp_string,
                        &self.command_send,
                        current_motor_can_id,
                        ParameterValue::SpeedKp,
                    );
                    param_row(
                        ui,
                        "Speed K_i",
                        &mut self.speed_ki,
                        &mut self.speed_ki_string,
                        &self.command_send,
                        current_motor_can_id,
                        ParameterValue::SpeedKi,
                    );
                    param_row(
                        ui,
                        "Current K_p",
                        &mut self.iq_kp,
                        &mut self.iq_kp_string,
                        &self.command_send,
                        current_motor_can_id,
                        ParameterValue::CurrentKp,
                    );
                    param_row(
                        ui,
                        "Current K_i",
                        &mut self.iq_ki,
                        &mut self.iq_ki_string,
                        &self.command_send,
                        current_motor_can_id,
                        ParameterValue::CurrentKi,
                    );
                    param_row(
                        ui,
                        "Spring",
                        &mut self.spring,
                        &mut self.spring_string,
                        &self.command_send,
                        current_motor_can_id,
                        ParameterValue::Spring,
                    );
                    param_row(
                        ui,
                        "Damping",
                        &mut self.damping,
                        &mut self.damping_string,
                        &self.command_send,
                        current_motor_can_id,
                        ParameterValue::Damping,
                    );
                });

            let name_label = ui.label("On-off frequency");
            let mut frequency_str = self.frequency.to_string();
            ui.text_edit_singleline(&mut frequency_str)
                .labelled_by(name_label.id);
            match frequency_str.parse::<f32>() {
                Ok(n) => self.frequency = n,
                _ => self.frequency = 0.0,
            }

            if self.frequency > 0.0 {
                let now = Instant::now();
                let max_dt = Duration::from_millis((1000.0 / self.frequency) as u64);
                if (now - self.last_nonzero_at) > max_dt {
                    self.last_nonzero_at = now;
                    if !self.is_non_inverted {
                        match self.run_mode {
                            RunMode::Angle => {
                                self.send_command(Command::SetParameter(ParameterValue::AngleRef(
                                    -self.angle,
                                )));
                            }
                            RunMode::Velocity => {
                                self.send_command(Command::SetParameter(ParameterValue::SpeedRef(
                                    -self.velocity,
                                )));
                            }
                            RunMode::Torque => {
                                self.send_command(Command::SetParameter(ParameterValue::IqRef(
                                    -self.iq,
                                )));
                            }
                            RunMode::Voltage => {
                                self.send_command(Command::SetParameter(ParameterValue::VqRef(
                                    -self.vq,
                                )));
                            }
                            _ => (),
                        }
                        self.is_non_inverted = true;
                    } else {
                        match self.run_mode {
                            RunMode::Angle => {
                                self.send_command(Command::SetParameter(ParameterValue::AngleRef(
                                    self.angle,
                                )));
                            }
                            RunMode::Velocity => {
                                self.send_command(Command::SetParameter(ParameterValue::SpeedRef(
                                    self.velocity,
                                )));
                            }
                            RunMode::Torque => {
                                self.send_command(Command::SetParameter(ParameterValue::IqRef(
                                    self.iq,
                                )));
                            }
                            RunMode::Voltage => {
                                self.send_command(Command::SetParameter(ParameterValue::VqRef(
                                    self.vq,
                                )));
                            }
                            _ => (),
                        }
                        self.is_non_inverted = false;
                    }
                }
            }

            ui.end_row();

            if ui.button("disable").clicked() {
                self.send_command(Command::Stop);
            }
            if ui.button("enable").clicked() {
                self.send_command(Command::Enable);
            }

            let before = self.plot_type;
            ui.label("Plot Type");
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.plot_type, PlotType::Angle, "Angle");
                ui.selectable_value(&mut self.plot_type, PlotType::Velocity, "Velocity");
                ui.selectable_value(&mut self.plot_type, PlotType::Torque, "Torque");
                ui.selectable_value(&mut self.plot_type, PlotType::Current, "Current");
            });
            if before != self.plot_type {
                if self.plot_type == PlotType::Current {
                    self.send_command(Command::SetFeedbackType(FeedbackType::Current));
                } else {
                    self.send_command(Command::SetFeedbackType(FeedbackType::Status));
                }
                self.n_plot_points = 0;
            }
            if ui.button("Plot").clicked() {
                if !self.is_plotting {
                    self.n_plot_points = 0;
                }
                self.is_plotting = !self.is_plotting;
            }

            let export_txt = format!("Export {} samples", self.plot_points_1.len());
            if ui
                .add_enabled(
                    !self.plot_points_1.is_empty(),
                    egui::Button::new(export_txt.as_str()),
                )
                .clicked()
            {
                self.file_dialog.save_file();
            }

            if let Some(path) = self.file_dialog.update(ui.ctx()).picked()
                && let Ok(mut file) = File::open(path)
            {
                for i in 0..self.n_plot_points {
                    let p = self.plot_points_1[i];
                    let line = format!("{},{}\n", p.x, p.y);
                    let _ = file.write_all(line.as_bytes());
                }
            }

            egui_plot::Plot::new("plot").show(ui, |plot_ui| {
                let points_1 =
                    egui_plot::PlotPoints::Borrowed(&self.plot_points_1[0..self.n_plot_points]);
                let line_1 = egui_plot::Line::new("y1", points_1)
                    .name("y1")
                    .style(egui_plot::LineStyle::Solid)
                    .color(egui::Color32::RED)
                    .width(2.0);
                plot_ui.line(line_1);

                if self.plot_type == PlotType::Current {
                    let points_2 =
                        egui_plot::PlotPoints::Borrowed(&self.plot_points_2[0..self.n_plot_points]);
                    let line_2 = egui_plot::Line::new("y2", points_2)
                        .name("y2")
                        .style(egui_plot::LineStyle::Solid)
                        .color(egui::Color32::GREEN)
                        .width(2.0);
                    plot_ui.line(line_2);
                }
            });
        });
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

fn response_matches_target(message: &ResponseMessage, target_motor_can_id: u8) -> bool {
    message.host_can_id == HOST_CAN_ID && message.motor_can_id == target_motor_can_id
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn response_filter_accepts_matching_host_and_motor_ids() {
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

        assert!(response_matches_target(&message, DEFAULT_MOTOR_CAN_ID));
    }

    #[test]
    fn response_filter_rejects_other_motor_id() {
        let message = ResponseMessage {
            motor_can_id: DEFAULT_MOTOR_CAN_ID + 1,
            host_can_id: HOST_CAN_ID,
            body: ResponseBody::MotorCurrent(can_message::message::MotorCurrent {
                i_q: 0.0,
                i_d: 0.0,
            }),
        };

        assert!(!response_matches_target(&message, DEFAULT_MOTOR_CAN_ID));
    }

    #[test]
    fn response_filter_rejects_other_host_id() {
        let message = ResponseMessage {
            motor_can_id: DEFAULT_MOTOR_CAN_ID,
            host_can_id: HOST_CAN_ID + 1,
            body: ResponseBody::ParameterValue(ParameterValue::SpeedKp(1.0)),
        };

        assert!(!response_matches_target(&message, DEFAULT_MOTOR_CAN_ID));
    }
}
