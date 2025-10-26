#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
use std::fs::File;
use std::io::prelude::Write;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::time::{Duration, Instant};
use std::{f32, thread};

use can_message::message::{
    CanMessage, Command, CommandMessage, ParameterIndex, ParameterValue, ResponseBody,
    ResponseMessage, RunMode,
};
use eframe::egui;
use socketcan::socket::{CanSocket, Socket};
use socketcan::{CanFrame, EmbeddedFrame, ExtendedId};

fn main() -> eframe::Result {
    env_logger::init();

    let (command_sender, command_receiver) = channel::<Command>();
    let (status_sender, status_receiver) = channel::<ResponseMessage>();

    let rx_socket = CanSocket::open("can0").unwrap();
    let tx_socket = CanSocket::open("can0").unwrap();

    thread::spawn(move || {
        forward_status(status_sender, rx_socket);
    });
    thread::spawn(move || {
        forward_command(command_receiver, tx_socket);
    });

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1024.0, 768.0]),
        ..Default::default()
    };
    eframe::run_native(
        "BLDC monitor",
        options,
        Box::new(|_| {
            Ok(Box::<MyApp>::new(MyApp::new(
                command_sender,
                status_receiver,
            )))
        }),
    )
}

fn forward_command(command_recv: Receiver<Command>, socket: CanSocket) {
    loop {
        for command in command_recv.try_iter() {
            let command_msg = CommandMessage {
                host_can_id: 0x00,  // Default host CAN ID
                motor_can_id: 0x0F, // Default motor CAN ID
                command,
            };
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

fn forward_status(status_send: Sender<ResponseMessage>, socket: CanSocket) {
    println!("CAN receiver started");

    loop {
        match socket.read_frame() {
            Ok(frame) => {
                let can_msg = can_frame_to_can_message(&frame);
                if let Ok(status_msg) = ResponseMessage::try_from(can_msg) {
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

struct MyApp {
    command_send: Sender<Command>,
    status_recv: Receiver<ResponseMessage>,

    angle_string: String,
    velocity_string: String,
    iq_string: String,

    angle_kp_string: String,
    speed_kp_string: String,
    speed_ki_string: String,

    iq_kp_string: String,
    iq_ki_string: String,

    angle: f32,
    velocity: f32,
    iq: f32,

    angle_kp: f32,
    speed_kp: f32,
    speed_ki: f32,

    iq_kp: f32,
    iq_ki: f32,

    run_mode: RunMode,

    plot_type: PlotType,
    plot_points: Vec<egui_plot::PlotPoint>,
    is_plotting: bool,

    file_dialog: egui_file_dialog::FileDialog,

    t0: Instant,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PlotType {
    Angle,
    Velocity,
    Torque,
}

impl MyApp {
    fn new(command_send: Sender<Command>, status_recv: Receiver<ResponseMessage>) -> Self {
        let angle = 0.0;
        let velocity = 0.0;
        let iq = 0.0;

        let angle_kp = 0.0;
        let speed_kp = 0.0;
        let speed_ki = 0.0;
        let iq_kp = 0.0;
        let iq_ki = 0.0;

        let _ = command_send.send(Command::GetParameter(ParameterIndex::AngleKp));
        let _ = command_send.send(Command::GetParameter(ParameterIndex::SpeedKp));
        let _ = command_send.send(Command::GetParameter(ParameterIndex::SpeedKi));
        let _ = command_send.send(Command::GetParameter(ParameterIndex::CurrentKp));
        let _ = command_send.send(Command::GetParameter(ParameterIndex::CurrentKi));

        Self {
            command_send,
            status_recv,

            angle_string: angle.to_string(),
            velocity_string: velocity.to_string(),
            iq_string: iq.to_string(),

            angle_kp_string: angle_kp.to_string(),
            speed_kp_string: speed_kp.to_string(),
            speed_ki_string: speed_ki.to_string(),

            iq_kp_string: iq_kp.to_string(),
            iq_ki_string: iq_ki.to_string(),

            angle,
            velocity,
            iq,

            angle_kp,
            speed_kp,
            speed_ki,

            iq_kp,
            iq_ki,

            plot_type: PlotType::Angle,
            run_mode: RunMode::Impedance,
            plot_points: vec![],
            is_plotting: false,

            file_dialog: egui_file_dialog::FileDialog::new(),
            t0: Instant::now(),
        }
    }
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        for s in self.status_recv.try_iter() {
            match s.body {
                ResponseBody::MotorStatus(x) => {
                    if self.is_plotting {
                        let value = match self.plot_type {
                            PlotType::Angle => x.angle,
                            PlotType::Velocity => x.velocity,
                            PlotType::Torque => x.torque,
                        };
                        if let Some(now) = Instant::now().checked_duration_since(self.t0) {
                            if s.host_can_id != 0 {
                                continue;
                            }
                            let timestamp = now.as_micros() as f32 / 1e6;
                            let new_point = egui_plot::PlotPoint::new(timestamp, value as f64);
                            self.plot_points.push(new_point);
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
                    _ => (),
                },
            }
        }
        ctx.set_pixels_per_point(1.5);
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("BLDC Monitor");

            let old_mode = self.run_mode;
            ui.label("Run mode");
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.run_mode, RunMode::Impedance, "Impedance");
                ui.selectable_value(&mut self.run_mode, RunMode::Angle, "Angle");
                ui.selectable_value(&mut self.run_mode, RunMode::Velocity, "Velocity");
                ui.selectable_value(&mut self.run_mode, RunMode::Torque, "Torque");
            });
            if old_mode != self.run_mode {
                let _ = self
                    .command_send
                    .send(Command::SetParameter(ParameterValue::RunMode(
                        self.run_mode,
                    )));
            }

            egui::Grid::new("params")
                .num_columns(3)
                .spacing([40.0, 4.0])
                .show(ui, |ui| {
                    let name_label = ui.label("Angle Ref");
                    ui.text_edit_singleline(&mut self.angle_string)
                        .labelled_by(name_label.id);

                    let mut is_button_active = true;
                    match self.angle_string.parse::<f32>() {
                        Ok(n) => self.angle = n,
                        _ => {
                            is_button_active = false;
                        }
                    }

                    if ui
                        .add_enabled(is_button_active, egui::Button::new("Set"))
                        .clicked()
                    {
                        let _ = self
                            .command_send
                            .send(Command::SetParameter(ParameterValue::AngleRef(self.angle)));
                    };

                    ui.end_row();

                    let name_label = ui.label("Velocity Ref");
                    ui.text_edit_singleline(&mut self.velocity_string)
                        .labelled_by(name_label.id);

                    let mut is_button_active = true;
                    match self.velocity_string.parse::<f32>() {
                        Ok(n) => self.velocity = n,
                        _ => {
                            is_button_active = false;
                        }
                    }

                    if ui
                        .add_enabled(is_button_active, egui::Button::new("Set"))
                        .clicked()
                    {
                        let _ = self.command_send.send(Command::SetParameter(
                            ParameterValue::SpeedRef(self.velocity),
                        ));
                    };
                    ui.end_row();

                    let name_label = ui.label("I_q Ref");
                    ui.text_edit_singleline(&mut self.iq_string)
                        .labelled_by(name_label.id);

                    let mut is_button_active = true;
                    match self.iq_string.parse::<f32>() {
                        Ok(n) => self.iq = n,
                        _ => {
                            is_button_active = false;
                        }
                    }

                    if ui
                        .add_enabled(is_button_active, egui::Button::new("Set"))
                        .clicked()
                    {
                        let _ = self
                            .command_send
                            .send(Command::SetParameter(ParameterValue::IqRef(self.iq)));
                    };

                    ui.end_row();

                    let name_label = ui.label("Angle K_p");
                    ui.text_edit_singleline(&mut self.angle_kp_string)
                        .labelled_by(name_label.id);

                    let mut is_button_active = true;
                    match self.angle_kp_string.parse::<f32>() {
                        Ok(n) => self.angle_kp = n,
                        _ => {
                            is_button_active = false;
                        }
                    }

                    if ui
                        .add_enabled(is_button_active, egui::Button::new("Set"))
                        .clicked()
                    {
                        let _ =
                            self.command_send
                                .send(Command::SetParameter(ParameterValue::AngleKp(
                                    self.angle_kp,
                                )));
                    };

                    ui.end_row();

                    let name_label = ui.label("Speed K_p");
                    ui.text_edit_singleline(&mut self.speed_kp_string)
                        .labelled_by(name_label.id);

                    let mut is_button_active = true;
                    match self.speed_kp_string.parse::<f32>() {
                        Ok(n) => self.speed_kp = n,
                        _ => {
                            is_button_active = false;
                        }
                    }

                    if ui
                        .add_enabled(is_button_active, egui::Button::new("Set"))
                        .clicked()
                    {
                        let _ =
                            self.command_send
                                .send(Command::SetParameter(ParameterValue::SpeedKp(
                                    self.speed_kp,
                                )));
                    };

                    ui.end_row();

                    let name_label = ui.label("Speed K_i");
                    ui.text_edit_singleline(&mut self.speed_ki_string)
                        .labelled_by(name_label.id);

                    let mut is_button_active = true;
                    match self.speed_ki_string.parse::<f32>() {
                        Ok(n) => self.speed_ki = n,
                        _ => {
                            is_button_active = false;
                        }
                    }

                    if ui
                        .add_enabled(is_button_active, egui::Button::new("Set"))
                        .clicked()
                    {
                        let _ =
                            self.command_send
                                .send(Command::SetParameter(ParameterValue::SpeedKi(
                                    self.speed_ki,
                                )));
                    };
                    ui.end_row();

                    let name_label = ui.label("Current K_p");
                    ui.text_edit_singleline(&mut self.iq_kp_string)
                        .labelled_by(name_label.id);

                    let mut is_button_active = true;
                    match self.iq_kp_string.parse::<f32>() {
                        Ok(n) => self.iq_kp = n,
                        _ => {
                            is_button_active = false;
                        }
                    }

                    if ui
                        .add_enabled(is_button_active, egui::Button::new("Set"))
                        .clicked()
                    {
                        let _ = self
                            .command_send
                            .send(Command::SetParameter(ParameterValue::CurrentKp(self.iq_kp)));
                    };

                    ui.end_row();

                    let name_label = ui.label("Current K_i");
                    ui.text_edit_singleline(&mut self.iq_ki_string)
                        .labelled_by(name_label.id);

                    let mut is_button_active = true;
                    match self.iq_ki_string.parse::<f32>() {
                        Ok(n) => self.iq_ki = n,
                        _ => {
                            is_button_active = false;
                        }
                    }

                    if ui
                        .add_enabled(is_button_active, egui::Button::new("Set"))
                        .clicked()
                    {
                        let _ = self
                            .command_send
                            .send(Command::SetParameter(ParameterValue::CurrentKi(self.iq_ki)));
                    };
                    ui.end_row();
                });

            if ui.button("disable").clicked() {
                let _ = self.command_send.send(Command::Stop);
            }
            if ui.button("enable").clicked() {
                let _ = self.command_send.send(Command::Enable);
            }

            let before = self.plot_type;
            ui.label("Plot Type");
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.plot_type, PlotType::Angle, "Angle");
                ui.selectable_value(&mut self.plot_type, PlotType::Velocity, "Velocity");
                ui.selectable_value(&mut self.plot_type, PlotType::Torque, "Torque");
            });
            if before != self.plot_type {
                self.plot_points.clear();
            }
            if ui.button("Plot").clicked() {
                if !self.is_plotting {
                    self.plot_points.clear();
                }
                self.is_plotting = !self.is_plotting;
            }

            let export_txt = format!("Export {} samples", self.plot_points.len());
            if ui
                .add_enabled(
                    !self.plot_points.is_empty(),
                    egui::Button::new(export_txt.as_str()),
                )
                .clicked()
            {
                self.file_dialog.save_file();
            }

            if let Some(path) = self.file_dialog.update(ctx).picked()
                && let Ok(mut file) = File::open(path)
            {
                for p in &self.plot_points {
                    let line = format!("{},{}\n", p.x, p.y);
                    let _ = file.write_all(line.as_bytes());
                }
            }

            egui_plot::Plot::new("plot").show(ui, |plot_ui| {
                let points = egui_plot::PlotPoints::Borrowed(&self.plot_points);
                let line = egui_plot::Line::new("Sine", points)
                    .name("Sine")
                    .style(egui_plot::LineStyle::Solid)
                    .color(egui::Color32::RED)
                    .width(2.0);
                plot_ui.line(line);
            });
            ctx.request_repaint();
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
