#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
use std::fs::File;
use std::io::prelude::Write;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use std::{f32, thread};

use can_message::message::{CanMessage, Command, CommandMessage, RunMode, StatusMessage};
use eframe::egui;
use socketcan::socket::{CanSocket, Socket};
use socketcan::{CanFrame, EmbeddedFrame, ExtendedId};

fn main() -> eframe::Result {
    env_logger::init();

    let (command_sender, command_receiver) = channel::<Command>();
    let (status_sender, status_receiver) = channel::<StatusMessage>();

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

fn forward_status(status_send: Sender<StatusMessage>, socket: CanSocket) {
    println!("CAN receiver started");

    loop {
        match socket.read_frame() {
            Ok(frame) => {
                let can_msg = can_frame_to_can_message(&frame);
                let status_msg = StatusMessage::from(can_msg);
                let _ = status_send.send(status_msg);
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
    status_recv: Receiver<StatusMessage>,

    angle_string: String,
    velocity_string: String,
    torque_string: String,

    angle: f32,
    velocity: f32,
    torque: f32,

    run_mode: RunMode,

    plot_type: PlotType,
    plot_points: Vec<egui_plot::PlotPoint>,
    is_plotting: bool,

    file_dialog: egui_file_dialog::FileDialog,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PlotType {
    Angle,
    Velocity,
    Torque,
}

impl MyApp {
    fn new(command_send: Sender<Command>, status_recv: Receiver<StatusMessage>) -> Self {
        let position = 0.0;
        let velocity = 0.0;
        let torque = 0.0;

        Self {
            command_send,
            status_recv,

            angle_string: position.to_string(),
            velocity_string: velocity.to_string(),
            torque_string: torque.to_string(),

            angle: position,
            velocity,
            torque,

            plot_type: PlotType::Angle,
            run_mode: RunMode::Impedance,
            plot_points: vec![],
            is_plotting: false,

            file_dialog: egui_file_dialog::FileDialog::new(),
        }
    }
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        for s in self.status_recv.try_iter() {
            if self.is_plotting {
                let value = match self.plot_type {
                    PlotType::Angle => s.motor_status.raw_angle as f32,
                    PlotType::Velocity => s.motor_status.raw_velocity as f32,
                    PlotType::Torque => s.motor_status.raw_torque as f32,
                };
                if let Ok(now) = SystemTime::now().duration_since(UNIX_EPOCH) {
                    if s.host_can_id != 0 {
                        continue;
                    }
                    let timestamp = now.as_nanos() as f64 / 1e9;
                    let new_point = egui_plot::PlotPoint::new(timestamp, value as f64);
                    self.plot_points.push(new_point);
                }
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
                let _ = self.command_send.send(Command::SetRunMode(self.run_mode));
            }

            egui::Grid::new("params")
                .num_columns(3)
                .spacing([40.0, 4.0])
                .show(ui, |ui| {
                    let name_label = ui.label("Angle");
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
                        let _ = self.command_send.send(Command::SetAngle(self.angle));
                    };

                    ui.end_row();

                    let name_label = ui.label("Velocity");
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
                        let _ = self.command_send.send(Command::SetVelocity(self.velocity));
                    };
                    ui.end_row();

                    let name_label = ui.label("Torque");
                    ui.text_edit_singleline(&mut self.torque_string)
                        .labelled_by(name_label.id);

                    let mut is_button_active = true;
                    match self.torque_string.parse::<f32>() {
                        Ok(n) => self.torque = n,
                        _ => {
                            is_button_active = false;
                        }
                    }

                    if ui
                        .add_enabled(is_button_active, egui::Button::new("Set"))
                        .clicked()
                    {
                        let _ = self.command_send.send(Command::SetTorque(self.torque));
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
