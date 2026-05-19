#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
use std::collections::VecDeque;
use std::fs::File;
use std::io::prelude::Write;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::thread::sleep;
use std::time::{Duration, Instant};
use std::{f32, thread};

use can_message::message::{
    CanMessage, Command, CommandMessage, DebugValueKind, FeedbackType, ParameterIndex,
    ParameterValue, ResponseBody, ResponseMessage, RunMode,
};
use eframe::egui;
use socketcan::socket::{CanSocket, Socket};
use socketcan::{CanFrame, EmbeddedFrame, ExtendedId};

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

    let rx_socket = CanSocket::open("can0").unwrap();
    let tx_socket = CanSocket::open("can0").unwrap();

    thread::spawn(move || {
        forward_status(status_sender, rx_socket);
    });
    thread::spawn(move || {
        forward_command(command_receiver, tx_socket);
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
            )))
        }),
    )
}

fn forward_command(command_recv: Receiver<CommandMessage>, socket: CanSocket) {
    loop {
        for mut command_msg in command_recv.try_iter() {
            command_msg.host_can_id = HOST_CAN_ID;
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

struct MyApp {
    command_send: Sender<CommandMessage>,
    status_recv: Receiver<ResponseMessage>,
    tabs: Vec<MotorTab>,
    selected_tab: usize,
    new_tab_can_id_string: String,
    file_dialog: egui_file_dialog::FileDialog,
    pending_export_points: Option<Vec<egui_plot::PlotPoint>>,
}

struct MotorTab {
    can_id: u8,
    rename_can_id_string: String,
    feedback_interval_string: String,

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
    plot_type: PlotType,
    plot_points_1: VecDeque<egui_plot::PlotPoint>,
    plot_points_2: VecDeque<egui_plot::PlotPoint>,
    is_plotting: bool,
    t0: Instant,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PlotType {
    Angle,
    Velocity,
    Torque,
    Current,
    SpeedError,
    IRef,
    VelocityIntegral,
}

impl MotorTab {
    fn new(can_id: u8) -> Self {
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

        Self {
            can_id,
            rename_can_id_string: can_id.to_string(),
            feedback_interval_string: "20".to_owned(),

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

            run_mode: RunMode::Impedance,
            plot_type: PlotType::Angle,
            plot_points_1: VecDeque::with_capacity(MAX_PLOT_POINTS),
            plot_points_2: VecDeque::with_capacity(MAX_PLOT_POINTS),
            is_plotting: false,
            t0: Instant::now(),
        }
    }

    fn current_feedback_type(&self) -> FeedbackType {
        match self.plot_type {
            PlotType::Current => FeedbackType::Current,
            PlotType::SpeedError => FeedbackType::SpeedError,
            PlotType::IRef => FeedbackType::IqRef,
            PlotType::VelocityIntegral => FeedbackType::VelocityIntegral,
            PlotType::Angle | PlotType::Velocity | PlotType::Torque => FeedbackType::Status,
        }
    }

    fn clear_plot(&mut self) {
        self.plot_points_1.clear();
        self.plot_points_2.clear();
        self.t0 = Instant::now();
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
                            | PlotType::IRef
                            | PlotType::VelocityIntegral
                    )
                {
                    return;
                }
                let y = match self.plot_type {
                    PlotType::Angle => x.angle as f64,
                    PlotType::Velocity => x.velocity as f64,
                    PlotType::Torque => x.torque as f64,
                    PlotType::Current
                    | PlotType::SpeedError
                    | PlotType::IRef
                    | PlotType::VelocityIntegral => return,
                };
                self.push_plot_points(y, None);
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
                    PlotType::IRef => Some(DebugValueKind::IqRef),
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
                ParameterValue::IqRef(x) => {
                    self.iq = *x;
                    self.iq_string = x.to_string();
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
}

impl MyApp {
    fn new(command_send: Sender<CommandMessage>, status_recv: Receiver<ResponseMessage>) -> Self {
        let app = Self {
            command_send,
            status_recv,
            tabs: vec![MotorTab::new(DEFAULT_MOTOR_CAN_ID)],
            selected_tab: 0,
            new_tab_can_id_string: DEFAULT_MOTOR_CAN_ID.to_string(),
            file_dialog: egui_file_dialog::FileDialog::new(),
            pending_export_points: None,
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

        self.send_command_to(current_can_id, Command::SetCanId(new_can_id));
        if let Some(tab) = self.selected_tab_mut() {
            tab.can_id = new_can_id;
            tab.rename_can_id_string = new_can_id.to_string();
            tab.clear_plot();
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
                tab.apply_response(&message.body);
            }
        }
    }
}

impl eframe::App for MyApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.handle_incoming_messages();
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
                            "I_q Ref",
                            &mut tab.iq,
                            &mut tab.iq_string,
                            &mut queued_commands,
                            ParameterValue::IqRef,
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

                ui.horizontal(|ui| {
                    if ui.button("disable").clicked() {
                        queued_commands.push(Command::Stop);
                    }
                    if ui.button("enable").clicked() {
                        queued_commands.push(Command::Enable);
                    }
                    if ui.button("Save to flash").clicked() {
                        queued_commands.push(Command::SaveParameters);
                    }
                    if ui.button("Run motor tuning").clicked() {
                        queued_commands.push(Command::RunMotorTuning);
                    }
                });

                let before = tab.plot_type;
                ui.label("Plot Type");
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut tab.plot_type, PlotType::Angle, "Angle");
                    ui.selectable_value(&mut tab.plot_type, PlotType::Velocity, "Velocity");
                    ui.selectable_value(&mut tab.plot_type, PlotType::Torque, "Torque");
                    ui.selectable_value(&mut tab.plot_type, PlotType::Current, "Current");
                    ui.selectable_value(&mut tab.plot_type, PlotType::SpeedError, "Speed Error");
                    ui.selectable_value(&mut tab.plot_type, PlotType::IRef, "I Ref");
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

                let points_1: Vec<_> = tab.plot_points_1.iter().copied().collect();
                let points_2: Vec<_> = tab.plot_points_2.iter().copied().collect();
                egui_plot::Plot::new(format!("plot-{}", tab.can_id)).show(ui, |plot_ui| {
                    let line_1 =
                        egui_plot::Line::new("y1", egui_plot::PlotPoints::Owned(points_1.clone()))
                            .name("y1")
                            .style(egui_plot::LineStyle::Solid)
                            .color(egui::Color32::RED)
                            .width(2.0);
                    plot_ui.line(line_1);

                    if tab.plot_type == PlotType::Current {
                        let line_2 = egui_plot::Line::new(
                            "y2",
                            egui_plot::PlotPoints::Owned(points_2.clone()),
                        )
                        .name("y2")
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

            if export_requested {
                self.pending_export_points = self
                    .selected_tab()
                    .map(|tab| tab.plot_points_1.iter().copied().collect::<Vec<_>>());
                self.file_dialog = egui_file_dialog::FileDialog::new();
                self.file_dialog.save_file();
            }

            let picked_path = self
                .file_dialog
                .update(ui.ctx())
                .picked()
                .map(|path| path.to_path_buf());
            if let Some(path) = picked_path
                && let Some(points) = self.pending_export_points.take()
                && let Ok(mut file) = File::create(path)
            {
                for point in &points {
                    let line = format!("{},{}\n", point.x, point.y);
                    let _ = file.write_all(line.as_bytes());
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

fn push_point(buffer: &mut VecDeque<egui_plot::PlotPoint>, point: egui_plot::PlotPoint) {
    if buffer.len() == MAX_PLOT_POINTS {
        buffer.pop_front();
    }
    buffer.push_back(point);
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
}
