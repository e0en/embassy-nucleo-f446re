#![no_std]
#![no_main]
mod adc;
mod as5047p;
mod bldc_driver;
mod calibration;
mod can;
mod clock;
mod cordic;
mod current_tuning;
mod drv8316;
mod flash_config;
mod flycat5010;
mod foc_isr;
mod gm2804;
mod gm3506;
mod motor_tuning;
mod pwm;

use core::sync::atomic::AtomicU32;

use foc::angle_input::AngleReading;

use crate::{
    as5047p::As5047P,
    bldc_driver::PwmDriver,
    can::{convert_response_message, parse_command_frame},
    cordic::{initialize_cordic, run_and_log_validation_tests, sincos},
    current_tuning::calculate_current_pi,
    drv8316::Drv8316,
};

use crate::gm3506 as motor;

use can_message::message::{
    Command, ParameterIndex, ParameterValue, ResponseBody, ResponseMessage,
};
use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    Config, bind_interrupts,
    can::{self as stm32_can, CanRx, CanTx},
    flash::Flash,
    gpio, i2c as stm32_i2c,
    interrupt::{self, InterruptExt},
    mode::Async,
    peripherals, spi as stm32_spi,
    time::mhz,
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex},
    channel::{Channel, Sender},
};
use embassy_time::{Duration, Instant, Timer};

use embedded_can::Id;
use foc::{
    angle_input::AngleInput,
    controller::{FocController, FocState, OutputLimit, RunMode},
    pwm_output::{DutyCycle3Phase, PwmOutput},
};

static ANGLE: AtomicU32 = AtomicU32::new(0);
static VELOCITY: AtomicU32 = AtomicU32::new(0);
static DT: AtomicU32 = AtomicU32::new(0);

static COMMAND_CHANNEL: Channel<ThreadModeRawMutex, Command, 32> = Channel::new();
/// Response channel uses CriticalSectionRawMutex because it's accessed from ADC interrupt
static RESPONSE_CHANNEL: Channel<CriticalSectionRawMutex, ResponseBody, 64> = Channel::new();

type SpiMutex = embassy_sync::mutex::Mutex<
    embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    Option<stm32_spi::Spi<'static, Async>>,
>;

static SPI: SpiMutex = SpiMutex::new(None);

type FlashMutex = embassy_sync::mutex::Mutex<
    embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    Option<Flash<'static, embassy_stm32::flash::Blocking>>,
>;

static FLASH: FlashMutex = FlashMutex::new(None);

type FocControllerType = FocController<fn(f32) -> (f32, f32)>;

fn create_foc_controller(psu_voltage: f32, use_current_sensing: bool) -> FocControllerType {
    let mut foc = FocController::new(
        motor::SETUP,
        psu_voltage,
        motor::CURRENT_PID,
        motor::ANGLE_PID,
        motor::VELOCITY_PID,
        OutputLimit {
            max_value: 16.0,
            ramp: 1000.0,
        },
        sincos as fn(f32) -> (f32, f32),
    );
    foc.set_current_limit(drv8316::MAX_CURRENT * 0.8); // 20% margin

    if use_current_sensing {
        foc.enable_current_sensing();
    } else {
        foc.disable_current_sensing();
    }

    foc
}

/// Apply stored configuration to FOC controller
fn apply_config(foc: &mut FocControllerType, config: &flash_config::ConfigData) {
    foc.current_mapping = config.current_mapping;
    foc.bias_angle = config.bias_angle;
    foc.sensor_direction = config.get_sensor_direction();
    foc.set_current_phase_bias(config.current_phase_bias);
    foc.set_current_offset(config.get_current_offset());

    // Apply PID gains
    foc.set_current_kp(config.current_kp);
    foc.set_current_ki(config.current_ki);
    foc.set_velocity_kp(config.velocity_kp);
    foc.set_velocity_ki(config.velocity_ki);
    foc.set_angle_kp(config.angle_kp);
}

/// Save current FOC configuration to flash
fn save_config(
    flash: &mut Flash<'_, embassy_stm32::flash::Blocking>,
    foc: &FocControllerType,
    can_id: u8,
) -> Result<(), flash_config::ConfigError> {
    let current_offset = foc.current_offset();
    let mut config = flash_config::ConfigData::new();

    config.current_mapping = foc.current_mapping;
    config.bias_angle = foc.bias_angle;
    config.set_sensor_direction(foc.sensor_direction);
    config.current_phase_bias = foc.current_phase_bias;
    config.set_current_offset(current_offset);

    config.current_kp = foc.current_q_pid.gains.p;
    config.current_ki = foc.current_q_pid.gains.i;
    config.velocity_kp = foc.velocity_pid.gains.p;
    config.velocity_ki = foc.velocity_pid.gains.i;
    config.angle_kp = foc.angle_pid.gains.p;

    config.can_id = can_id;

    flash_config::write_config(flash, &mut config)
}

/// Run motor calibration: current offset, phase mapping, sensor alignment, impedance measurement
async fn run_motor_calibration(
    foc: &mut FocControllerType,
    p_adc: &mut adc::DummyAdc<peripherals::ADC1>,
    driver: &mut PwmDriver<'_, peripherals::TIM1>,
    csa_gain: drv8316::CsaGain,
    align_voltage: f32,
    use_current_sensing: bool,
) -> Result<(), &'static str> {
    if use_current_sensing {
        let current_offset = calibration::get_current_offset(p_adc, driver, csa_gain).await;
        if let Some(m) =
            calibration::get_phase_mapping(p_adc, driver, csa_gain, current_offset, 0.05).await
        {
            info!("phase mapping = {} {} {}", m.0, m.1, m.2);
            foc.current_mapping = m;
        } else {
            return Err("Current phase mapping failed");
        }
        foc.set_current_offset(current_offset);
    }

    let set_motor = |d: DutyCycle3Phase| driver.run(d);
    let wait_seconds = async |s: f32| Timer::after(Duration::from_millis((s * 1e3) as u64)).await;

    if foc
        .align_sensor(align_voltage, read_sensor, set_motor, wait_seconds)
        .await
        .is_err()
    {
        return Err("Sensor align failed");
    }

    if use_current_sensing
        && let Some(offset) = calibration::align_current(p_adc, driver, csa_gain, foc).await
    {
        foc.set_current_phase_bias(offset);
        info!("current phase bias = {}", offset);

        let impedance =
            current_tuning::find_motor_impedance(foc, driver, p_adc, csa_gain, read_sensor).await;
        info!(
            "R_s = {}, L_q = {}, L_d = {}",
            impedance.r_s, impedance.l_q, impedance.l_d
        );
        let current_gain = calculate_current_pi(impedance, 300.0);
        info!("current kp={}, ki={}", current_gain.p, current_gain.i);
    }

    Ok(())
}

#[inline]
async fn init_foc(
    p_spi: &'static SpiMutex,
    cs_drv: gpio::Output<'static>,
    mut drvoff_pin: gpio::Output<'static>,
    timer: pwm::Pwm6Timer<
        'static,
        peripherals::TIM1,
        peripherals::PA8,
        peripherals::PA9,
        peripherals::PA10,
        peripherals::PB13,
        peripherals::PB14,
        peripherals::PB15,
    >,
    mut p_adc: adc::DummyAdc<peripherals::ADC1>,
    mut p_flash: Flash<'_, embassy_stm32::flash::Blocking>,
) -> Option<(Drv8316<'static>, u8)> {
    let use_current_sensing = true;
    let csa_gain = drv8316::CsaGain::Gain0_3V;
    let slew_rate = drv8316::SlewRate::Rate200V;
    let psu_voltage = 16.0;
    let align_voltage: f32 = 2.0;

    // Try to load stored configuration
    let stored_config = flash_config::read_config(&mut p_flash);
    let has_stored_config = stored_config.is_some();
    let can_id = stored_config.as_ref().map(|c| c.can_id).unwrap_or(0x0F);

    if has_stored_config {
        info!("Found stored calibration, skipping motor calibration");
    } else {
        info!("No stored calibration found, running motor calibration");
    }

    // Initialize hardware
    drvoff_pin.set_high();
    let mut driver = PwmDriver::new(timer.timer);
    let mut gate_driver = Drv8316::new(p_spi, cs_drv, drvoff_pin);
    gate_driver.initialize(csa_gain, slew_rate).await;

    let mut foc = create_foc_controller(psu_voltage, use_current_sensing);
    gate_driver.turn_on();

    if let Some(config) = &stored_config {
        apply_config(&mut foc, config);
        info!(
            "Loaded calibration: mapping=({},{},{}), bias_angle={}",
            config.current_mapping.0,
            config.current_mapping.1,
            config.current_mapping.2,
            config.bias_angle
        );
    } else {
        if let Err(e) = run_motor_calibration(
            &mut foc,
            &mut p_adc,
            &mut driver,
            csa_gain,
            align_voltage,
            use_current_sensing,
        )
        .await
        {
            error!("{}", e);
            gate_driver.turn_off();
            return None;
        }

        if let Err(e) = save_config(&mut p_flash, &foc, can_id) {
            warn!("Failed to save calibration to flash: {:?}", e);
        } else {
            info!("Calibration saved to flash");
        }
    }

    foc.set_run_mode(RunMode::Torque);
    foc.set_target_torque(0.0);
    foc.enable();

    if !has_stored_config {
        let kv =
            motor_tuning::find_kv_rating(&mut foc, &mut driver, &mut p_adc, csa_gain, read_sensor)
                .await;
        info!("Motor kv rating = {}", kv / core::f32::consts::TAU * 60.0);
    }

    // Initialize FOC context for interrupt-driven control
    // From this point on, FOC runs in the ADC interrupt
    foc_isr::initialize_foc_context(foc, csa_gain, use_current_sensing);
    info!("FOC interrupt-driven control started");

    // Leak the driver to prevent TIM1 from being stopped when it goes out of scope.
    // The ISR accesses TIM1 directly via PAC, but Embassy's Timer wrapper
    // may stop the hardware timer on drop.
    core::mem::forget(driver);

    Some((gate_driver, can_id))
}

#[embassy_executor::task]
async fn monitor_task(mut gate_driver: Drv8316<'static>) {
    let mut last_logged_at = Instant::now();
    loop {
        Timer::after_millis(100).await;

        let loop_count = foc_isr::get_loop_counter();
        if loop_count > 0 {
            let now = Instant::now();
            if let Some(dt) = now.checked_duration_since(last_logged_at) {
                if let Some(state) = foc_isr::get_foc_state() {
                    log_state(&state, &dt, loop_count as usize);
                }
                last_logged_at = now;
            }
        }

        if let Ok(x) = gate_driver.read_ic_status().await
            && x.device_fault
        {
            let _ = gate_driver.unlock_registers().await;
            let _ = gate_driver.clear_fault().await;
            let _ = gate_driver.lock_registers().await;
            warn!("DRV8316 status = {}", x);
        }
    }
}

#[embassy_executor::task]
async fn command_task() {
    let command_receiver = COMMAND_CHANNEL.receiver();
    let response_sender = RESPONSE_CHANNEL.sender();

    loop {
        let command = command_receiver.receive().await;

        match command {
            Command::SetFeedbackInterval(period) => {
                foc_isr::set_feedback_period(period);
            }
            Command::SetFeedbackType(typ) => {
                foc_isr::set_feedback_type(typ);
            }
            Command::Recalibrate => {
                info!("Recalibrate command received, clearing config and resetting");
                // Clear stored config and trigger system reset
                {
                    let mut flash_guard = FLASH.lock().await;
                    if let Some(flash) = flash_guard.as_mut() {
                        if let Err(e) = flash_config::clear_config(flash) {
                            error!("Failed to clear config: {:?}", e);
                        } else {
                            info!("Config cleared, resetting system");
                            cortex_m::peripheral::SCB::sys_reset();
                        }
                    }
                }
            }
            Command::SaveConfig => {
                info!("SaveConfig command received");
                // Save current FOC state to flash
                {
                    let mut flash_guard = FLASH.lock().await;
                    if let Some(flash) = flash_guard.as_mut() {
                        // Get CAN ID from stored config or use default
                        let can_id = flash_config::read_config(flash)
                            .map(|c| c.can_id)
                            .unwrap_or(0x0F);

                        foc_isr::with_foc(|foc| {
                            let current_offset = foc.current_offset();
                            let mut config = flash_config::ConfigData::new();

                            config.current_mapping = foc.current_mapping;
                            config.bias_angle = foc.bias_angle;
                            config.set_sensor_direction(foc.sensor_direction);
                            config.current_phase_bias = foc.current_phase_bias;
                            config.set_current_offset(current_offset);

                            config.current_kp = foc.current_q_pid.gains.p;
                            config.current_ki = foc.current_q_pid.gains.i;
                            config.velocity_kp = foc.velocity_pid.gains.p;
                            config.velocity_ki = foc.velocity_pid.gains.i;
                            config.angle_kp = foc.angle_pid.gains.p;

                            config.can_id = can_id;

                            if let Err(e) = flash_config::write_config(flash, &mut config) {
                                error!("Failed to save config: {:?}", e);
                            } else {
                                info!("Config saved to flash");
                            }
                        });
                    }
                }
            }
            _ => {
                handle_command_isr(&command, &response_sender).await;
            }
        }
    }
}

async fn handle_command_isr(
    command: &Command,
    response_sender: &Sender<'static, CriticalSectionRawMutex, ResponseBody, 64>,
) {
    match command {
        Command::Enable => {
            foc_isr::with_foc(|foc| foc.enable());
        }
        Command::Stop => {
            foc_isr::with_foc(|foc| foc.stop());
        }

        Command::SetParameter(ParameterValue::RunMode(m)) => {
            info!("received runmode {}", *m as u8);
            foc_isr::with_foc(|foc| foc.set_run_mode(decode_run_mode(m)));
        }
        Command::SetParameter(ParameterValue::AngleRef(p)) => {
            foc_isr::with_foc(|foc| foc.set_target_angle(*p));
        }
        Command::SetParameter(ParameterValue::SpeedRef(v)) => {
            foc_isr::with_foc(|foc| foc.set_target_velocity(*v));
        }
        Command::SetParameter(ParameterValue::IqRef(t)) => {
            foc_isr::with_foc(|foc| foc.set_target_torque(*t));
        }
        Command::SetParameter(ParameterValue::SpeedLimit(v)) => {
            foc_isr::with_foc(|foc| foc.set_velocity_limit(*v));
        }
        Command::SetParameter(ParameterValue::CurrentLimit(i)) => {
            foc_isr::with_foc(|foc| foc.set_current_limit(*i));
        }
        Command::SetParameter(ParameterValue::TorqueLimit(t)) => {
            foc_isr::with_foc(|foc| foc.set_torque_limit(*t));
        }

        Command::SetParameter(ParameterValue::CurrentKp(kp)) => {
            foc_isr::with_foc(|foc| foc.set_current_kp(*kp));
        }
        Command::SetParameter(ParameterValue::CurrentKi(ki)) => {
            foc_isr::with_foc(|foc| foc.set_current_ki(*ki));
        }

        Command::SetParameter(ParameterValue::SpeedKp(kp)) => {
            foc_isr::with_foc(|foc| foc.set_velocity_kp(*kp));
        }
        Command::SetParameter(ParameterValue::SpeedKi(ki)) => {
            foc_isr::with_foc(|foc| foc.set_velocity_ki(*ki));
        }
        Command::SetParameter(ParameterValue::Iq(f)) => {
            foc_isr::with_foc(|foc| foc.set_current_filter(*f));
        }
        Command::SetParameter(ParameterValue::AngleKp(kp)) => {
            foc_isr::with_foc(|foc| foc.set_angle_kp(*kp));
        }

        Command::SetParameter(ParameterValue::Spring(k)) => {
            foc_isr::with_foc(|foc| foc.set_spring(*k));
        }
        Command::SetParameter(ParameterValue::Damping(b)) => {
            foc_isr::with_foc(|foc| foc.set_damping(*b));
        }
        Command::SetParameter(ParameterValue::VqRef(v)) => {
            info!("received target Vq {}", v);
            foc_isr::with_foc(|foc| foc.set_target_voltage(*v));
        }

        Command::GetParameter(p) => {
            info!("parameter requested {}", *p as u16);
            let pv = foc_isr::with_foc(|foc| match p {
                ParameterIndex::RunMode => ParameterValue::RunMode(encode_run_mode(&foc.mode)),
                ParameterIndex::IqRef => ParameterValue::IqRef(foc.state.i_ref),
                ParameterIndex::Angle => ParameterValue::Angle(foc.state.angle),
                ParameterIndex::AngleRef => ParameterValue::AngleRef(foc.target.angle),
                ParameterIndex::Speed => ParameterValue::Speed(foc.state.velocity),
                ParameterIndex::SpeedRef => ParameterValue::SpeedRef(foc.target.velocity),
                ParameterIndex::SpeedLimit => {
                    ParameterValue::SpeedLimit(foc.velocity_limit.unwrap_or(0.0))
                }
                ParameterIndex::TorqueLimit => {
                    ParameterValue::TorqueLimit(foc.torque_limit.unwrap_or(0.0))
                }
                ParameterIndex::Iq => ParameterValue::Iq(foc.state.i_q),

                ParameterIndex::AngleKp => ParameterValue::AngleKp(foc.angle_pid.gains.p),
                ParameterIndex::SpeedKp => ParameterValue::SpeedKp(foc.velocity_pid.gains.p),
                ParameterIndex::SpeedKi => ParameterValue::SpeedKi(foc.velocity_pid.gains.i),

                ParameterIndex::CurrentKp => ParameterValue::CurrentKp(foc.current_q_pid.gains.p),
                ParameterIndex::CurrentKi => ParameterValue::CurrentKi(foc.current_q_pid.gains.i),
                ParameterIndex::CurrentFilter => {
                    ParameterValue::CurrentFilter(foc.current_q_filter.time_constant)
                }
                ParameterIndex::CurrentLimit => {
                    ParameterValue::CurrentLimit(foc.current_limit.unwrap_or(0.0))
                }
                ParameterIndex::Spring => ParameterValue::Spring(foc.target.spring),
                ParameterIndex::Damping => ParameterValue::Damping(foc.target.damping),
                ParameterIndex::VqRef => ParameterValue::Damping(foc.target.voltage),
            });
            if let Some(pv) = pv {
                let body = ResponseBody::ParameterValue(pv);
                response_sender.send(body).await;
            }
        }
        _ => (),
    }
}

fn log_state(state: &FocState, dt: &Duration, check_count: usize) {
    let dt_seconds = (dt.as_micros() as f32) / 1e6;
    let freq = (check_count as f32) / dt_seconds;
    info!(
        "MEASURED: a={}, v={}, t={}, {} Hz",
        state.angle, state.velocity, state.v_q, freq
    );
}

#[embassy_executor::task]
async fn encoder_task(mut sensor: As5047P<'static>) {
    loop {
        match sensor.read_async().await {
            Ok(reading) => {
                ANGLE.store(
                    u32::from_le_bytes(reading.angle.to_le_bytes()),
                    core::sync::atomic::Ordering::Relaxed,
                );
                VELOCITY.store(
                    u32::from_le_bytes(reading.velocity.to_le_bytes()),
                    core::sync::atomic::Ordering::Relaxed,
                );
                DT.store(
                    u32::from_le_bytes(reading.dt.to_le_bytes()),
                    core::sync::atomic::Ordering::Relaxed,
                );
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
        Timer::after_micros(50).await;
    }
}

fn read_sensor() -> AngleReading {
    let raw_angle = ANGLE.load(core::sync::atomic::Ordering::Relaxed);
    let raw_velocity = VELOCITY.load(core::sync::atomic::Ordering::Relaxed);
    let raw_dt = DT.load(core::sync::atomic::Ordering::Relaxed);

    AngleReading {
        angle: f32::from_le_bytes(raw_angle.to_le_bytes()),
        velocity: f32::from_le_bytes(raw_velocity.to_le_bytes()),
        dt: f32::from_le_bytes(raw_dt.to_le_bytes()),
    }
}

#[embassy_executor::task]
async fn can_tx_task(mut can_tx: CanTx<'static>) {
    let can_id: u8 = 0x0F;
    let response_receiver = RESPONSE_CHANNEL.receiver();
    let host_id: Option<u8> = Some(0);

    loop {
        let r = response_receiver.receive().await;
        if let Some(h) = host_id {
            let message = ResponseMessage::new(can_id, h, r);
            if let Ok(frame) = convert_response_message(message) {
                can_tx.write(&frame).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn can_rx_task(mut can_rx: CanRx<'static>, initial_can_id: u8) {
    let mut can_id: u8 = initial_can_id;
    let command_sender = COMMAND_CHANNEL.sender();

    loop {
        if let Ok(m) = can_rx.read().await {
            if let Ok(message) = parse_command_frame(m.frame) {
                if message.motor_can_id != can_id {
                    continue;
                }
                match message.command {
                    Command::SetCanId(new_can_id) => {
                        can_id = new_can_id;
                        info!("CAN ID changed to {}", new_can_id);
                        // Save new CAN ID to flash
                        {
                            let mut flash_guard = FLASH.lock().await;
                            if let Some(flash) = flash_guard.as_mut()
                                && let Some(mut config) = flash_config::read_config(flash)
                            {
                                config.can_id = new_can_id;
                                if let Err(e) = flash_config::write_config(flash, &mut config) {
                                    warn!("Failed to save CAN ID to flash: {:?}", e);
                                } else {
                                    info!("CAN ID saved to flash");
                                }
                            }
                        }
                    }
                    cmd => {
                        if command_sender.try_send(cmd).is_err() {
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

bind_interrupts!(
    struct Irqs {
        I2C1_EV => stm32_i2c::EventInterruptHandler<peripherals::I2C1>;
        I2C1_ER => stm32_i2c::ErrorInterruptHandler<peripherals::I2C1>;
        FDCAN1_IT1 => stm32_can::IT1InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
        FDCAN1_IT0 => stm32_can::IT0InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Delay for probe-rs to attach during firmware flashing
    cortex_m::asm::delay(160_000); // ~10ms at 16MHz HSI

    let mut config = Config::default();
    clock::set_clock(&mut config);
    initialize_cordic();
    run_and_log_validation_tests();
    let p = embassy_stm32::init(config);
    clock::print_clock_info(&p.RCC);

    embassy_stm32::interrupt::TIM1_UP_TIM16.set_priority(interrupt::Priority::P0);
    embassy_stm32::interrupt::ADC1_2.set_priority(interrupt::Priority::P1);
    embassy_stm32::interrupt::FDCAN1_IT0.set_priority(interrupt::Priority::P7);
    embassy_stm32::interrupt::FDCAN1_IT1.set_priority(interrupt::Priority::P7);

    let mut spi_config = stm32_spi::Config::default();
    spi_config.miso_pull = gpio::Pull::Up;
    spi_config.mode = stm32_spi::MODE_1;
    spi_config.bit_order = stm32_spi::BitOrder::MsbFirst;
    spi_config.frequency = mhz(10);
    spi_config.gpio_speed = gpio::Speed::VeryHigh;
    let cs_out = gpio::Output::new(p.PB6, gpio::Level::High, gpio::Speed::VeryHigh);

    {
        let p_spi = stm32_spi::Spi::new(
            p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH4, spi_config,
        );
        *(SPI.lock().await) = Some(p_spi);
    }

    let mut can_conf = stm32_can::CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    can_conf.set_bitrate(1_000_000);
    can_conf.properties().set_extended_filter(
        stm32_can::filter::ExtendedFilterSlot::_0,
        stm32_can::filter::ExtendedFilter::accept_all_into_fifo1(),
    );
    let p_can = can_conf.into_normal_mode();
    let (can_tx, can_rx, _properties) = p_can.split();

    let drvoff_pin = gpio::Output::new(p.PB12, gpio::Level::High, gpio::Speed::Medium);
    let cs_drv = gpio::Output::new(p.PB5, gpio::Level::High, gpio::Speed::VeryHigh);

    let mut p_adc = adc::DummyAdc::new(p.ADC1, p.PA0, p.PA1, p.PC2);
    p_adc.calibrate();
    p_adc.initialize(1, 2, 8);

    let mut timer = pwm::Pwm6Timer::new(p.TIM1, p.PA8, p.PA9, p.PA10, p.PB13, p.PB14, p.PB15);
    timer.initialize(foc_isr::MAX_COMPARE_VALUE as u16);
    info!("Timer frequency: {}", timer.get_frequency());

    let velocity_filter: f32 = 0.005;
    let mut sensor = As5047P::new(&SPI, cs_out, velocity_filter);
    if let Err(e) = sensor.initialize().await {
        error!("Sensor initialization failed, {:?}", e);
        return;
    }

    // Create flash peripheral for config storage
    let p_flash = Flash::new_blocking(p.FLASH);

    // Start encoder task early so sensor readings are available during FOC init
    unwrap!(spawner.spawn(encoder_task(sensor)));

    let Some((gate_driver, can_id)) =
        init_foc(&SPI, cs_drv, drvoff_pin, timer, p_adc, p_flash).await
    else {
        return;
    };

    // Store flash in static for use by command handlers
    {
        let flash = Flash::new_blocking(unsafe {
            // SAFETY: We're taking ownership of FLASH again after init_foc is done
            // This is safe because init_foc doesn't hold a reference to flash after returning
            embassy_stm32::Peripherals::steal().FLASH
        });
        *(FLASH.lock().await) = Some(flash);
    }

    unwrap!(spawner.spawn(monitor_task(gate_driver)));
    unwrap!(spawner.spawn(command_task()));
    unwrap!(spawner.spawn(can_rx_task(can_rx, can_id)));
    unwrap!(spawner.spawn(can_tx_task(can_tx)));
}
