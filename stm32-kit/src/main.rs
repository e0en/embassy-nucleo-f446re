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
mod drv8316c;
mod drv8316t;
mod encoder_correction;
mod flash_config;
mod flycat5010;
mod foc_isr;
mod gm2804;
mod gm3506;
mod motor_tuning;
mod pwm;
mod velocity_tuning;

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use foc::encoder::EncoderReading;

use crate::{
    adc::measure_vm_sense,
    as5047p::As5047P,
    bldc_driver::PwmDriver,
    can::{convert_response_message, parse_command_frame},
    cordic::{initialize_cordic, run_and_log_validation_tests, sincos},
    current_tuning::calculate_current_pi,
    encoder_correction::{
        AngleTracker, runtime_snapshot, runtime_version, set_runtime_correction, wrap_0_tau,
    },
};

use crate::gm3506 as motor;

const PSU_VOLTAGE: f32 = 16.0;
const VOLTAGE_RAMP_RATE: f32 = 1000.0;
const CURRENT_SAFETY_MARGIN: f32 = 0.8;
const PHASE_MAPPING_TOLERANCE: f32 = 0.05;
/// Must stay below current filter cutoff (1/(2π×0.001) ≈ 159 Hz)
const CURRENT_PI_FREQUENCY: f32 = 100.0;
const VELOCITY_PI_FREQUENCY: f32 = 3.0;
const ALIGN_VOLTAGE: f32 = 2.0;
const ALIGN_VOLTAGE_SEARCH_START: f32 = 0.1;
const ALIGN_VOLTAGE_SEARCH_STEP: f32 = 0.1;
const ALIGN_VOLTAGE_SEARCH_SETTLE_MS: u64 = 200;
const ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_RAD: f32 = 0.01;
const ALIGN_VOLTAGE_SEARCH_SWEEP_STEPS: usize = 40;
const ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_MS: u64 = 10;
const ALIGN_VOLTAGE_SEARCH_DELTA_EPSILON_RAD: f32 = 0.0005;
const ALIGN_VOLTAGE_SEARCH_MIN_DOMINANT_STEPS: usize = 16;
const ALIGN_VOLTAGE_SEARCH_MIN_NET_ANGLE_RAD: f32 = 0.005;
const INIT_DELAY_CYCLES: u32 = 160_000;
const CAN_BITRATE: u32 = 1_000_000;
const CAN_RECOVERY_REINIT_INTERVAL_MS: u64 = 100;
const VELOCITY_OBSERVER_BANDWIDTH: f32 = 20.0;
const CAN_INTERRUPT_PRIORITY: interrupt::Priority = interrupt::Priority::P7;
const DEFAULT_HOST_CAN_ID: u8 = 0;
const DRV_FAULT_POLL_INTERVAL_MS: u64 = 10;
const AS5047P_RAW_TO_RADIAN: f32 = 2.0 * core::f32::consts::PI / ((1 << 14) as f32);
const IMPEDANCE_TUNING_MAX_CURRENT: f32 = 1.5;
const USE_SECONDARY_ENCODER: bool = true;

use can_message::message::{
    Command, ParameterIndex, ParameterValue, ResponseBody, ResponseMessage,
};
use {defmt_rtt as _, panic_probe as _};

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

macro_rules! dispatch_get_param {
    ($p:expr, $( $variant:ident => $value:expr ),* $(,)?) => {
        match $p {
            $(
                ParameterIndex::$variant => ParameterValue::$variant($value),
            )*
        }
    };
}

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
    mutex::Mutex,
};
use embassy_time::{Duration, Instant, Timer};

use embedded_can::Id;
use foc::{
    controller::{FocController, FocState, OutputLimit, RunMode},
    pwm_output::{DutyCycle3Phase, PwmOutput},
    svpwm,
};

struct SensorSnapshotBuffer {
    angle: AtomicU32,
    phase_angle: AtomicU32,
    velocity: AtomicU32,
    dt: AtomicU32,
}

impl SensorSnapshotBuffer {
    const fn new() -> Self {
        Self {
            angle: AtomicU32::new(0),
            phase_angle: AtomicU32::new(0),
            velocity: AtomicU32::new(0),
            dt: AtomicU32::new(0),
        }
    }
}

static SENSOR_SNAPSHOTS: [SensorSnapshotBuffer; 2] =
    [SensorSnapshotBuffer::new(), SensorSnapshotBuffer::new()];
static ACTIVE_SENSOR_SNAPSHOT: AtomicU8 = AtomicU8::new(0);
static DRV_FAULT_ACTIVE: AtomicBool = AtomicBool::new(false);
static CAN_FAULT_ACTIVE: AtomicBool = AtomicBool::new(false);
static MOTOR_DISARMED_BY_FAULT: AtomicBool = AtomicBool::new(false);
static ANGLE_2: AtomicU32 = AtomicU32::new(0);
static SECONDARY_RAW_ANGLE: AtomicU32 = AtomicU32::new(0);
static CAN_ID: AtomicU8 = AtomicU8::new(0);
static PRIMARY_ZERO_OFFSET: AtomicU32 = AtomicU32::new(0);
static SECONDARY_ZERO_OFFSET: AtomicU32 = AtomicU32::new(0);
static ZERO_OFFSET_VERSION: AtomicU32 = AtomicU32::new(1);

static COMMAND_CHANNEL: Channel<ThreadModeRawMutex, can_message::message::CommandMessage, 32> =
    Channel::new();
/// Response channel uses CriticalSectionRawMutex because it's accessed from ADC interrupt
static RESPONSE_CHANNEL: Channel<CriticalSectionRawMutex, QueuedResponse, 64> = Channel::new();

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

type GateDriverMutex = Mutex<
    embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    Option<drv8316t::Drv8316T<'static>>,
>;

static GATE_DRIVER: GateDriverMutex = GateDriverMutex::new(None);

type CanPropertiesMutex =
    Mutex<embassy_sync::blocking_mutex::raw::ThreadModeRawMutex, Option<stm32_can::Properties>>;

static CAN_PROPERTIES: CanPropertiesMutex = CanPropertiesMutex::new(None);

type FocControllerType = FocController<fn(f32) -> (f32, f32)>;

#[derive(Clone, Copy)]
struct QueuedResponse {
    host_can_id: u8,
    body: ResponseBody,
}

fn create_foc_controller(use_current_sensing: bool) -> FocControllerType {
    let mut foc = FocController::new(
        motor::SETUP,
        motor::CURRENT_PID,
        motor::ANGLE_PID,
        motor::VELOCITY_PID,
        OutputLimit {
            max_value: PSU_VOLTAGE,
            ramp: VOLTAGE_RAMP_RATE,
        },
        sincos as fn(f32) -> (f32, f32),
    );
    foc.set_driver_current_limit(drv8316::MAX_CURRENT * CURRENT_SAFETY_MARGIN);

    if use_current_sensing {
        foc.enable_current_sensing();
    } else {
        foc.disable_current_sensing();
    }

    foc
}

/// Run motor calibration: current offset, phase mapping, sensor alignment, impedance measurement
async fn run_motor_calibration(
    foc: &mut FocControllerType,
    p_adc: &mut adc::DummyAdc<peripherals::ADC1>,
    driver: &mut PwmDriver<'_, peripherals::TIM1>,
    gate_driver: &mut drv8316t::Drv8316T<'static>,
    csa_gain: drv8316::CsaGain,
    use_current_sensing: bool,
) -> Result<f32, &'static str> {
    if use_current_sensing {
        let current_offset = calibration::get_current_offset(p_adc, driver, csa_gain).await;
        if let Some(m) = calibration::get_phase_mapping(
            p_adc,
            driver,
            csa_gain,
            current_offset,
            PHASE_MAPPING_TOLERANCE,
        )
        .await
        {
            info!("phase mapping = {} {} {}", m.0, m.1, m.2);
            foc.current_mapping = m;
        } else {
            return Err("Current phase mapping failed");
        }
        foc.set_current_offset(current_offset);
    }

    let align_voltage = find_minimum_align_voltage(driver, measure_vm_sense()).await?;
    info!("Using align voltage {}", align_voltage);
    let wait_seconds = async |s: f32| Timer::after(Duration::from_millis((s * 1e3) as u64)).await;

    match foc
        .detect_pole_pairs(align_voltage, read_sensor, |d| driver.run(d), wait_seconds)
        .await
    {
        Ok(pp) => info!("Detected {} pole pairs", pp),
        Err(_) => warn!(
            "Pole pair detection failed, using default: {}",
            foc.motor.pole_pair_count
        ),
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
        && let Some(offset) =
            calibration::align_current(p_adc, driver, csa_gain, foc, align_voltage).await
    {
        foc.set_current_phase_bias(offset);
        info!("current phase bias = {}", offset);

        let impedance = current_tuning::find_motor_impedance(
            foc,
            driver,
            p_adc,
            csa_gain,
            read_sensor,
            IMPEDANCE_TUNING_MAX_CURRENT,
            || gate_driver.turn_off(),
        )
        .await
        .map_err(|err| match err {
            current_tuning::ImpedanceError::Overcurrent => {
                "Motor impedance measurement overcurrent"
            }
            current_tuning::ImpedanceError::DutyGenerationFailed => {
                "Motor impedance measurement failed"
            }
        })?;
        foc.motor.phase_resistance = impedance.r_s;
        info!(
            "R_s = {}, L_q = {}, L_d = {}",
            impedance.r_s, impedance.l_q, impedance.l_d
        );
        let current_gain = calculate_current_pi(impedance, CURRENT_PI_FREQUENCY);
        foc.set_current_d_kp(current_gain.d.p);
        foc.set_current_d_ki(current_gain.d.i);
        foc.set_current_q_kp(current_gain.q.p);
        foc.set_current_q_ki(current_gain.q.i);
        info!(
            "current d: kp={}, ki={} / q: kp={}, ki={}",
            current_gain.d.p, current_gain.d.i, current_gain.q.p, current_gain.q.i
        );
    }

    Ok(align_voltage)
}

async fn find_minimum_align_voltage(
    driver: &mut PwmDriver<'_, peripherals::TIM1>,
    psu_voltage: f32,
) -> Result<f32, &'static str> {
    const LOCK_ANGLE: f32 = 3.0 * core::f32::consts::FRAC_PI_2;

    if psu_voltage <= 0.0 {
        return Err("Invalid PSU voltage for align search");
    }

    let mut candidate_voltage = ALIGN_VOLTAGE_SEARCH_START;
    while candidate_voltage <= ALIGN_VOLTAGE {
        let lock_signal = svpwm(candidate_voltage, 0.0, LOCK_ANGLE, psu_voltage, sincos)
            .map_err(|_| "Failed to generate align search lock signal")?;
        driver.run(lock_signal);
        Timer::after_millis(ALIGN_VOLTAGE_SEARCH_SETTLE_MS).await;

        let mut electrical_angle = LOCK_ANGLE;
        let mut previous_angle = read_sensor().angle;
        let mut forward_positive_steps = 0;
        let mut forward_negative_steps = 0;
        let mut forward_net_angle = 0.0;

        for _ in 0..ALIGN_VOLTAGE_SEARCH_SWEEP_STEPS {
            electrical_angle += ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_RAD;
            let signal = svpwm(
                candidate_voltage,
                0.0,
                electrical_angle,
                psu_voltage,
                sincos,
            )
            .map_err(|_| "Failed to generate align search sweep signal")?;
            driver.run(signal);
            Timer::after_millis(ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_MS).await;
            let angle = read_sensor().angle;
            let delta = angle - previous_angle;
            if delta >= ALIGN_VOLTAGE_SEARCH_DELTA_EPSILON_RAD {
                forward_positive_steps += 1;
            } else if delta <= -ALIGN_VOLTAGE_SEARCH_DELTA_EPSILON_RAD {
                forward_negative_steps += 1;
            }
            forward_net_angle += delta;
            previous_angle = angle;
        }

        let mut backward_positive_steps = 0;
        let mut backward_negative_steps = 0;
        let mut backward_net_angle = 0.0;
        for _ in 0..ALIGN_VOLTAGE_SEARCH_SWEEP_STEPS {
            electrical_angle -= ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_RAD;
            let signal = svpwm(
                candidate_voltage,
                0.0,
                electrical_angle,
                psu_voltage,
                sincos,
            )
            .map_err(|_| "Failed to generate align search sweep signal")?;
            driver.run(signal);
            Timer::after_millis(ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_MS).await;
            let angle = read_sensor().angle;
            let delta = angle - previous_angle;
            if delta <= -ALIGN_VOLTAGE_SEARCH_DELTA_EPSILON_RAD {
                backward_negative_steps += 1;
            } else if delta >= ALIGN_VOLTAGE_SEARCH_DELTA_EPSILON_RAD {
                backward_positive_steps += 1;
            }
            backward_net_angle += delta;
            previous_angle = angle;
        }

        driver.run(DutyCycle3Phase::zero());
        let forward_dominant_positive = forward_positive_steps > forward_negative_steps;
        let backward_dominant_positive = backward_positive_steps > backward_negative_steps;
        let forward_dominant_steps = if forward_dominant_positive {
            forward_positive_steps
        } else {
            forward_negative_steps
        };
        let backward_dominant_steps = if backward_dominant_positive {
            backward_positive_steps
        } else {
            backward_negative_steps
        };
        info!(
            "Align search {}V: fwd(+/-)={}/{} fwd_net={} bwd(+/-)={}/{} bwd_net={}",
            candidate_voltage,
            forward_positive_steps,
            forward_negative_steps,
            forward_net_angle,
            backward_positive_steps,
            backward_negative_steps,
            backward_net_angle
        );

        if forward_dominant_steps >= ALIGN_VOLTAGE_SEARCH_MIN_DOMINANT_STEPS
            && backward_dominant_steps >= ALIGN_VOLTAGE_SEARCH_MIN_DOMINANT_STEPS
            && forward_net_angle.abs() >= ALIGN_VOLTAGE_SEARCH_MIN_NET_ANGLE_RAD
            && backward_net_angle.abs() >= ALIGN_VOLTAGE_SEARCH_MIN_NET_ANGLE_RAD
            && forward_net_angle * backward_net_angle < 0.0
            && forward_dominant_positive != backward_dominant_positive
        {
            return Ok(candidate_voltage);
        }

        candidate_voltage += ALIGN_VOLTAGE_SEARCH_STEP;
    }

    Err("Failed to find align voltage with directional response")
}

#[inline]
async fn init_foc(
    drvoff_pin: gpio::Output<'static>,
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
    p_flash: &mut Flash<'static, embassy_stm32::flash::Blocking>,
    stored_config: Option<flash_config::ConfigData>,
) -> Option<u8> {
    let use_current_sensing = true;
    let csa_gain = drv8316::CsaGain::Gain0_3V;

    let has_stored_config = stored_config.is_some();
    let can_id = stored_config.as_ref().map(|c| c.can_id).unwrap_or(0x0F);

    if has_stored_config {
        info!("Found stored calibration, skipping motor calibration");
    } else {
        info!("No stored calibration found, running motor calibration");
    }

    // Initialize hardware
    let mut driver = PwmDriver::new(timer.timer);
    let mut gate_driver = drv8316t::Drv8316T::new(drvoff_pin);
    Timer::after_millis(1).await; // ready time of gate driver

    let mut foc = create_foc_controller(use_current_sensing);
    gate_driver.turn_on();

    foc.set_psu_voltage(measure_vm_sense());

    if let Some(config) = &stored_config {
        flash_config::apply_to_foc(&mut foc, config);
        set_runtime_correction(config.get_encoder_correction());
        write_zero_offsets(config.primary_zero_offset, config.secondary_zero_offset);
        info!(
            "Loaded calibration: mapping=({},{},{}), bias_angle={}",
            config.current_mapping.0,
            config.current_mapping.1,
            config.current_mapping.2,
            config.bias_angle
        );
    } else {
        write_zero_offsets(0.0, 0.0);
        let align_voltage = match run_motor_calibration(
            &mut foc,
            &mut p_adc,
            &mut driver,
            &mut gate_driver,
            csa_gain,
            use_current_sensing,
        )
        .await
        {
            Ok(v) => v,
            Err(e) => {
                error!("{}", e);
                gate_driver.turn_off();
                return None;
            }
        };

        match calibration::calibrate_encoder_correction(&mut driver, &foc, align_voltage).await {
            Ok(correction) => {
                set_runtime_correction(correction);
                info!("Encoder LUT calibration complete");
            }
            Err(e) => warn!("{}", e),
        }

        foc.enable();

        if let Some(velocity_gain) = velocity_tuning::tune_velocity_pi(
            &mut foc,
            &mut driver,
            &mut p_adc,
            csa_gain,
            read_sensor,
            VELOCITY_PI_FREQUENCY,
        )
        .await
        {
            foc.set_velocity_kp(velocity_gain.p);
            foc.set_velocity_ki(velocity_gain.i);
            info!("velocity kp={}, ki={}", velocity_gain.p, velocity_gain.i);
        } else {
            warn!("Velocity PI tuning failed, using defaults");
        }

        let kv =
            motor_tuning::find_kv_rating(&mut foc, &mut driver, &mut p_adc, csa_gain, read_sensor)
                .await;
        foc.motor.kv_rating = kv;
        info!("Motor kv rating = {}", kv / core::f32::consts::TAU * 60.0);

        let mut config = flash_config::from_foc(&foc, can_id);
        let (primary_zero_offset, secondary_zero_offset) = read_zero_offsets();
        config.primary_zero_offset = primary_zero_offset;
        config.secondary_zero_offset = secondary_zero_offset;
        config.set_encoder_correction(runtime_snapshot());
        if let Err(e) = flash_config::write_config(p_flash, &mut config) {
            warn!("Failed to save calibration to flash: {:?}", e);
        } else {
            info!("Calibration saved to flash");
        }
    }

    foc.set_run_mode(RunMode::Torque);
    foc.set_target_torque(0.0);
    foc.enable();

    // Initialize FOC context for interrupt-driven control
    // From this point on, FOC runs in the ADC interrupt
    foc_isr::initialize_foc_context(foc, csa_gain, use_current_sensing);
    info!("FOC interrupt-driven control started");

    *(GATE_DRIVER.lock().await) = Some(gate_driver);

    // Leak the driver to prevent TIM1 from being stopped when it goes out of scope.
    // The ISR accesses TIM1 directly via PAC, but Embassy's Timer wrapper
    // may stop the hardware timer on drop.
    core::mem::forget(driver);

    Some(can_id)
}

#[embassy_executor::task]
async fn monitor_task() {
    let mut last_logged_at = Instant::now();
    let mut psu_voltage_tick: u8 = 0;
    let mut can_diag_tick: u8 = 0;
    loop {
        Timer::after_millis(100).await;

        psu_voltage_tick += 1;
        if psu_voltage_tick >= 10 {
            foc_isr::with_foc(|foc| foc.set_psu_voltage(adc::measure_vm_sense()));
            psu_voltage_tick = 0;
        }

        can_diag_tick += 1;
        if can_diag_tick >= 10 {
            can_diag_tick = 0;
            log_fdcan_status();
        }

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
    }
}

fn set_fault_led(led: &mut gpio::Output<'static>, fault_active: bool) {
    if fault_active {
        led.set_high();
    } else {
        led.set_low();
    }
}

#[embassy_executor::task]
async fn drv_fault_monitor_task(
    fault_pin: gpio::Input<'static>,
    mut led_fault: gpio::Output<'static>,
) {
    let mut last_drv_fault_active = false;
    let mut last_can_fault_active = false;
    let mut last_led_active = false;
    let mut last_can_reinit_at: Option<Instant> = None;
    set_fault_led(&mut led_fault, last_led_active);

    loop {
        let drv_fault_active = fault_pin.is_low();
        DRV_FAULT_ACTIVE.store(drv_fault_active, Ordering::Relaxed);
        if drv_fault_active != last_drv_fault_active {
            if drv_fault_active {
                warn!("DRV8316 fault asserted");
                disarm_motor_due_to_fault().await;
            } else {
                info!("DRV8316 fault cleared");
            }
            last_drv_fault_active = drv_fault_active;
        }

        let can_status = read_fdcan_status();
        let can_fault_active = can_status.has_fault();
        CAN_FAULT_ACTIVE.store(can_fault_active, Ordering::Relaxed);
        if can_fault_active != last_can_fault_active {
            if can_fault_active {
                warn!(
                    "CAN fault detected: {} TEC={} REC={} last_err={}",
                    can_status.bus_state,
                    can_status.tx_error_count,
                    can_status.rx_error_count,
                    can_status.last_error,
                );
                log_fdcan_registers("fault");
                disarm_motor_due_to_fault().await;
            } else {
                info!("CAN fault cleared");
                log_fdcan_registers("cleared");
            }
            last_can_fault_active = can_fault_active;
        }

        if can_fault_active {
            let now = Instant::now();
            let should_reinitialize = last_can_reinit_at.is_none_or(|last_reinit_at| {
                now.checked_duration_since(last_reinit_at)
                    .is_some_and(|dt| dt >= Duration::from_millis(CAN_RECOVERY_REINIT_INTERVAL_MS))
            });

            if should_reinitialize {
                warn!(
                    "Reinitializing CAN after fault: {} TEC={} REC={} last_err={}",
                    can_status.bus_state,
                    can_status.tx_error_count,
                    can_status.rx_error_count,
                    can_status.last_error,
                );
                reinitialize_fdcan();
                log_fdcan_registers("reinit");
                last_can_reinit_at = Some(now);
            }
        } else {
            last_can_reinit_at = None;
        }

        let led_active = drv_fault_active || can_fault_active;
        if led_active != last_led_active {
            set_fault_led(&mut led_fault, led_active);
            last_led_active = led_active;
        }

        Timer::after_millis(DRV_FAULT_POLL_INTERVAL_MS).await;
    }
}

#[derive(defmt::Format, Clone, Copy)]
enum BusState {
    Ok,
    ErrorWarning,
    ErrorPassive,
    BusOff,
}

impl BusState {
    fn is_ok(&self) -> bool {
        matches!(self, BusState::Ok)
    }
}

#[derive(defmt::Format, Clone, Copy, PartialEq, Eq)]
enum CanLastError {
    None,
    Stuff,
    Form,
    Ack,
    Bit1Recessive,
    Bit0Dominant,
    Crc,
    NoChange,
}

impl CanLastError {
    fn is_fault(&self) -> bool {
        !matches!(self, CanLastError::None | CanLastError::NoChange)
    }
}

#[derive(defmt::Format, Clone, Copy)]
struct FdcanStatus {
    bus_state: BusState,
    tx_error_count: u8,
    rx_error_count: u8,
    last_error: CanLastError,
}

impl FdcanStatus {
    fn has_fault(&self) -> bool {
        !self.bus_state.is_ok()
            || self.tx_error_count > 0
            || self.rx_error_count > 0
            || self.last_error.is_fault()
    }
}

#[derive(defmt::Format, Clone, Copy)]
struct FdcanRegisters {
    cccr: u32,
    nbtp: u32,
    psr: u32,
    ecr: u32,
    ir: u32,
}

fn read_fdcan_registers() -> FdcanRegisters {
    use embassy_stm32::pac;

    let fdcan = pac::FDCAN1;
    FdcanRegisters {
        cccr: fdcan.cccr().read().0,
        nbtp: fdcan.nbtp().read().0,
        psr: fdcan.psr().read().0,
        ecr: fdcan.ecr().read().0,
        ir: fdcan.ir().read().0,
    }
}

fn log_fdcan_registers(context: &'static str) {
    let regs = read_fdcan_registers();
    info!(
        "CAN regs[{}]: CCCR={=u32:#010x} NBTP={=u32:#010x} PSR={=u32:#010x} ECR={=u32:#010x} IR={=u32:#010x}",
        context, regs.cccr, regs.nbtp, regs.psr, regs.ecr, regs.ir,
    );
}

fn can_id_filter(can_id: u8) -> stm32_can::filter::ExtendedFilter {
    stm32_can::filter::ExtendedFilter {
        filter: stm32_can::filter::FilterType::BitMask {
            filter: can_id as u32,
            mask: 0xFF,
        },
        action: stm32_can::filter::Action::StoreInFifo1,
    }
}

async fn update_can_hardware_filter(can_id: u8) {
    let guard = CAN_PROPERTIES.lock().await;
    if let Some(properties) = guard.as_ref() {
        properties.set_extended_filter(
            stm32_can::filter::ExtendedFilterSlot::_0,
            can_id_filter(can_id),
        );
    }
}

#[derive(Clone, Copy)]
struct FdcanRecoveryConfig {
    nbtp: u32,
    tscc: u32,
    txbtie: u32,
    ie: u32,
    ile: u32,
    rxgfc: u32,
}

fn read_fdcan_recovery_config() -> FdcanRecoveryConfig {
    use embassy_stm32::pac;

    let fdcan = pac::FDCAN1;
    FdcanRecoveryConfig {
        nbtp: fdcan.nbtp().read().0,
        tscc: fdcan.tscc().read().0,
        txbtie: fdcan.txbtie().read().0,
        ie: fdcan.ie().read().0,
        ile: fdcan.ile().read().0,
        rxgfc: fdcan.rxgfc().read().0,
    }
}

fn reinitialize_fdcan() {
    use embassy_stm32::pac;

    let config = read_fdcan_recovery_config();

    critical_section::with(|_| {
        let rcc = pac::RCC;
        let fdcan = pac::FDCAN1;

        rcc.apb1enr1().modify(|w| w.set_fdcanen(true));
        rcc.apb1rstr1().modify(|w| w.set_fdcanrst(true));
        rcc.apb1rstr1().modify(|w| w.set_fdcanrst(false));

        fdcan.cccr().modify(|w| w.set_init(true));
        while !fdcan.cccr().read().init() {}

        fdcan.cccr().modify(|w| w.set_cce(true));
        fdcan.cccr().modify(|w| {
            w.set_test(false);
            w.set_mon(false);
            w.set_asm(false);
        });
        fdcan.nbtp().write(|w| w.0 = config.nbtp);
        fdcan.tscc().write(|w| w.0 = config.tscc);
        fdcan.txbtie().write(|w| w.0 = config.txbtie);
        fdcan.ie().write(|w| w.0 = config.ie);
        fdcan.ile().write(|w| w.0 = config.ile);
        fdcan.rxgfc().write(|w| w.0 = config.rxgfc);
        fdcan.ir().write(|w| w.0 = u32::MAX);
        fdcan.cccr().modify(|w| w.set_cce(false));
        fdcan.cccr().modify(|w| w.set_init(false));
        while fdcan.cccr().read().init() {}
    });
}

fn read_fdcan_status() -> FdcanStatus {
    use embassy_stm32::pac;

    let fdcan = pac::FDCAN1;
    let psr = fdcan.psr().read();
    let ecr = fdcan.ecr().read();

    let bus_state = if psr.bo() {
        BusState::BusOff
    } else if psr.ep() {
        BusState::ErrorPassive
    } else if psr.ew() {
        BusState::ErrorWarning
    } else {
        BusState::Ok
    };

    let last_error = match psr.lec().to_bits() {
        0 => CanLastError::None,
        1 => CanLastError::Stuff,
        2 => CanLastError::Form,
        3 => CanLastError::Ack,
        4 => CanLastError::Bit1Recessive,
        5 => CanLastError::Bit0Dominant,
        6 => CanLastError::Crc,
        _ => CanLastError::NoChange,
    };

    FdcanStatus {
        bus_state,
        tx_error_count: ecr.tec(),
        rx_error_count: ecr.rec(),
        last_error,
    }
}

fn log_fdcan_status() {
    let status = read_fdcan_status();

    if status.bus_state.is_ok() {
        info!(
            "CAN: {} TEC={} REC={} last_err={}",
            status.bus_state, status.tx_error_count, status.rx_error_count, status.last_error,
        );
    } else {
        warn!(
            "CAN: {} TEC={} REC={} last_err={}",
            status.bus_state, status.tx_error_count, status.rx_error_count, status.last_error,
        );
    }
}

#[embassy_executor::task]
async fn command_task() {
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
            Command::SetZeroPosition => {
                info!("SetZeroPosition command received");
                let (primary_angle, secondary_angle) = read_zero_reference_angles();
                write_zero_offsets(primary_angle, secondary_angle);
                foc_isr::with_foc(|foc| {
                    foc.reset();
                    foc.set_target_angle(0.0);
                    foc.state.angle = 0.0;
                    foc.state.angular_change = 0.0;
                    foc.state.velocity = 0.0;
                    foc.state.angle_error = 0.0;
                    foc.state.angle_integral = 0.0;
                    foc.state.velocity_error = 0.0;
                    foc.state.velocity_integral = 0.0;
                });
            }
            Command::RunMotorTuning => {
                info!("RunMotorTuning command received, clearing config and resetting");
                {
                    let mut flash_guard = FLASH.lock().await;
                    if let Some(flash) = flash_guard.as_mut() {
                        if let Err(e) = flash_config::clear_config(flash) {
                            error!("Failed to clear config: {:?}", e);
                        } else {
                            info!("Config cleared, resetting system for full motor tuning");
                            cortex_m::peripheral::SCB::sys_reset();
                        }
                    }
                }
            }
            Command::SaveParameters | Command::SaveConfig => {
                info!("Save-to-flash command received");
                let mut flash_guard = FLASH.lock().await;
                if let Some(flash) = flash_guard.as_mut() {
                    let can_id = flash_config::read_config(flash)
                        .map(|c| c.can_id)
                        .unwrap_or(0x0F);

                    foc_isr::with_foc(|foc| {
                        let mut config = flash_config::from_foc(foc, can_id);
                        let (primary_zero_offset, secondary_zero_offset) = read_zero_offsets();
                        config.primary_zero_offset = primary_zero_offset;
                        config.secondary_zero_offset = secondary_zero_offset;
                        if let Err(e) = flash_config::write_config(flash, &mut config) {
                            error!("Failed to save config: {:?}", e);
                        } else {
                            info!("Config saved to flash");
                        }
                    });
                }
            }
            _ => {
                handle_command_isr(&command, message.host_can_id, &response_sender).await;
            }
        }
    }
}

async fn handle_command_isr(
    command: &Command,
    host_can_id: u8,
    response_sender: &Sender<'static, CriticalSectionRawMutex, QueuedResponse, 64>,
) {
    match command {
        Command::Enable => {
            if has_active_fault() {
                warn!("Enable rejected while fault is active");
            } else {
                if MOTOR_DISARMED_BY_FAULT.swap(false, Ordering::Relaxed) {
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

        _ => {
            dispatch_set_param!(command,
                AngleRef => set_target_angle,
                SpeedRef => set_target_velocity,
                IqRef => set_target_torque,
                SpeedLimit => set_velocity_limit,
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
                IqRef => foc.state.i_ref,
                Angle => foc.state.angle,
                AngleRef => foc.target.angle,
                Speed => foc.state.velocity,
                SpeedRef => foc.target.velocity,
                SpeedLimit => foc.velocity_limit.unwrap_or(0.0),
                TorqueLimit => foc.torque_limit.unwrap_or(0.0),
                Iq => foc.state.i_q,
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

fn log_state(state: &FocState, dt: &Duration, check_count: usize) {
    let dt_seconds = (dt.as_micros() as f32) / 1e6;
    let freq = (check_count as f32) / dt_seconds;
    let angle_2 = f32::from_le_bytes(
        ANGLE_2
            .load(core::sync::atomic::Ordering::Relaxed)
            .to_le_bytes(),
    );
    info!(
        "MEASURED: a={}, a2={}, v={}, t={}, {} Hz",
        state.angle, angle_2, state.velocity, state.v_q, freq
    );
}

fn has_active_fault() -> bool {
    DRV_FAULT_ACTIVE.load(Ordering::Relaxed) || CAN_FAULT_ACTIVE.load(Ordering::Relaxed)
}

async fn set_gate_driver_enabled(enabled: bool) {
    let mut gate_driver = GATE_DRIVER.lock().await;
    if let Some(gate_driver) = gate_driver.as_mut() {
        if enabled {
            gate_driver.turn_on();
        } else {
            gate_driver.turn_off();
        }
    }
}

async fn disarm_motor_due_to_fault() {
    foc_isr::with_foc(|foc| foc.stop());
    set_gate_driver_enabled(false).await;
    MOTOR_DISARMED_BY_FAULT.store(true, Ordering::Relaxed);
}

fn write_sensor_snapshot(encoder_reading: EncoderReading) {
    let active_index = ACTIVE_SENSOR_SNAPSHOT.load(Ordering::Relaxed) as usize;
    let next_index = active_index ^ 1;
    let snapshot = &SENSOR_SNAPSHOTS[next_index];

    snapshot.angle.store(
        u32::from_le_bytes(encoder_reading.angle.to_le_bytes()),
        Ordering::Relaxed,
    );
    snapshot.phase_angle.store(
        u32::from_le_bytes(encoder_reading.phase_angle.to_le_bytes()),
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

fn read_zero_offsets() -> (f32, f32) {
    let primary = f32::from_le_bytes(PRIMARY_ZERO_OFFSET.load(Ordering::Relaxed).to_le_bytes());
    let secondary = f32::from_le_bytes(SECONDARY_ZERO_OFFSET.load(Ordering::Relaxed).to_le_bytes());
    (primary, secondary)
}

fn read_zero_reference_angles() -> (f32, f32) {
    let active_index = ACTIVE_SENSOR_SNAPSHOT.load(Ordering::Acquire) as usize;
    let snapshot = &SENSOR_SNAPSHOTS[active_index];
    let primary = f32::from_le_bytes(snapshot.phase_angle.load(Ordering::Relaxed).to_le_bytes());
    let secondary = f32::from_le_bytes(SECONDARY_RAW_ANGLE.load(Ordering::Relaxed).to_le_bytes());
    (primary, secondary)
}

fn write_zero_offsets(primary_offset: f32, secondary_offset: f32) {
    PRIMARY_ZERO_OFFSET.store(
        u32::from_le_bytes(primary_offset.to_le_bytes()),
        Ordering::Relaxed,
    );
    SECONDARY_ZERO_OFFSET.store(
        u32::from_le_bytes(secondary_offset.to_le_bytes()),
        Ordering::Relaxed,
    );
    ZERO_OFFSET_VERSION.fetch_add(1, Ordering::Relaxed);
}

fn apply_zero_offset(angle: f32, zero_offset: f32) -> f32 {
    wrap_0_tau(angle - zero_offset)
}

#[embassy_executor::task]
async fn encoder_task(
    mut sensor: As5047P<'static>,
    mut secondary_sensor: Option<As5047P<'static>>,
) {
    let mut tracker = AngleTracker::new(VELOCITY_OBSERVER_BANDWIDTH);
    let mut correction_version = 0;
    let mut correction = runtime_snapshot();
    let mut zero_offset_version = ZERO_OFFSET_VERSION.load(Ordering::Relaxed);

    loop {
        let latest_version = runtime_version();
        if latest_version != correction_version {
            correction = runtime_snapshot();
            correction_version = latest_version;
        }
        let latest_zero_offset_version = ZERO_OFFSET_VERSION.load(Ordering::Relaxed);
        if latest_zero_offset_version != zero_offset_version {
            tracker = AngleTracker::new(VELOCITY_OBSERVER_BANDWIDTH);
            zero_offset_version = latest_zero_offset_version;
        }
        let (primary_zero_offset, secondary_zero_offset) = read_zero_offsets();

        match sensor.read_raw_angle().await {
            Ok(raw_angle) => {
                let wrapped_angle =
                    correction.correct_wrapped_angle(raw_angle as f32 * AS5047P_RAW_TO_RADIAN);
                let zeroed_angle = apply_zero_offset(wrapped_angle, primary_zero_offset);
                let mut encoder_reading = tracker.update(zeroed_angle, Instant::now());
                encoder_reading.phase_angle = wrapped_angle;
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

        if let Some(secondary_sensor) = secondary_sensor.as_mut() {
            match secondary_sensor.read_raw_angle().await {
                Ok(raw_angle) => {
                    let raw_secondary_angle = raw_angle as f32 * AS5047P_RAW_TO_RADIAN;
                    SECONDARY_RAW_ANGLE.store(
                        u32::from_le_bytes(raw_secondary_angle.to_le_bytes()),
                        core::sync::atomic::Ordering::Relaxed,
                    );
                    let angle_2 = apply_zero_offset(raw_secondary_angle, secondary_zero_offset);
                    ANGLE_2.store(
                        u32::from_le_bytes(angle_2.to_le_bytes()),
                        core::sync::atomic::Ordering::Relaxed,
                    );
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
        }

        Timer::after_micros(50).await;
    }
}

fn read_sensor() -> EncoderReading {
    let active_index = ACTIVE_SENSOR_SNAPSHOT.load(Ordering::Acquire) as usize;
    let snapshot = &SENSOR_SNAPSHOTS[active_index];
    let raw_angle = snapshot.angle.load(Ordering::Relaxed);
    let raw_phase_angle = snapshot.phase_angle.load(Ordering::Relaxed);
    let raw_velocity = snapshot.velocity.load(Ordering::Relaxed);
    let raw_dt = snapshot.dt.load(Ordering::Relaxed);

    EncoderReading {
        angle: f32::from_le_bytes(raw_angle.to_le_bytes()),
        phase_angle: f32::from_le_bytes(raw_phase_angle.to_le_bytes()),
        velocity: f32::from_le_bytes(raw_velocity.to_le_bytes()),
        dt: f32::from_le_bytes(raw_dt.to_le_bytes()),
    }
}

#[embassy_executor::task]
async fn can_tx_task(mut can_tx: CanTx<'static>) {
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

#[embassy_executor::task]
async fn can_rx_task(mut can_rx: CanRx<'static>) {
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
                            .try_send(can_message::message::CommandMessage {
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

async fn save_can_id_to_flash(new_can_id: u8) {
    let mut flash_guard = FLASH.lock().await;
    if let Some(flash) = flash_guard.as_mut()
        && let Some(mut config) = flash_config::read_config(flash)
    {
        config.can_id = new_can_id;
        let (primary_zero_offset, secondary_zero_offset) = read_zero_offsets();
        config.primary_zero_offset = primary_zero_offset;
        config.secondary_zero_offset = secondary_zero_offset;
        if let Err(e) = flash_config::write_config(flash, &mut config) {
            warn!("Failed to save CAN ID to flash: {:?}", e);
        } else {
            info!("CAN ID saved to flash");
        }
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
    // Delay for probe-rs to attach during firmware flashing (~10ms at 16MHz HSI)
    cortex_m::asm::delay(INIT_DELAY_CYCLES);

    let mut config = Config::default();
    clock::set_clock(&mut config);
    initialize_cordic();
    run_and_log_validation_tests();
    let p = embassy_stm32::init(config);
    clock::print_clock_info(&p.RCC);

    embassy_stm32::interrupt::TIM1_UP_TIM16.set_priority(interrupt::Priority::P0);
    embassy_stm32::interrupt::ADC1_2.set_priority(interrupt::Priority::P1);
    embassy_stm32::interrupt::FDCAN1_IT0.set_priority(CAN_INTERRUPT_PRIORITY);
    embassy_stm32::interrupt::FDCAN1_IT1.set_priority(CAN_INTERRUPT_PRIORITY);

    let mut spi_config = stm32_spi::Config::default();
    spi_config.miso_pull = gpio::Pull::Up;
    spi_config.mode = stm32_spi::MODE_1;
    spi_config.bit_order = stm32_spi::BitOrder::MsbFirst;
    spi_config.frequency = mhz(10);
    spi_config.gpio_speed = gpio::Speed::VeryHigh;
    let enc1_cs_out = gpio::Output::new(p.PB0, gpio::Level::High, gpio::Speed::VeryHigh);
    let enc2_cs_out = gpio::Output::new(p.PB1, gpio::Level::High, gpio::Speed::VeryHigh);

    {
        let p_spi = stm32_spi::Spi::new(
            p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH4, spi_config,
        );
        *(SPI.lock().await) = Some(p_spi);
    }

    let mut p_flash = Flash::new_blocking(p.FLASH);
    let stored_config = flash_config::read_config(&mut p_flash);
    let startup_can_id = stored_config
        .as_ref()
        .map(|config| config.can_id)
        .unwrap_or(0x0F);

    let mut can_conf = stm32_can::CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    can_conf.set_bitrate(CAN_BITRATE);
    can_conf.set_config(
        can_conf
            .config()
            .set_global_filter(stm32_can::config::GlobalFilter::reject_all()),
    );
    can_conf.properties().set_extended_filter(
        stm32_can::filter::ExtendedFilterSlot::_0,
        can_id_filter(startup_can_id),
    );
    let p_can = can_conf.into_normal_mode();
    log_fdcan_registers("startup");
    let (can_tx, can_rx, properties) = p_can.split();

    {
        *(CAN_PROPERTIES.lock().await) = Some(properties);
    }

    let drvoff_pin = gpio::Output::new(p.PB12, gpio::Level::High, gpio::Speed::Medium);
    let drv_fault_pin = gpio::Input::new(p.PB11, gpio::Pull::Up);
    let led_fault = gpio::Output::new(p.PA4, gpio::Level::High, gpio::Speed::Low);

    let mut p_adc = adc::DummyAdc::new(p.ADC1, p.PA0, p.PA1, p.PA2, p.PA3);
    p_adc.calibrate();
    p_adc.initialize(1, 2, 3);

    let mut timer = pwm::Pwm6Timer::new(p.TIM1, p.PA8, p.PA9, p.PA10, p.PB13, p.PB14, p.PB15);
    timer.initialize(foc_isr::MAX_COMPARE_VALUE as u16);
    info!(
        "Timer edge-aligned wrap frequency (before center-aligned/REP effects): {}",
        timer.get_frequency()
    );

    let mut sensor = As5047P::new(&SPI, enc1_cs_out, VELOCITY_OBSERVER_BANDWIDTH);
    match sensor.initialize().await {
        Ok(diagnostics) => info!("ENC1 diagnostics: {:?}", diagnostics),
        Err(e) => {
            error!("ENC1 initialization failed, {:?}", e);
            return;
        }
    }

    let secondary_sensor = if USE_SECONDARY_ENCODER {
        let mut secondary_sensor = As5047P::new(&SPI, enc2_cs_out, VELOCITY_OBSERVER_BANDWIDTH);
        match secondary_sensor.initialize().await {
            Ok(diagnostics) => {
                info!("ENC2 diagnostics: {:?}", diagnostics);
                Some(secondary_sensor)
            }
            Err(e) => {
                warn!("ENC2 initialization failed, continuing without it: {:?}", e);
                None
            }
        }
    } else {
        info!("ENC2 disabled by configuration");
        None
    };

    // Start encoder task early so sensor readings are available during FOC init
    unwrap!(spawner.spawn(encoder_task(sensor, secondary_sensor)));

    let Some(can_id) = init_foc(drvoff_pin, timer, p_adc, &mut p_flash, stored_config).await else {
        return;
    };
    CAN_ID.store(can_id, core::sync::atomic::Ordering::Relaxed);

    // Store flash in static for use by command handlers
    {
        *(FLASH.lock().await) = Some(p_flash);
    }

    unwrap!(spawner.spawn(monitor_task()));
    unwrap!(spawner.spawn(drv_fault_monitor_task(drv_fault_pin, led_fault)));
    unwrap!(spawner.spawn(command_task()));
    unwrap!(spawner.spawn(can_rx_task(can_rx)));
    unwrap!(spawner.spawn(can_tx_task(can_tx)));
}
