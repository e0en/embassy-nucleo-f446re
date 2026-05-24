use embassy_stm32::interrupt;
use foc::controller::FocController;

pub(crate) mod can_control;
pub(crate) mod encoder;
pub(crate) mod fault;
pub(crate) mod init;
pub(crate) mod monitor;
pub(crate) mod persistence;

pub(crate) type FocControllerType = FocController<fn(f32) -> (f32, f32)>;

pub(crate) const PSU_VOLTAGE: f32 = 16.0;
pub(crate) const VOLTAGE_RAMP_RATE: f32 = 1000.0;
pub(crate) const CURRENT_SAFETY_MARGIN: f32 = 0.8;
pub(crate) const PHASE_MAPPING_TOLERANCE: f32 = 0.05;
/// Must stay below current filter cutoff (1/(2π×0.001) ≈ 159 Hz)
pub(crate) const CURRENT_PI_FREQUENCY: f32 = 100.0;
pub(crate) const VELOCITY_PI_FREQUENCY: f32 = 3.0;
pub(crate) const ALIGN_VOLTAGE: f32 = 2.0;
pub(crate) const ALIGN_VOLTAGE_SEARCH_START: f32 = 0.1;
pub(crate) const ALIGN_VOLTAGE_SEARCH_STEP: f32 = 0.1;
pub(crate) const ALIGN_VOLTAGE_SEARCH_SETTLE_MS: u64 = 200;
pub(crate) const ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_RAD: f32 = 0.01;
pub(crate) const ALIGN_VOLTAGE_SEARCH_SWEEP_STEPS: usize = 40;
pub(crate) const ALIGN_VOLTAGE_SEARCH_SWEEP_STEP_MS: u64 = 10;
pub(crate) const ALIGN_VOLTAGE_SEARCH_DELTA_EPSILON_RAD: f32 = 0.0005;
pub(crate) const ALIGN_VOLTAGE_SEARCH_MIN_DOMINANT_STEPS: usize = 16;
pub(crate) const ALIGN_VOLTAGE_SEARCH_MIN_NET_ANGLE_RAD: f32 = 0.005;
pub(crate) const INIT_DELAY_CYCLES: u32 = 160_000;
pub(crate) const CAN_BITRATE: u32 = 1_000_000;
pub(crate) const CAN_RECOVERY_REINIT_INTERVAL_MS: u64 = 100;
pub(crate) const VELOCITY_OBSERVER_BANDWIDTH: f32 = 20.0;
pub(crate) const CAN_INTERRUPT_PRIORITY: interrupt::Priority = interrupt::Priority::P7;
pub(crate) const DRV_FAULT_POLL_INTERVAL_MS: u64 = 10;
pub(crate) const IMPEDANCE_TUNING_MAX_CURRENT: f32 = 1.5;
pub(crate) const ACTUATOR_REDUCTION_RATIO_MAGNITUDE: i32 = 19;
pub(crate) const ACTUATOR_REDUCTION_RATIO: f32 = ACTUATOR_REDUCTION_RATIO_MAGNITUDE as f32;
