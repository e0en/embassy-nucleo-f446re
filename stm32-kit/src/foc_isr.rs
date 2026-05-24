//! Interrupt-driven FOC controller
//!
//! This module runs the FOC computation directly in the ADC interrupt handler,
//! synchronized with PWM updates for deterministic timing.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, Ordering};

use critical_section::Mutex;
use embassy_stm32::pac;

use can_message::message::{
    DebugValue, DebugValueKind, FeedbackType, MotorCurrent, MotorStatus, ResponseBody,
};
use foc::controller::{FocController, FocState, RunMode};
use foc::current::PhaseCurrent;
use foc::encoder::EncoderReading;
use foc::pwm_output::DutyCycle3Phase;

use crate::app::ACTUATOR_REDUCTION_RATIO;
use crate::drv8316::{self, CsaGain};
use crate::{RESPONSE_CHANNEL, read_sensor};

/// Maximum PWM compare value (11-bit: 2047)
pub const MAX_COMPARE_VALUE: u32 = (1 << 11) - 1;
const MAX_DUTY: u32 = MAX_COMPARE_VALUE + 1;
const TIM1_CLOCK_HZ: u32 = 170_000_000;
const DEFAULT_FEEDBACK_HZ: u32 = 1_000;
const DEFAULT_FEEDBACK_INTERVAL_TICKS: u8 =
    ((TIM1_CLOCK_HZ / (2 * MAX_DUTY * DEFAULT_FEEDBACK_HZ)) as u8).saturating_sub(1);
// TIM1 runs from the 170 MHz APB2 clock. In center-aligned mode the counter
// traverses one full PWM cycle in 2 * ARR counts, and REP=1 reduces the two
// update events (overflow + underflow) to one TRGO/ADC trigger per full cycle.
const CONTROL_LOOP_DT_SECONDS: f32 = (2.0 * MAX_DUTY as f32) / TIM1_CLOCK_HZ as f32;

static FOC_CONTEXT: Mutex<RefCell<Option<FocContext>>> = Mutex::new(RefCell::new(None));

static FEEDBACK_TYPE: AtomicU8 = AtomicU8::new(0); // 0 = Status, 1 = Current
static FEEDBACK_PERIOD: AtomicU8 = AtomicU8::new(DEFAULT_FEEDBACK_INTERVAL_TICKS);
static FEEDBACK_COUNTER: AtomicU8 = AtomicU8::new(0);
static FEEDBACK_HOST_CAN_ID: AtomicU8 = AtomicU8::new(crate::DEFAULT_HOST_CAN_ID);
static FEEDBACK_HOST_CAN_ID_SET: AtomicBool = AtomicBool::new(false);

static LOOP_COUNTER: AtomicU16 = AtomicU16::new(0);
static OUTPUT_ZERO_OFFSET: AtomicU32 = AtomicU32::new(0);

#[inline(always)]
fn input_to_output_shaft(value: f32) -> f32 {
    value / ACTUATOR_REDUCTION_RATIO
}

#[inline(always)]
fn input_to_user_output_shaft(value: f32) -> f32 {
    input_to_output_shaft(value) - output_zero_offset()
}

#[inline(always)]
fn user_output_to_input_shaft(value: f32) -> f32 {
    (value + output_zero_offset()) * ACTUATOR_REDUCTION_RATIO
}

pub struct FocContext {
    pub foc: FocController<fn(f32) -> (f32, f32)>,
    pub csa_gain: CsaGain,
    pub use_current_sensing: bool,
}

/// Must be called after sensor alignment and calibration
pub fn initialize_foc_context(
    foc: FocController<fn(f32) -> (f32, f32)>,
    csa_gain: CsaGain,
    use_current_sensing: bool,
) {
    critical_section::with(|cs| {
        FOC_CONTEXT.borrow(cs).replace(Some(FocContext {
            foc,
            csa_gain,
            use_current_sensing,
        }));
    });
}

pub fn is_initialized() -> bool {
    critical_section::with(|cs| FOC_CONTEXT.borrow(cs).borrow().is_some())
}

pub fn set_feedback_type(typ: FeedbackType) {
    let val = match typ {
        FeedbackType::Status => 0,
        FeedbackType::Current => 1,
        FeedbackType::SpeedError => 2,
        FeedbackType::IqRef => 3,
        FeedbackType::VelocityIntegral => 4,
    };
    FEEDBACK_TYPE.store(val, Ordering::Relaxed);
}

pub fn set_feedback_period(period: u8) {
    FEEDBACK_PERIOD.store(period, Ordering::Relaxed);
}

pub fn set_feedback_host_can_id(host_can_id: u8) {
    FEEDBACK_HOST_CAN_ID.store(host_can_id, Ordering::Relaxed);
    FEEDBACK_HOST_CAN_ID_SET.store(true, Ordering::Relaxed);
}

pub fn get_loop_counter() -> u16 {
    LOOP_COUNTER.swap(0, Ordering::Relaxed)
}

pub fn set_output_zero_offset(offset: f32) {
    OUTPUT_ZERO_OFFSET.store(u32::from_le_bytes(offset.to_le_bytes()), Ordering::Relaxed);
}

pub fn output_zero_offset() -> f32 {
    f32::from_le_bytes(OUTPUT_ZERO_OFFSET.load(Ordering::Relaxed).to_le_bytes())
}

pub fn input_to_user_angle(value: f32) -> f32 {
    input_to_user_output_shaft(value)
}

pub fn user_to_input_angle(value: f32) -> f32 {
    user_output_to_input_shaft(value)
}

pub fn with_foc<F, R>(f: F) -> Option<R>
where
    F: FnOnce(&mut FocController<fn(f32) -> (f32, f32)>) -> R,
{
    critical_section::with(|cs| {
        if let Some(ref mut ctx) = *FOC_CONTEXT.borrow(cs).borrow_mut() {
            Some(f(&mut ctx.foc))
        } else {
            None
        }
    })
}

pub fn get_foc_state() -> Option<FocState> {
    critical_section::with(|cs| {
        FOC_CONTEXT
            .borrow(cs)
            .borrow()
            .as_ref()
            .map(|ctx| ctx.foc.state)
    })
}

#[allow(dead_code)]
pub fn get_run_mode() -> Option<RunMode> {
    critical_section::with(|cs| {
        FOC_CONTEXT
            .borrow(cs)
            .borrow()
            .as_ref()
            .map(|ctx| ctx.foc.mode)
    })
}

/// Called from ADC1_2 interrupt - keep it fast!
#[inline(always)]
pub fn run_foc_iteration(ia_raw: u16, ib_raw: u16, ic_raw: u16) {
    critical_section::with(|cs| {
        if let Some(ref mut ctx) = *FOC_CONTEXT.borrow(cs).borrow_mut() {
            // Read sensor data from atomics (updated by encoder_task)
            let encoder_reading = read_sensor();
            let foc_reading = EncoderReading {
                phase: encoder_reading.phase,
                full_rotations: encoder_reading.full_rotations,
                cumulative_angle: encoder_reading.cumulative_angle,
                velocity: encoder_reading.velocity,
                dt: CONTROL_LOOP_DT_SECONDS,
            };

            let phase_current = if ctx.use_current_sensing {
                drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, ctx.csa_gain)
            } else {
                PhaseCurrent::new(0.0, 0.0, 0.0)
            };

            if let Ok(duty) = ctx.foc.get_duty_cycle(&foc_reading, phase_current) {
                // Update PWM directly via PAC (no Embassy overhead)
                set_pwm_duty(duty);

                let counter = FEEDBACK_COUNTER.load(Ordering::Relaxed);
                let period = FEEDBACK_PERIOD.load(Ordering::Relaxed);

                if counter >= period {
                    FEEDBACK_COUNTER.store(0, Ordering::Relaxed);
                    send_feedback(&ctx.foc.state);
                } else {
                    FEEDBACK_COUNTER.store(counter + 1, Ordering::Relaxed);
                }
            }

            LOOP_COUNTER.fetch_add(1, Ordering::Relaxed);
        }
    });
}

#[inline(always)]
fn set_pwm_duty(duty: DutyCycle3Phase) {
    let tim1 = pac::TIM1;

    let d1 = (duty.t1 * (MAX_DUTY as f32)) as u32;
    let d2 = (duty.t2 * (MAX_DUTY as f32)) as u32;
    let d3 = (duty.t3 * (MAX_DUTY as f32)) as u32;

    tim1.ccr(0).write(|w| w.set_ccr(d1 as u16));
    tim1.ccr(1).write(|w| w.set_ccr(d2 as u16));
    tim1.ccr(2).write(|w| w.set_ccr(d3 as u16));
}

#[inline(always)]
fn send_feedback(state: &FocState) {
    if !FEEDBACK_HOST_CAN_ID_SET.load(Ordering::Relaxed) {
        return;
    }

    let feedback_type = FEEDBACK_TYPE.load(Ordering::Relaxed);
    let host_can_id = FEEDBACK_HOST_CAN_ID.load(Ordering::Relaxed);
    let sender = RESPONSE_CHANNEL.sender();

    let body = match feedback_type {
        0 => ResponseBody::MotorStatus(MotorStatus {
            angle: input_to_user_output_shaft(state.angle),
            velocity: input_to_output_shaft(state.velocity),
            torque: state.i_q,
            temperature: 0.0,
        }),
        1 => ResponseBody::MotorCurrent(MotorCurrent {
            i_q: state.i_q,
            i_d: state.i_d,
        }),
        2 => ResponseBody::DebugValue(DebugValue {
            kind: DebugValueKind::SpeedError,
            value: state.velocity_error,
        }),
        3 => ResponseBody::DebugValue(DebugValue {
            kind: DebugValueKind::IqRef,
            value: state.i_ref,
        }),
        _ => ResponseBody::DebugValue(DebugValue {
            kind: DebugValueKind::VelocityIntegral,
            value: state.velocity_integral,
        }),
    };

    // Non-blocking send - drop if channel is full
    let _ = sender.try_send(crate::QueuedResponse { host_can_id, body });
}
