//! Interrupt-driven FOC controller
//!
//! This module runs the FOC computation directly in the ADC interrupt handler,
//! synchronized with PWM updates for deterministic timing.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, AtomicU16, Ordering};

use critical_section::Mutex;
use embassy_stm32::pac;

use can_message::message::{FeedbackType, MotorCurrent, MotorStatus, ResponseBody};
use foc::controller::{FocController, FocState, RunMode};
use foc::current::PhaseCurrent;
use foc::pwm_output::DutyCycle3Phase;

use crate::drv8316::{self, CsaGain};
use crate::{RESPONSE_CHANNEL, read_sensor};

/// Maximum PWM compare value (12-bit: 4095)
const MAX_COMPARE_VALUE: u32 = (1 << 12) - 1;
const MAX_DUTY: u32 = MAX_COMPARE_VALUE + 1;

/// Static FOC controller wrapped for interrupt-safe access
static FOC_CONTEXT: Mutex<RefCell<Option<FocContext>>> = Mutex::new(RefCell::new(None));

/// Feedback configuration - can be modified from thread mode
static FEEDBACK_TYPE: AtomicU8 = AtomicU8::new(0); // 0 = Status, 1 = Current
static FEEDBACK_PERIOD: AtomicU8 = AtomicU8::new(10);
static FEEDBACK_COUNTER: AtomicU8 = AtomicU8::new(0);

/// Loop iteration counter for diagnostics
static LOOP_COUNTER: AtomicU16 = AtomicU16::new(0);

/// FOC context that lives in interrupt context
pub struct FocContext {
    pub foc: FocController<fn(f32) -> (f32, f32)>,
    pub csa_gain: CsaGain,
    pub use_current_sensing: bool,
}

/// Initialize the FOC context from thread mode
/// This must be called after sensor alignment and calibration
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

/// Check if FOC context is initialized
pub fn is_initialized() -> bool {
    critical_section::with(|cs| FOC_CONTEXT.borrow(cs).borrow().is_some())
}

/// Set feedback type (called from thread mode)
pub fn set_feedback_type(typ: FeedbackType) {
    let val = match typ {
        FeedbackType::Status => 0,
        FeedbackType::Current => 1,
    };
    FEEDBACK_TYPE.store(val, Ordering::Relaxed);
}

/// Set feedback period (called from thread mode)
pub fn set_feedback_period(period: u8) {
    FEEDBACK_PERIOD.store(period, Ordering::Relaxed);
}

/// Get loop counter for diagnostics
pub fn get_loop_counter() -> u16 {
    LOOP_COUNTER.swap(0, Ordering::Relaxed)
}

/// Access FOC controller from thread mode (via critical section)
/// Use this for configuration changes from command handlers
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

/// Get FOC state (for telemetry/logging from thread mode)
pub fn get_foc_state() -> Option<FocState> {
    critical_section::with(|cs| {
        FOC_CONTEXT
            .borrow(cs)
            .borrow()
            .as_ref()
            .map(|ctx| ctx.foc.state)
    })
}

/// Get current run mode
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

/// Main FOC interrupt handler - called from ADC1_2 interrupt
/// This is the hot path - keep it fast!
#[inline(always)]
pub fn run_foc_iteration(ia_raw: u16, ib_raw: u16, ic_raw: u16) {
    critical_section::with(|cs| {
        if let Some(ref mut ctx) = *FOC_CONTEXT.borrow(cs).borrow_mut() {
            // Read sensor data from atomics (updated by encoder_task)
            let reading = read_sensor();

            // Convert raw ADC to phase currents
            let phase_current = if ctx.use_current_sensing {
                drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, ctx.csa_gain)
            } else {
                PhaseCurrent::new(0.0, 0.0, 0.0)
            };

            // Run FOC computation
            if let Ok(duty) = ctx.foc.get_duty_cycle(&reading, phase_current) {
                // Update PWM directly via PAC (no Embassy overhead)
                set_pwm_duty(duty);

                // Handle feedback
                let counter = FEEDBACK_COUNTER.load(Ordering::Relaxed);
                let period = FEEDBACK_PERIOD.load(Ordering::Relaxed);

                if counter >= period {
                    FEEDBACK_COUNTER.store(0, Ordering::Relaxed);
                    send_feedback(&ctx.foc.state);
                } else {
                    FEEDBACK_COUNTER.store(counter + 1, Ordering::Relaxed);
                }
            }

            // Increment loop counter for diagnostics
            LOOP_COUNTER.fetch_add(1, Ordering::Relaxed);
        }
    });
}

/// Set PWM duty cycle directly via PAC registers
#[inline(always)]
fn set_pwm_duty(duty: DutyCycle3Phase) {
    let tim1 = pac::TIM1;

    let d1 = (duty.t1 * (MAX_DUTY as f32)) as u32;
    let d2 = (duty.t2 * (MAX_DUTY as f32)) as u32;
    let d3 = (duty.t3 * (MAX_DUTY as f32)) as u32;

    // CCR1, CCR2, CCR3 for channels 1, 2, 3
    tim1.ccr(0).write(|w| w.set_ccr(d1 as u16));
    tim1.ccr(1).write(|w| w.set_ccr(d2 as u16));
    tim1.ccr(2).write(|w| w.set_ccr(d3 as u16));
}

/// Send feedback via response channel (best-effort, non-blocking)
#[inline(always)]
fn send_feedback(state: &FocState) {
    let feedback_type = FEEDBACK_TYPE.load(Ordering::Relaxed);
    let sender = RESPONSE_CHANNEL.sender();

    let body = match feedback_type {
        0 => ResponseBody::MotorStatus(MotorStatus {
            angle: state.angle,
            velocity: state.velocity,
            torque: state.i_q,
            temperature: 0.0,
        }),
        _ => ResponseBody::MotorCurrent(MotorCurrent {
            i_q: state.i_q,
            i_d: state.i_d,
        }),
    };

    // Non-blocking send - drop if channel is full
    let _ = sender.try_send(body);
}
