//! Interrupt-driven FOC controller
//!
//! This module runs the FOC computation directly in the ADC interrupt handler,
//! synchronized with PWM updates for deterministic timing.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

use critical_section::Mutex;
use embassy_stm32::pac;

use foc::controller::{FocController, FocState, RunMode};
use foc::encoder::EncoderReading;
use foc::pwm_output::DutyCycle3Phase;

use crate::app::units::{input_to_output_shaft, output_to_input_shaft};
use crate::drv8316::{self, CsaGain};
use crate::read_sensor;

type FocSincos = fn(f32) -> (f32, f32);

/// Maximum PWM compare value (11-bit: 2047)
pub const MAX_COMPARE_VALUE: u32 = (1 << 11) - 1;
const MAX_DUTY: u32 = MAX_COMPARE_VALUE + 1;
const TIM1_CLOCK_HZ: u32 = 170_000_000;
// TIM1 runs from the 170 MHz APB2 clock. In center-aligned mode the counter
// traverses one full PWM cycle in 2 * ARR counts, and REP=1 reduces the two
// update events (overflow + underflow) to one TRGO/ADC trigger per full cycle.
pub const CONTROL_LOOP_DT_SECONDS: f32 = (2.0 * MAX_DUTY as f32) / TIM1_CLOCK_HZ as f32;

static FOC_CONTEXT: Mutex<RefCell<Option<FocContext>>> = Mutex::new(RefCell::new(None));

static LOOP_COUNTER: AtomicU16 = AtomicU16::new(0);
static OUTPUT_ZERO_OFFSET: AtomicU32 = AtomicU32::new(0);

#[inline(always)]
fn input_to_user_output_shaft(value: f32) -> f32 {
    input_to_output_shaft(value) - output_zero_offset()
}

#[inline(always)]
fn user_output_to_input_shaft(value: f32) -> f32 {
    output_to_input_shaft(value + output_zero_offset())
}

pub struct FocContext {
    pub foc: FocController<FocSincos>,
    pub csa_gain: CsaGain,
}

/// Must be called after sensor alignment and calibration
#[cfg(feature = "main-fw")]
pub fn initialize_foc_context(foc: FocController<FocSincos>, csa_gain: CsaGain) {
    critical_section::with(|cs| {
        FOC_CONTEXT
            .borrow(cs)
            .replace(Some(FocContext { foc, csa_gain }));
    });
}

pub fn is_initialized() -> bool {
    critical_section::with(|cs| FOC_CONTEXT.borrow(cs).borrow().is_some())
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
    F: FnOnce(&mut FocController<FocSincos>) -> R,
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

            let phase_current = drv8316::convert_csa_readings(ia_raw, ib_raw, ic_raw, ctx.csa_gain);

            if let Ok(duty) = ctx.foc.get_duty_cycle(&foc_reading, phase_current) {
                // Update PWM directly via PAC (no Embassy overhead)
                set_pwm_duty(duty);
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
