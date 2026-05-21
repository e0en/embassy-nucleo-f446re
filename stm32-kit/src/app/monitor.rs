use defmt::info;
use embassy_executor::task;
use embassy_time::{Duration, Instant, Timer};
use foc::controller::FocState;

use crate::{adc, foc_isr};

use super::fault::log_fdcan_status;

#[task]
pub(crate) async fn monitor_task() {
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

fn log_state(state: &FocState, dt: &Duration, check_count: usize) {
    let dt_seconds = (dt.as_micros() as f32) / 1e6;
    let freq = (check_count as f32) / dt_seconds;
    info!(
        "MEASURED: a={}, v={}, t={}, {} Hz",
        state.angle, state.velocity, state.v_q, freq
    );
}
