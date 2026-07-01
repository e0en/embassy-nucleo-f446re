use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{Format, info, warn};
use embassy_executor::task;
use embassy_stm32::{gpio, pac};
use embassy_time::Timer;

use crate::foc_isr;

use super::{DRV_FAULT_POLL_INTERVAL_MS, persistence::GATE_DRIVER};

static DRV_FAULT_ACTIVE: AtomicBool = AtomicBool::new(false);
static MOTOR_DISARMED_BY_FAULT: AtomicBool = AtomicBool::new(false);

fn set_fault_led(led: &mut gpio::Output<'static>, fault_active: bool) {
    if fault_active {
        led.set_high();
    } else {
        led.set_low();
    }
}

#[task]
pub(crate) async fn drv_fault_monitor_task(
    fault_pin: gpio::Input<'static>,
    mut led_fault: gpio::Output<'static>,
) {
    let mut last_drv_fault_active = false;
    let mut last_led_active = false;
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

        let led_active = drv_fault_active;
        if led_active != last_led_active {
            set_fault_led(&mut led_fault, led_active);
            last_led_active = led_active;
        }

        Timer::after_millis(DRV_FAULT_POLL_INTERVAL_MS).await;
    }
}

#[derive(Format, Clone, Copy)]
struct FdcanRegisters {
    cccr: u32,
    nbtp: u32,
    psr: u32,
    ecr: u32,
    ir: u32,
}

fn read_fdcan_registers() -> FdcanRegisters {
    let fdcan = pac::FDCAN1;
    FdcanRegisters {
        cccr: fdcan.cccr().read().0,
        nbtp: fdcan.nbtp().read().0,
        psr: fdcan.psr().read().0,
        ecr: fdcan.ecr().read().0,
        ir: fdcan.ir().read().0,
    }
}

pub(crate) fn log_fdcan_registers(context: &'static str) {
    let regs = read_fdcan_registers();
    info!(
        "CAN regs[{}]: CCCR={=u32:#010x} NBTP={=u32:#010x} PSR={=u32:#010x} ECR={=u32:#010x} IR={=u32:#010x}",
        context, regs.cccr, regs.nbtp, regs.psr, regs.ecr, regs.ir,
    );
}

pub(crate) fn has_active_fault() -> bool {
    DRV_FAULT_ACTIVE.load(Ordering::Relaxed)
}

pub(crate) fn take_motor_disarmed_by_fault() -> bool {
    MOTOR_DISARMED_BY_FAULT.swap(false, Ordering::Relaxed)
}

pub(crate) async fn set_gate_driver_enabled(enabled: bool) {
    let mut gate_driver = GATE_DRIVER.lock().await;
    if let Some(gate_driver) = gate_driver.as_mut() {
        if enabled {
            gate_driver.turn_on();
        } else {
            gate_driver.turn_off();
        }
    }
}

pub(crate) async fn disarm_motor_due_to_fault() {
    foc_isr::with_foc(|foc| foc.stop());
    set_gate_driver_enabled(false).await;
    MOTOR_DISARMED_BY_FAULT.store(true, Ordering::Relaxed);
}
