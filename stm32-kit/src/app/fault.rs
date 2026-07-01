use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{Format, info, warn};
use embassy_executor::task;
use embassy_stm32::{gpio, pac};
use embassy_time::{Duration, Instant, Timer};

use crate::foc_isr;

use super::{
    CAN_RECOVERY_REINIT_INTERVAL_MS, DRV_FAULT_POLL_INTERVAL_MS, persistence::GATE_DRIVER,
};

static DRV_FAULT_ACTIVE: AtomicBool = AtomicBool::new(false);
static CAN_FAULT_ACTIVE: AtomicBool = AtomicBool::new(false);
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

#[derive(Format, Clone, Copy)]
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

#[derive(Format, Clone, Copy, PartialEq, Eq)]
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

#[derive(Format, Clone, Copy)]
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

pub(crate) fn has_active_fault() -> bool {
    DRV_FAULT_ACTIVE.load(Ordering::Relaxed) || CAN_FAULT_ACTIVE.load(Ordering::Relaxed)
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
