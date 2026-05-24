use core::sync::atomic::AtomicU8;

use defmt::{error, info, warn};
use embassy_stm32::{can as stm32_can, flash::Flash, mode::Async, spi as stm32_spi};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};

use crate::{drv8316t, flash_config, foc_isr};

pub(crate) type SpiMutex = Mutex<ThreadModeRawMutex, Option<stm32_spi::Spi<'static, Async>>>;

pub(crate) static SPI: SpiMutex = SpiMutex::new(None);

pub(crate) type FlashMutex =
    Mutex<ThreadModeRawMutex, Option<Flash<'static, embassy_stm32::flash::Blocking>>>;

pub(crate) static FLASH: FlashMutex = FlashMutex::new(None);

pub(crate) type GateDriverMutex = Mutex<ThreadModeRawMutex, Option<drv8316t::Drv8316T<'static>>>;

pub(crate) static GATE_DRIVER: GateDriverMutex = GateDriverMutex::new(None);

pub(crate) type CanPropertiesMutex = Mutex<ThreadModeRawMutex, Option<stm32_can::Properties>>;

pub(crate) static CAN_PROPERTIES: CanPropertiesMutex = CanPropertiesMutex::new(None);

pub(crate) static CAN_ID: AtomicU8 = AtomicU8::new(0);

const CONFIG_SAVED_MESSAGE: &str = "Config saved to flash";
const CAN_ID_SAVED_MESSAGE: &str = "CAN ID saved to flash";
const OUTPUT_ZERO_OFFSET_SAVED_MESSAGE: &str = "Output zero offset saved to flash";

async fn with_flash<R>(
    f: impl FnOnce(&mut Flash<'static, embassy_stm32::flash::Blocking>) -> R,
) -> Option<R> {
    let mut flash_guard = FLASH.lock().await;
    flash_guard.as_mut().map(f)
}

fn write_config(
    flash: &mut Flash<'static, embassy_stm32::flash::Blocking>,
    config: &mut flash_config::ConfigData,
    success_message: &'static str,
) {
    if let Err(e) = flash_config::write_config(flash, config) {
        warn!("Failed to save {}: {:?}", success_message, e);
    } else {
        info!("{}", success_message);
    }
}

async fn update_config(
    f: impl FnOnce(&mut flash_config::ConfigData),
    success_message: &'static str,
) {
    with_flash(|flash| {
        if let Some(mut config) = flash_config::read_config(flash) {
            f(&mut config);
            write_config(flash, &mut config, success_message);
        }
    })
    .await;
}

pub(crate) async fn save_runtime_config() {
    if let Some(config) = foc_isr::with_foc(|foc| flash_config::from_foc(foc, 0x0F)) {
        update_config(
            |stored_config| {
                let mut runtime_config = config;
                runtime_config.can_id = stored_config.can_id;
                runtime_config.output_zero_offset = stored_config.output_zero_offset;
                runtime_config.secondary_zero_offset = stored_config.secondary_zero_offset;
                *stored_config = runtime_config;
            },
            CONFIG_SAVED_MESSAGE,
        )
        .await;
    }
}

pub(crate) async fn clear_config_and_reset(reason: &'static str) {
    with_flash(|flash| {
        if let Err(e) = flash_config::clear_config(flash) {
            error!("Failed to clear config: {:?}", e);
        } else {
            info!("Config cleared, resetting system for {}", reason);
            cortex_m::peripheral::SCB::sys_reset();
        }
    })
    .await;
}

pub(crate) async fn save_can_id_to_flash(new_can_id: u8) {
    update_config(
        |config| {
            config.can_id = new_can_id;
        },
        CAN_ID_SAVED_MESSAGE,
    )
    .await;
}

pub(crate) async fn save_output_zero_offset_to_flash(output_zero_offset: f32) {
    update_config(
        |config| {
            config.output_zero_offset = output_zero_offset;
        },
        OUTPUT_ZERO_OFFSET_SAVED_MESSAGE,
    )
    .await;
}
