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

pub(crate) async fn save_runtime_config() {
    let mut flash_guard = FLASH.lock().await;
    if let Some(flash) = flash_guard.as_mut() {
        let stored_config = flash_config::read_config(flash);
        let can_id = stored_config.map(|c| c.can_id).unwrap_or(0x0F);
        let primary_zero_offset = stored_config.map(|c| c.primary_zero_offset).unwrap_or(0.0);
        let secondary_zero_offset = stored_config
            .map(|c| c.secondary_zero_offset)
            .unwrap_or(0.0);

        foc_isr::with_foc(|foc| {
            let mut config = flash_config::from_foc(foc, can_id);
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

pub(crate) async fn clear_config_and_reset(reason: &'static str) {
    let mut flash_guard = FLASH.lock().await;
    if let Some(flash) = flash_guard.as_mut() {
        if let Err(e) = flash_config::clear_config(flash) {
            error!("Failed to clear config: {:?}", e);
        } else {
            info!("Config cleared, resetting system for {}", reason);
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}

pub(crate) async fn save_can_id_to_flash(new_can_id: u8) {
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

pub(crate) async fn save_zero_offsets_to_flash(
    primary_zero_offset: f32,
    secondary_zero_offset: f32,
) {
    let mut flash_guard = FLASH.lock().await;
    if let Some(flash) = flash_guard.as_mut()
        && let Some(mut config) = flash_config::read_config(flash)
    {
        config.primary_zero_offset = primary_zero_offset;
        config.secondary_zero_offset = secondary_zero_offset;
        if let Err(e) = flash_config::write_config(flash, &mut config) {
            warn!("Failed to save zero offsets to flash: {:?}", e);
        } else {
            info!("Zero offsets saved to flash");
        }
    }
}
