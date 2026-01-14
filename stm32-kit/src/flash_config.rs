//! Persistent configuration storage in internal flash.
//!
//! Stores motor calibration data, PID gains, and CAN ID in the last flash page (2KB).
//! STM32G431RB: 128KB flash, 2KB pages. Config page at offset 0x1F800.

use embassy_stm32::flash::{Blocking, Flash, WRITE_SIZE};
use foc::controller::Direction;
use foc::current::PhaseCurrent;

/// Magic number to identify valid config: "MOTC" in ASCII
const CONFIG_MAGIC: u32 = 0x4D4F5443;
/// Current config version for future migrations
const CONFIG_VERSION: u8 = 1;
/// Flash offset for config page (last 2KB of 128KB)
const CONFIG_OFFSET: u32 = 0x1F800;
/// Size of config page
const CONFIG_PAGE_SIZE: u32 = 2048;

/// CRC32 lookup table (IEEE 802.3 polynomial)
const CRC32_TABLE: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = i as u32;
        let mut j = 0;
        while j < 8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
            j += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
};

fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFF_u32;
    for &byte in data {
        let index = ((crc ^ (byte as u32)) & 0xFF) as usize;
        crc = (crc >> 8) ^ CRC32_TABLE[index];
    }
    !crc
}

/// Persistent motor configuration stored in flash
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ConfigData {
    /// Magic number for validation
    pub magic: u32,
    /// Config version for migrations
    pub version: u8,

    // Calibration data
    /// Phase-to-ADC channel mapping
    pub current_mapping: (u8, u8, u8),
    /// Sensor zero position offset (radians)
    pub bias_angle: f32,
    /// Sensor direction (0 = CW, 1 = CCW)
    pub sensor_direction: u8,
    /// Current measurement phase offset (radians)
    pub current_phase_bias: f32,
    /// ADC zero-current offsets
    pub current_offset_a: f32,
    pub current_offset_b: f32,
    pub current_offset_c: f32,

    // PID gains
    pub current_kp: f32,
    pub current_ki: f32,
    pub velocity_kp: f32,
    pub velocity_ki: f32,
    pub angle_kp: f32,

    // System config
    pub can_id: u8,

    /// CRC32 checksum (must be last field)
    pub crc: u32,
}

impl ConfigData {
    /// Size of data before CRC field
    const DATA_SIZE: usize = core::mem::size_of::<Self>() - 4;

    pub fn new() -> Self {
        Self {
            magic: CONFIG_MAGIC,
            version: CONFIG_VERSION,
            current_mapping: (0, 1, 2),
            bias_angle: 0.0,
            sensor_direction: 0,
            current_phase_bias: 0.0,
            current_offset_a: 0.0,
            current_offset_b: 0.0,
            current_offset_c: 0.0,
            current_kp: 0.0,
            current_ki: 0.0,
            velocity_kp: 0.0,
            velocity_ki: 0.0,
            angle_kp: 0.0,
            can_id: 0x0F,
            crc: 0,
        }
    }

    /// Convert struct to bytes for flash storage
    fn as_bytes(&self) -> [u8; core::mem::size_of::<Self>()] {
        unsafe { core::mem::transmute_copy(self) }
    }

    /// Create struct from bytes read from flash
    fn from_bytes(bytes: &[u8; core::mem::size_of::<Self>()]) -> Self {
        unsafe { core::mem::transmute_copy(bytes) }
    }

    /// Compute CRC over data fields (excluding CRC field itself)
    fn compute_crc(&self) -> u32 {
        let bytes = self.as_bytes();
        crc32(&bytes[..Self::DATA_SIZE])
    }

    /// Update CRC field to match current data
    pub fn update_crc(&mut self) {
        self.crc = self.compute_crc();
    }

    /// Validate magic, version, and CRC
    pub fn is_valid(&self) -> bool {
        self.magic == CONFIG_MAGIC
            && self.version == CONFIG_VERSION
            && self.crc == self.compute_crc()
    }

    pub fn get_sensor_direction(&self) -> Direction {
        match self.sensor_direction {
            0 => Direction::Clockwise,
            _ => Direction::CounterClockWise,
        }
    }

    pub fn set_sensor_direction(&mut self, dir: Direction) {
        self.sensor_direction = match dir {
            Direction::Clockwise => 0,
            Direction::CounterClockWise => 1,
        };
    }

    pub fn get_current_offset(&self) -> PhaseCurrent {
        PhaseCurrent::new(
            self.current_offset_a,
            self.current_offset_b,
            self.current_offset_c,
        )
    }

    pub fn set_current_offset(&mut self, offset: PhaseCurrent) {
        self.current_offset_a = offset.a;
        self.current_offset_b = offset.b;
        self.current_offset_c = offset.c;
    }
}

#[derive(Debug, defmt::Format)]
#[allow(dead_code)]
pub enum ConfigError {
    FlashError,
    InvalidConfig,
}

/// Read configuration from flash. Returns None if no valid config exists.
pub fn read_config(flash: &mut Flash<'_, Blocking>) -> Option<ConfigData> {
    let mut buffer = [0u8; core::mem::size_of::<ConfigData>()];

    // Read from flash at config offset
    if flash.blocking_read(CONFIG_OFFSET, &mut buffer).is_err() {
        return None;
    }

    let config = ConfigData::from_bytes(&buffer);
    if config.is_valid() {
        Some(config)
    } else {
        None
    }
}

/// Write configuration to flash. Erases the config page first.
pub fn write_config(
    flash: &mut Flash<'_, Blocking>,
    config: &mut ConfigData,
) -> Result<(), ConfigError> {
    // Ensure CRC is up to date
    config.update_crc();

    let bytes = config.as_bytes();

    // Pad to WRITE_SIZE alignment (typically 8 bytes for STM32G4)
    let write_size = WRITE_SIZE;
    let aligned_len = bytes.len().div_ceil(write_size) * write_size;
    let mut aligned_buffer = [0xFFu8; 256]; // Max reasonable config size
    aligned_buffer[..bytes.len()].copy_from_slice(&bytes);

    // Erase the config page
    flash
        .blocking_erase(CONFIG_OFFSET, CONFIG_OFFSET + CONFIG_PAGE_SIZE)
        .map_err(|_| ConfigError::FlashError)?;

    // Write config data
    flash
        .blocking_write(CONFIG_OFFSET, &aligned_buffer[..aligned_len])
        .map_err(|_| ConfigError::FlashError)?;

    Ok(())
}

/// Erase configuration page (clears stored config)
pub fn clear_config(flash: &mut Flash<'_, Blocking>) -> Result<(), ConfigError> {
    flash
        .blocking_erase(CONFIG_OFFSET, CONFIG_OFFSET + CONFIG_PAGE_SIZE)
        .map_err(|_| ConfigError::FlashError)
}

/// Apply stored configuration to FOC controller
pub fn apply_to_foc<Fsincos>(foc: &mut foc::controller::FocController<Fsincos>, config: &ConfigData)
where
    Fsincos: Fn(f32) -> (f32, f32),
{
    foc.current_mapping = config.current_mapping;
    foc.bias_angle = config.bias_angle;
    foc.sensor_direction = config.get_sensor_direction();
    foc.set_current_phase_bias(config.current_phase_bias);
    foc.set_current_offset(config.get_current_offset());

    foc.set_current_kp(config.current_kp);
    foc.set_current_ki(config.current_ki);
    foc.set_velocity_kp(config.velocity_kp);
    foc.set_velocity_ki(config.velocity_ki);
    foc.set_angle_kp(config.angle_kp);
}

/// Create ConfigData from current FOC state
pub fn from_foc<Fsincos>(foc: &foc::controller::FocController<Fsincos>, can_id: u8) -> ConfigData
where
    Fsincos: Fn(f32) -> (f32, f32),
{
    let current_offset = foc.current_offset();
    let mut config = ConfigData::new();

    config.current_mapping = foc.current_mapping;
    config.bias_angle = foc.bias_angle;
    config.set_sensor_direction(foc.sensor_direction);
    config.current_phase_bias = foc.current_phase_bias;
    config.set_current_offset(current_offset);

    config.current_kp = foc.current_q_pid.gains.p;
    config.current_ki = foc.current_q_pid.gains.i;
    config.velocity_kp = foc.velocity_pid.gains.p;
    config.velocity_ki = foc.velocity_pid.gains.i;
    config.angle_kp = foc.angle_pid.gains.p;

    config.can_id = can_id;
    config
}
