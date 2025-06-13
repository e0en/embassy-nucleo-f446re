use crate::i2c;
use embassy_stm32::i2c as stm32_i2c;
use embassy_stm32::mode::Blocking;

const I2C_ADDRESS: u8 = 0x36; // AS5600 I2C address

const REGISTER_CONF_H: u8 = 0x07;
const REGISTER_CONF_L: u8 = 0x08;
const REGISTER_RAW_ANGLE_H: u8 = 0x0C;
const REGISTER_RAW_ANGLE_L: u8 = 0x0D;
const REGISTER_STATUS: u8 = 0x0B;

#[derive(defmt::Format)]
pub enum MagnetStatus {
    Ok,
    NoMagnet,
    TooStrong,
    TooWeak,
}

pub fn read_raw_angle(
    i2c_peripheral: &mut stm32_i2c::I2c<'_, Blocking>,
) -> Result<u16, stm32_i2c::Error> {
    let mut buffer: [u8; 2] = [0, 0];
    i2c::blocking_read(
        i2c_peripheral,
        I2C_ADDRESS,
        REGISTER_RAW_ANGLE_H,
        &mut buffer,
    )?;
    let high_byte = (buffer[0] & 0b0000_1111) as u16;
    let low_byte = buffer[1] as u16;
    Ok(high_byte << 8 | low_byte)
}

pub fn read_magnet_status(
    i2c_peripheral: &mut stm32_i2c::I2c<'_, Blocking>,
) -> Result<MagnetStatus, stm32_i2c::Error> {
    let mut buffer: [u8; 1] = [0];
    i2c::blocking_read(i2c_peripheral, I2C_ADDRESS, REGISTER_STATUS, &mut buffer)?;
    let byte = &buffer[0];
    let is_too_strong = (byte & 0b0000_1000) != 0;
    let is_too_weak = (byte & 0b0001_0000) != 0;
    let is_detected = (byte & 0b0010_0000) != 0;

    if is_too_strong {
        return Ok(MagnetStatus::TooStrong);
    } else if is_too_weak {
        return Ok(MagnetStatus::TooWeak);
    } else if is_detected {
        return Ok(MagnetStatus::Ok);
    }
    Ok(MagnetStatus::NoMagnet)
}

pub fn set_digital_output_mode(
    i2c_peripheral: &mut stm32_i2c::I2c<'_, Blocking>,
) -> Result<(), stm32_i2c::Error> {
    let mut buffer: [u8; 1] = [0];
    i2c::blocking_read(i2c_peripheral, I2C_ADDRESS, REGISTER_CONF_L, &mut buffer)?;
    // Set the bit 5,4 to 0b10 (digital output mode)
    buffer[0] &= 0b1100_1111;
    buffer[0] |= 0b0010_0000;
    i2c::blocking_write_byte(i2c_peripheral, I2C_ADDRESS, REGISTER_CONF_L, buffer[0])?;

    Ok(())
}
