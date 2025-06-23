use crate::i2c;
use embassy_stm32::i2c::{self as stm32_i2c, Master};
use embassy_stm32::mode::Async;
use embassy_time::Instant;
use foc::angle_input::{AngleInput, AngleReading};
use foc::units::{Radian, Second};

const I2C_ADDRESS: u8 = 0x36; // AS5600 I2C address

const RAW_ANGLE_MAX: u16 = 1 << 12;
const REGISTER_CONF_L: u8 = 0x08;
const REGISTER_RAW_ANGLE_H: u8 = 0x0C;
const REGISTER_STATUS: u8 = 0x0B;

pub struct As5600 {
    previous_raw_angle: u16,
    previous_angle: Radian,
    previous_time: Instant,
}

impl As5600 {
    pub fn new() -> Self {
        As5600 {
            previous_raw_angle: 0,
            previous_angle: Radian(0.0),
            previous_time: Instant::from_secs(0),
        }
    }
}

impl AngleInput for As5600 {
    type Bus = stm32_i2c::I2c<'static, Async, Master>;
    type ReadError = stm32_i2c::Error;

    async fn read_async(&mut self, bus: &mut Self::Bus) -> Result<AngleReading, Self::ReadError> {
        let now = Instant::now();
        let dt = Second((now - self.previous_time).as_micros() as f32 / 1e6);
        let raw_angle = read_raw_angle(bus).await?;
        let delta_1 = {
            if raw_angle >= self.previous_raw_angle {
                (raw_angle - self.previous_raw_angle) as i32
            } else {
                (RAW_ANGLE_MAX as i32 + raw_angle as i32) - (self.previous_raw_angle as i32)
            }
        };
        let delta_2 = {
            if raw_angle <= self.previous_raw_angle {
                (self.previous_raw_angle - raw_angle) as i32
            } else {
                (RAW_ANGLE_MAX as i32 + self.previous_raw_angle as i32) - (raw_angle as i32)
            }
        };

        let raw_angle_change: i32 = {
            if delta_1.abs() < delta_2.abs() {
                delta_1
            } else if delta_1.abs() > delta_2.abs() {
                delta_2
            } else {
                0
            }
        };
        let angular_change = Radian((raw_angle_change as f32) / (RAW_ANGLE_MAX as f32));
        let angle = self.previous_angle + angular_change;
        let velocity = angular_change / dt;
        self.previous_angle = angle;
        self.previous_time = now;
        Ok(AngleReading {
            angle,
            velocity,
            dt,
        })
    }
}

#[derive(defmt::Format)]
pub enum MagnetStatus {
    Ok,
    NoMagnet,
    TooStrong,
    TooWeak,
}

pub async fn read_raw_angle(
    i2c_peripheral: &mut stm32_i2c::I2c<'_, Async, Master>,
) -> Result<u16, stm32_i2c::Error> {
    let mut buffer: [u8; 2] = [0, 0];
    i2c::read(
        i2c_peripheral,
        I2C_ADDRESS,
        REGISTER_RAW_ANGLE_H,
        &mut buffer,
    )
    .await?;
    let high_byte = (buffer[0] & 0b0000_1111) as u16;
    let low_byte = buffer[1] as u16;
    Ok(high_byte << 8 | low_byte)
}

pub async fn read_magnet_status(
    i2c_peripheral: &mut stm32_i2c::I2c<'_, Async, Master>,
) -> Result<MagnetStatus, stm32_i2c::Error> {
    let mut buffer: [u8; 1] = [0];
    i2c::read(i2c_peripheral, I2C_ADDRESS, REGISTER_STATUS, &mut buffer).await?;
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

pub async fn set_digital_output_mode(
    i2c_peripheral: &mut stm32_i2c::I2c<'_, Async, Master>,
) -> Result<(), stm32_i2c::Error> {
    let mut buffer: [u8; 1] = [0];
    i2c::read(i2c_peripheral, I2C_ADDRESS, REGISTER_CONF_L, &mut buffer).await?;
    // Set the bit 5,4 to 0b10 (digital output mode)
    buffer[0] &= 0b1100_1111;
    buffer[0] |= 0b0010_0000;
    i2c::write_byte(i2c_peripheral, I2C_ADDRESS, REGISTER_CONF_L, buffer[0]).await?;

    Ok(())
}
