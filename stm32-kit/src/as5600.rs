use crate::i2c;
use embassy_stm32::i2c::{self as stm32_i2c, Master};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Instant, Timer};
use foc::angle_input::{AngleInput, AngleReading};
use foc::units::{Radian, RadianPerSecond, Second};

const I2C_ADDRESS: u8 = 0x36; // AS5600 I2C address

const RAW_ANGLE_MAX: u16 = 1 << 12;
const RAW_TO_RADIAN: f32 = 2.0 * core::f32::consts::PI / (RAW_ANGLE_MAX as f32);
const REGISTER_CONF_H: u8 = 0x07;
const REGISTER_CONF_L: u8 = 0x08;
const REGISTER_RAW_ANGLE_H: u8 = 0x0C;
const REGISTER_STATUS: u8 = 0x0B;

pub struct As5600 {
    previous_raw_angle: Option<u16>,
    previous_angle: Radian,
    previous_time: Instant,
    full_rotations: i32,
}

impl As5600 {
    pub fn new() -> Self {
        As5600 {
            previous_raw_angle: None,
            previous_angle: Radian(0.0),
            previous_time: Instant::from_secs(0),
            full_rotations: 0,
        }
    }

    pub async fn initialize(
        &mut self,
        i2c_peripheral: &mut stm32_i2c::I2c<'_, Async, Master>,
    ) -> Result<(), stm32_i2c::Error> {
        set_fast_sampling(i2c_peripheral).await?;
        Timer::after(Duration::from_millis(10)).await;
        set_digital_output_mode(i2c_peripheral).await?;
        Timer::after(Duration::from_millis(10)).await;
        Ok(())
    }
}

impl AngleInput for As5600 {
    type Bus = stm32_i2c::I2c<'static, Async, Master>;
    type ReadError = stm32_i2c::Error;

    async fn read_async(&mut self, bus: &mut Self::Bus) -> Result<AngleReading, Self::ReadError> {
        let now = Instant::now();
        let dt = Second((now - self.previous_time).as_micros() as f32 / 1e6);
        let raw_angle = read_raw_angle(bus).await?;
        match self.previous_raw_angle {
            None => {
                self.previous_time = now;
                self.previous_raw_angle = Some(raw_angle);
                self.previous_angle = Radian(raw_angle as f32 * RAW_TO_RADIAN);
                Ok(AngleReading {
                    angle: self.previous_angle,
                    velocity: RadianPerSecond(0.0),
                    dt: Second(0.0),
                })
            }
            Some(previous_raw_angle) => {
                let delta_1 = {
                    if raw_angle >= previous_raw_angle {
                        (raw_angle as i32) - (previous_raw_angle as i32)
                    } else {
                        (RAW_ANGLE_MAX as i32 + raw_angle as i32) - (previous_raw_angle as i32)
                    }
                };
                let delta_2 = {
                    if raw_angle <= previous_raw_angle {
                        (previous_raw_angle as i32) - (raw_angle as i32)
                    } else {
                        (RAW_ANGLE_MAX as i32 + previous_raw_angle as i32) - (raw_angle as i32)
                    }
                };

                let raw_angle_change: i32 = {
                    if delta_1.abs() < delta_2.abs() {
                        delta_1
                    } else if delta_1.abs() > delta_2.abs() {
                        -delta_2
                    } else {
                        0
                    }
                };
                let angular_change = Radian((raw_angle_change as f32) * RAW_TO_RADIAN);
                let mut angle = self.previous_angle + angular_change;
                if angle.0 > 2.0 * core::f32::consts::PI {
                    angle.0 -= 2.0 * core::f32::consts::PI;
                    self.full_rotations += 1;
                } else if angle.0 < -2.0 * core::f32::consts::PI {
                    angle.0 += 2.0 * core::f32::consts::PI;
                    self.full_rotations -= 1;
                }
                let velocity = angular_change / dt;
                self.previous_raw_angle = Some(raw_angle);
                self.previous_angle = angle;
                self.previous_time = now;
                Ok(AngleReading {
                    angle: angle + 2.0 * core::f32::consts::PI * self.full_rotations as f32,
                    velocity,
                    dt,
                })
            }
        }
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

pub async fn set_fast_sampling(
    i2c_peripheral: &mut stm32_i2c::I2c<'_, Async, Master>,
) -> Result<(), stm32_i2c::Error> {
    let mut buffer: [u8; 1] = [0];
    i2c::read(i2c_peripheral, I2C_ADDRESS, REGISTER_CONF_H, &mut buffer).await?;
    // set bit 1~0 to 0b11 (fastest sampling)
    buffer[0] &= 0b1111_1100;
    buffer[0] |= 0b0000_0011;
    i2c::write_byte(i2c_peripheral, I2C_ADDRESS, REGISTER_CONF_H, buffer[0]).await?;

    Ok(())
}
