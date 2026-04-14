use defmt::{error, info};
use embassy_stm32::mode::Async;
use embassy_stm32::{gpio, spi};
use embassy_time::{Duration, Instant, Timer};
use foc::angle_input::{AngleInput, AngleReading};
use foc::units::{Radian, RadianPerSecond, Second};

const RAW_ANGLE_MAX: u16 = 1 << 12;
// Full rotation angle in radians
const TWO_PI: f32 = 2.0 * core::f32::consts::PI;
const RAW_TO_RADIAN: f32 = TWO_PI / (RAW_ANGLE_MAX as f32);

// AS5048A register addresses (from datasheet)
const REG_ANGLE: u16 = 0x3FFF;
const REG_DIAGNOSTICS: u16 = 0x3FFD;
const REG_CLEAR_ERROR: u16 = 0x0001;

// AS5048A SPI protocol constants
const SPI_READ_COMMAND: u16 = 0b_0100_0000_0000_0000;
const SPI_ERROR_FLAG_MASK: u16 = 0x8000;
const SPI_DATA_MASK: u16 = 0x7FFF;

// AS5048A diagnostic register bit masks
const DIAG_COMP_LOW: u16 = 0x0800;
const DIAG_COMP_HIGH: u16 = 0x0400;
const DIAG_COF: u16 = 0x0200;
const DIAG_OCF: u16 = 0x0100;
const DIAG_GAIN_MASK: u16 = 0x00FF;

// Timing constants (milliseconds)
const INIT_WAIT_MS: u64 = 15;

// Conversion constant for microseconds to seconds
const MICROS_PER_SECOND: f32 = 1e6;

pub struct As5048A<'d> {
    previous_raw_angle: Option<u16>,
    previous_angle: Radian,
    previous_time: Instant,
    full_rotations: i32,
    spi_mutex: &'static SpiMutex,
    cs_pin: gpio::Output<'d>,
}

#[derive(Debug, defmt::Format)]
pub enum Error {
    NoBus,
    ErrorFlag,
    LowLevel(#[defmt(Debug2Format)] spi::Error),
}

impl From<spi::Error> for Error {
    fn from(value: spi::Error) -> Self {
        Error::LowLevel(value)
    }
}

impl<'d> As5048A<'d> {
    pub fn new(spi_mutex: &'static SpiMutex, cs_pin: gpio::Output<'d>) -> Self {
        As5048A {
            previous_raw_angle: None,
            previous_angle: Radian(0.0),
            previous_time: Instant::from_secs(0),
            full_rotations: 0,
            spi_mutex,
            cs_pin,
        }
    }

    pub async fn initialize(&mut self) {
        // startup time = 10ms
        let _ = self.reset_error_flag().await;
        if let Ok(diag) = self.read_diagnostics().await {
            info!("{:?}", diag);
        } else {
            error!("initialization error");
        }
        Timer::after(Duration::from_millis(INIT_WAIT_MS)).await;
    }

    pub async fn read_raw_angle(&mut self) -> Result<u16, Error> {
        self.read_address(REG_ANGLE).await
    }

    pub async fn read_raw_angle_fast(&mut self) -> Result<u16, Error> {
        self.read_address(REG_ANGLE).await
    }

    pub async fn read_diagnostics(&mut self) -> Result<Diagnostics, Error> {
        let response = self.read_address(REG_DIAGNOSTICS).await?;
        Ok(Diagnostics {
            is_comp_low: response & DIAG_COMP_LOW != 0,
            is_comp_high: response & DIAG_COMP_HIGH != 0,
            cof: response & DIAG_COF != 0,
            ocf: response & DIAG_OCF != 0,
            gain: (response & DIAG_GAIN_MASK) as u8,
        })
    }

    pub async fn reset_error_flag(&mut self) -> Result<u16, Error> {
        self.read_address(REG_CLEAR_ERROR).await
    }

    pub async fn test_cs_pin(&mut self) {
        self.cs_pin.set_low();
        Timer::after(Duration::from_secs(1)).await;
        self.cs_pin.set_high();
        Timer::after(Duration::from_secs(1)).await;
    }

    async fn read_address(&mut self, address: u16) -> Result<u16, Error> {
        if let Some(spi_peripheral) = self.spi_mutex.lock().await.as_mut() {
            let cmd = SPI_READ_COMMAND | address;
            let parity = cmd.count_ones() % 2;
            let request = (cmd | (parity as u16) << 15).to_be_bytes();
            self.cs_pin.set_low();
            let _ = spi_peripheral.write(&request).await;
            self.cs_pin.set_high();

            let mut response_be_bytes = [0x00u8; 2];
            self.cs_pin.set_low();
            spi_peripheral
                .transfer_in_place(&mut response_be_bytes)
                .await?;
            self.cs_pin.set_high();
            let response = u16::from_be_bytes(response_be_bytes);
            let err_flag = response & SPI_ERROR_FLAG_MASK;
            if err_flag != 0 {
                return Err(Error::ErrorFlag);
            }
            let result = response & SPI_DATA_MASK;
            Ok(result)
        } else {
            Err(Error::NoBus)
        }
    }

    async fn read_address_sequence(&mut self, address: u16) -> Result<u16, Error> {
        if let Some(spi_peripheral) = self.spi_mutex.lock().await.as_mut() {
            // ensure that the previous request was also for the same address
            let parity = address.count_ones() % 2;
            let request = [address | (parity as u16) << 15];

            let mut response = [0x00u16; 1];

            spi_peripheral.transfer(&mut response, &request).await?;
            let err_flag = response[0] & SPI_ERROR_FLAG_MASK;
            if err_flag != 0 {
                return Err(Error::ErrorFlag);
            }
            Ok(response[0] & SPI_DATA_MASK)
        } else {
            Err(Error::NoBus)
        }
    }
}

type SpiMutex = embassy_sync::mutex::Mutex<
    embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    Option<spi::Spi<'static, Async>>,
>;

impl<'d> AngleInput for As5048A<'d> {
    type ReadError = Error;

    async fn read_async(&mut self) -> Result<AngleReading, Self::ReadError> {
        let now = Instant::now();
        let dt = Second((now - self.previous_time).as_micros() as f32 / MICROS_PER_SECOND);
        let raw_angle = self.read_raw_angle().await?;
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
                if angle.0 > TWO_PI {
                    angle.0 -= TWO_PI;
                    self.full_rotations += 1;
                } else if angle.0 < -TWO_PI {
                    angle.0 += TWO_PI;
                    self.full_rotations -= 1;
                }
                let velocity = if dt.0 > 0.0 {
                    angular_change / dt
                } else {
                    RadianPerSecond(0.0)
                };
                self.previous_raw_angle = Some(raw_angle);
                self.previous_angle = angle;
                self.previous_time = now;
                Ok(AngleReading {
                    angle: angle + TWO_PI * self.full_rotations as f32,
                    velocity,
                    dt,
                })
            }
        }
    }
}

#[derive(defmt::Format)]
pub struct Diagnostics {
    pub is_comp_low: bool,
    pub is_comp_high: bool,
    pub cof: bool,
    pub ocf: bool,
    pub gain: u8,
}
