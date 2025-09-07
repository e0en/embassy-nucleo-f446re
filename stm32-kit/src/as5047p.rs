use embassy_stm32::mode::Async;
use embassy_stm32::{gpio, spi};
use embassy_time::{Duration, Instant, Timer};
use foc::angle_input::{AngleInput, AngleReading};
use foc::units::{Radian, RadianPerSecond, Second};

const RAW_ANGLE_MAX: u16 = 1 << 14;
const RAW_TO_RADIAN: f32 = 2.0 * core::f32::consts::PI / (RAW_ANGLE_MAX as f32);

pub struct As5047P<'d> {
    previous_raw_angle: Option<u16>,
    previous_angle: Radian,
    previous_time: Instant,
    full_rotations: i32,
    spi_mutex: &'static SpiMutex,
    cs_pin: gpio::Output<'d>,
}

#[allow(dead_code)]
#[derive(Debug, defmt::Format)]
pub enum Error {
    NoBus,
    CommandFrame,
    Parity,
    LowLevel(spi::Error),
}

#[derive(defmt::Format)]
pub struct ErrorFlag {
    pub parity_error: bool,
    pub invalid_command: bool,
    pub framing_error: bool,
}

impl From<spi::Error> for Error {
    fn from(value: spi::Error) -> Self {
        Error::LowLevel(value)
    }
}

impl<'d> As5047P<'d> {
    pub fn new(spi_mutex: &'static SpiMutex, cs_pin: gpio::Output<'d>) -> Self {
        As5047P {
            previous_raw_angle: None,
            previous_angle: Radian::new(0.0),
            previous_time: Instant::from_secs(0),
            full_rotations: 0,
            spi_mutex,
            cs_pin,
        }
    }

    pub async fn initialize(&mut self) -> Result<Diagnostics, Error> {
        // startup time = 10ms
        Timer::after(Duration::from_millis(20)).await;
        // ignore the first error response
        let _ = self.read_and_reset_error_flag().await;
        self.read_and_reset_error_flag().await?;
        self.read_diagnostics().await
    }

    pub async fn read_raw_angle(&mut self) -> Result<u16, Error> {
        self.read_address(0x3FFF).await
    }

    pub async fn read_diagnostics(&mut self) -> Result<Diagnostics, Error> {
        let response = self.read_address(0x3FFC).await?;
        Ok(Diagnostics {
            magnet_weak: response & 0x0800 != 0,
            magnet_strong: response & 0x0400 != 0,
            cof: response & 0x0200 != 0,
            lf: response & 0x0100 != 0,
            agc: (response & 0xFF) as u8,
        })
    }

    #[allow(dead_code)]
    pub async fn get_cordic_magnitude(&mut self) -> Result<u16, Error> {
        self.read_address(0x3FFD).await
    }

    pub async fn read_and_reset_error_flag(&mut self) -> Result<ErrorFlag, Error> {
        let response = self.read_address(0x0001).await?;
        Ok(ErrorFlag {
            parity_error: response & 0x0004 != 0,
            invalid_command: response & 0x0002 != 0,
            framing_error: response & 0x0001 != 0,
        })
    }

    async fn read_address(&mut self, address: u16) -> Result<u16, Error> {
        if let Some(bus) = self.spi_mutex.lock().await.as_mut() {
            let cmd = to_read_command(&address);
            let nop = to_read_command(&0x00);
            self.transfer(cmd, bus).await?;
            let response_be_bytes = self.transfer(nop, bus).await?;
            let response = u16::from_be_bytes(response_be_bytes);
            if response.count_ones() % 2 != 0 {
                return Err(Error::Parity);
            }
            if response & 0x4000 != 0 {
                return Err(Error::CommandFrame);
            }
            let result = response & 0x3FFFu16;
            Ok(result)
        } else {
            Err(Error::NoBus)
        }
    }

    async fn transfer<'a>(
        &mut self,
        bytes: [u8; 2],
        spi_peripheral: &mut spi::Spi<'a, Async>,
    ) -> Result<[u8; 2], Error> {
        let mut result = [0x00u8; 2];
        self.cs_pin.set_low();
        spi_peripheral.transfer(&mut result, &bytes).await?;
        self.cs_pin.set_high();
        Ok(result)
    }
}

fn to_read_command(address: &u16) -> [u8; 2] {
    let cmd = 0x4000u16 | (address & 0x3FFFu16);
    let parity = cmd.count_ones() % 2;
    (cmd | (parity as u16) << 15).to_be_bytes()
}

type SpiMutex = embassy_sync::mutex::Mutex<
    embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    Option<spi::Spi<'static, Async>>,
>;

impl<'d> AngleInput for As5047P<'d> {
    type ReadError = Error;

    async fn read_async(&mut self) -> Result<AngleReading, Self::ReadError> {
        let now = Instant::now();
        let dt = Second((now - self.previous_time).as_micros() as f32 / 1e6);
        let raw_angle = self.read_raw_angle().await?;
        match self.previous_raw_angle {
            None => {
                self.previous_time = now;
                self.previous_raw_angle = Some(raw_angle);
                self.previous_angle = Radian::new(raw_angle as f32 * RAW_TO_RADIAN);
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
                let angular_change = Radian::new((raw_angle_change as f32) * RAW_TO_RADIAN);
                let mut angle = self.previous_angle + angular_change;
                if angle.angle > 2.0 * core::f32::consts::PI {
                    angle.angle -= 2.0 * core::f32::consts::PI;
                    self.full_rotations += 1;
                } else if angle.angle < -2.0 * core::f32::consts::PI {
                    angle.angle += 2.0 * core::f32::consts::PI;
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
pub struct Diagnostics {
    pub magnet_weak: bool,
    pub magnet_strong: bool,
    pub cof: bool,
    pub lf: bool,
    pub agc: u8,
}

#[allow(dead_code)]
impl Diagnostics {
    pub fn is_okay(&self) -> bool {
        !self.magnet_weak
            && !self.magnet_strong
            && !self.cof
            && self.lf
            && self.agc > 0
            && self.agc < 255
            && !self.is_all_zero()
    }

    pub fn is_all_zero(&self) -> bool {
        !self.magnet_weak && !self.magnet_strong && !self.cof && !self.lf && self.agc == 0
    }
}
