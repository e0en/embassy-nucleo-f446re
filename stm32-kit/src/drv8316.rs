use embassy_stm32::mode::Async;
use embassy_stm32::{gpio, spi};
use embassy_time::Timer;
use foc::current::PhaseCurrent;

const IC_STATUS_REGISTER: u8 = 0x00u8;
const STATUS_REGISTER_2: u8 = 0x02u8;
const CONTROL_REGISTER_1: u8 = 0x03u8;
const CONTROL_REGISTER_2: u8 = 0x04u8;
const CONTROL_REGISTER_5: u8 = 0x07u8;
const CONTROL_REGISTER_6: u8 = 0x08u8;

pub struct Drv8316<'d> {
    spi_mutex: &'static SpiMutex,
    cs_pin: gpio::Output<'d>,
    drvoff_pin: gpio::Output<'d>,
}

#[allow(dead_code)]
#[derive(Debug, defmt::Format)]
pub enum Error {
    NoBus,
    CommandFrame,
    Parity,
    AddressRange,
    LowLevel(spi::Error),
}

impl From<spi::Error> for Error {
    fn from(value: spi::Error) -> Self {
        Error::LowLevel(value)
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum CsaGain {
    Gain0_15V = 0b00, // 0.15 V/A
    Gain0_3V = 0b01,  // 0.3 V/A
    Gain0_6V = 0b10,  // 0.6 V/A
    Gain1_2V = 0b11,  // 1.2 V/A
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum SlewRate {
    Rate25V = 0b00,  // 25 V/s
    Rate50V = 0b01,  // 50 V/s
    Rate125V = 0b10, // 125 V/s
    Rate200V = 0b11, // 200 V/s
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub enum BuckVoltage {
    V3_3 = 0b0, // 3.3V
    V5_0 = 0b1, // 5.0V
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub enum BuckCurrentLimit {
    Limit200mA = 0b0, // 200mA (default)
    Limit600mA = 0b1, // 600mA (when BUCK_CL = 0)
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub struct BuckRegulatorConfig {
    pub enable: bool,
    pub voltage: BuckVoltage,
    pub current_limit: BuckCurrentLimit,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct IcStatusRegister {
    pub fault: bool,             // Bit 0: Fault condition
    pub ot_warning: bool,        // Bit 1: Over-temperature warning
    pub ot_shutdown: bool,       // Bit 2: Over-temperature shutdown
    pub buck_error: bool,        // Bit 3: Buck regulator error
    pub charge_pump_error: bool, // Bit 4: Charge pump error
    pub ocp_ls_c: bool,          // Bit 5: Overcurrent on low-side C
    pub ocp_hs_c: bool,          // Bit 6: Overcurrent on high-side C
    pub ocp_ls_b: bool,          // Bit 7: Overcurrent on low-side B
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct SpiStatus {
    pub parity_error: bool,
    pub clock_error: bool,
    pub address_error: bool,
}

impl From<u8> for SpiStatus {
    fn from(value: u8) -> Self {
        Self {
            parity_error: (value & 0x04) != 0,
            clock_error: (value & 0x02) != 0,
            address_error: (value & 0x01) != 0,
        }
    }
}

impl From<u8> for IcStatusRegister {
    fn from(value: u8) -> Self {
        Self {
            fault: (value & 0x01) != 0,
            ot_warning: (value & 0x02) != 0,
            ot_shutdown: (value & 0x04) != 0,
            buck_error: (value & 0x08) != 0,
            charge_pump_error: (value & 0x10) != 0,
            ocp_ls_c: (value & 0x20) != 0,
            ocp_hs_c: (value & 0x40) != 0,
            ocp_ls_b: (value & 0x80) != 0,
        }
    }
}

impl<'d> Drv8316<'d> {
    pub fn new(
        spi_mutex: &'static SpiMutex,
        cs_pin: gpio::Output<'d>,
        drvoff_pin: gpio::Output<'d>,
    ) -> Self {
        Drv8316 {
            spi_mutex,
            cs_pin,
            drvoff_pin,
        }
    }

    pub async fn initialize(&mut self, csa_gain: CsaGain, slew_rate: SlewRate) {
        Timer::after_millis(1).await; // ready time of gate driver
        self.unlock_registers().await.unwrap();
        self.set_csa_gain(csa_gain).await.unwrap();
        self.set_slew_rate(slew_rate).await.unwrap();
        let config = BuckRegulatorConfig {
            enable: true,
            voltage: BuckVoltage::V5_0,
            current_limit: BuckCurrentLimit::Limit200mA,
        };
        self.configure_buck_regulator(config).await.unwrap();
        self.lock_registers().await.unwrap();
        Timer::after_millis(1).await; // wait for register value update
    }

    pub fn turn_on(&mut self) {
        self.drvoff_pin.set_low();
    }

    pub fn turn_off(&mut self) {
        self.drvoff_pin.set_low();
    }

    async fn read_address(&mut self, address: u8) -> Result<u16, Error> {
        if let Some(bus) = self.spi_mutex.lock().await.as_mut() {
            let cmd: u16 = (Command {
                type_: CommandType::Read,
                address,
                data: 0x00u8,
            })
            .try_into()?;
            let response = self.transfer(cmd, bus).await?;
            Ok(response)
        } else {
            Err(Error::NoBus)
        }
    }

    async fn write_address(&mut self, address: u8, data: u8) -> Result<(), Error> {
        if let Some(bus) = self.spi_mutex.lock().await.as_mut() {
            let cmd: u16 = (Command {
                type_: CommandType::Write,
                address,
                data,
            })
            .try_into()?;
            self.transfer(cmd, bus).await?;
            Ok(())
        } else {
            Err(Error::NoBus)
        }
    }

    pub async fn set_csa_gain(&mut self, gain: CsaGain) -> Result<(), Error> {
        let current_reg = self.read_address(CONTROL_REGISTER_5).await?;
        let current_data = (current_reg & 0xFF) as u8;
        let new_data = (current_data & 0b11111100) | (gain as u8);
        self.write_address(CONTROL_REGISTER_5, new_data).await
    }

    pub async fn set_slew_rate(&mut self, rate: SlewRate) -> Result<(), Error> {
        let current_reg = self.read_address(CONTROL_REGISTER_2).await?;
        let current_data = (current_reg & 0xFF) as u8;
        let new_data = (current_data & 0b11100111) | ((rate as u8) << 3);
        self.write_address(CONTROL_REGISTER_2, new_data).await
    }

    pub async fn clear_fault(&mut self) -> Result<(), Error> {
        let current_reg = self.read_address(CONTROL_REGISTER_2).await?;
        let new_data = (current_reg & 0xFF) as u8 | 0x01;
        self.write_address(CONTROL_REGISTER_2, new_data).await
    }

    pub async fn unlock_registers(&mut self) -> Result<(), Error> {
        self.write_address(CONTROL_REGISTER_1, 0x03).await
    }

    pub async fn lock_registers(&mut self) -> Result<(), Error> {
        self.write_address(CONTROL_REGISTER_1, 0x06).await
    }

    pub async fn read_ic_status(&mut self) -> Result<IcStatusRegister, Error> {
        let reg_value = self.read_address(IC_STATUS_REGISTER).await?;
        Ok(IcStatusRegister::from((reg_value & 0xFF) as u8))
    }

    #[allow(dead_code)]
    pub async fn read_spi_status(&mut self) -> Result<SpiStatus, Error> {
        let reg_value = self.read_address(STATUS_REGISTER_2).await?;
        Ok(SpiStatus::from((reg_value & 0x07) as u8))
    }

    #[allow(dead_code)]
    pub async fn send_raw_frame(&mut self, frame: u16) -> Result<(), Error> {
        if let Some(bus) = self.spi_mutex.lock().await.as_mut() {
            self.transfer(frame, bus).await?;
            Ok(())
        } else {
            Err(Error::NoBus)
        }
    }

    pub async fn configure_buck_regulator(
        &mut self,
        config: BuckRegulatorConfig,
    ) -> Result<(), Error> {
        let current_reg = self.read_address(CONTROL_REGISTER_6).await?;
        let current_data = (current_reg & 0xFF) as u8;

        let mut new_data = current_data & 0b1111_1000;

        if !config.enable {
            new_data |= 0b0000_0001;
        }
        new_data |= (config.voltage as u8) << 1;
        new_data |= (config.current_limit as u8) << 2;
        self.write_address(CONTROL_REGISTER_6, new_data).await
    }

    async fn transfer<'a>(
        &mut self,
        word: u16,
        spi_peripheral: &mut spi::Spi<'a, Async>,
    ) -> Result<u16, Error> {
        let mut result = [0x0000u16; 1];
        self.cs_pin.set_low();
        spi_peripheral.transfer(&mut result, &[word]).await?;
        self.cs_pin.set_high();
        Ok(result[0])
    }
}

enum CommandType {
    Read,
    Write,
}

struct Command {
    pub type_: CommandType,
    pub address: u8, // 6 bits
    pub data: u8,    // 8 bits
}

impl TryFrom<Command> for u16 {
    type Error = Error;
    fn try_from(value: Command) -> Result<Self, Self::Error> {
        if value.address >= 1 << 6 {
            return Err(Error::AddressRange);
        }
        let mut command = match value.type_ {
            CommandType::Read => 0x8000u16,
            CommandType::Write => 0x0000u16,
        };
        command |= (value.address as u16) << 9;
        command |= value.data as u16;
        let parity = command.count_ones() % 2;
        Ok(command | ((parity as u16) << 8))
    }
}

type SpiMutex = embassy_sync::mutex::Mutex<
    embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    Option<spi::Spi<'static, Async>>,
>;

pub fn convert_csa_readings(ia_raw: u16, ib_raw: u16, ic_raw: u16, gain: CsaGain) -> PhaseCurrent {
    let mut ia = (ia_raw as f32) * 3.3 / 4096.0 - 3.3 / 2.0;
    let mut ib = (ib_raw as f32) * 3.3 / 4096.0 - 3.3 / 2.0;
    let mut ic = (ic_raw as f32) * 3.3 / 4096.0 - 3.3 / 2.0;

    ia = 0.995832 * ia - 0.028119 * ib - 0.014988 * ic;
    ib = 0.037737 * ia + 1.007723 * ib - 0.033757 * ic;
    ic = 0.009226 * ia + 0.029805 * ib + 1.003268 * ic;
    let gain_value = match gain {
        CsaGain::Gain0_15V => 0.15,
        CsaGain::Gain0_3V => 0.3,
        CsaGain::Gain0_6V => 0.6,
        CsaGain::Gain1_2V => 1.2,
    };

    ia /= gain_value;
    ib /= gain_value;
    ic /= gain_value;
    PhaseCurrent::new(ia, ib, ic)
}
