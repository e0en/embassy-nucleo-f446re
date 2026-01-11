#[repr(u8)]
#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum RunMode {
    Impedance = 0x00,
    Angle = 0x01,
    Velocity = 0x02,
    Torque = 0x03,
    Voltage = 0x04,
}

impl TryFrom<u8> for RunMode {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(RunMode::Impedance),
            0x01 => Ok(RunMode::Angle),
            0x02 => Ok(RunMode::Velocity),
            0x03 => Ok(RunMode::Torque),
            0x04 => Ok(RunMode::Voltage),
            _ => Err(()),
        }
    }
}

pub struct CanMessage {
    pub id: u32,
    pub data: [u8; 8],
    pub length: u8,
}

#[derive(Copy, Clone)]
pub struct ResponseMessage {
    pub motor_can_id: u8,
    pub host_can_id: u8,
    pub body: ResponseBody,
}

#[derive(Copy, Clone)]
pub enum ResponseBody {
    MotorStatus(MotorStatus),
    MotorCurrent(MotorCurrent),
    ParameterValue(ParameterValue),
}

#[repr(u16)]
#[derive(Copy, Clone)]
pub enum ParameterIndex {
    RunMode = 0x7005,
    IqRef = 0x7006,
    SpeedRef = 0x700A,
    TorqueLimit = 0x700B,
    CurrentKp = 0x7010,
    CurrentKi = 0x7011,
    CurrentFilter = 0x7014,
    AngleRef = 0x7016,
    SpeedLimit = 0x7017,
    CurrentLimit = 0x7018,
    Angle = 0x7019,
    Iq = 0x701A,
    Speed = 0x701B,
    AngleKp = 0x701E,
    SpeedKp = 0x701F,
    SpeedKi = 0x7020,

    Spring = 0x7021,
    Damping = 0x7022,
    VqRef = 0x7023,
}

impl TryFrom<u16> for ParameterIndex {
    type Error = ();
    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            0x7005 => Ok(Self::RunMode),
            0x7006 => Ok(Self::IqRef),
            0x700A => Ok(Self::SpeedRef),
            0x700B => Ok(Self::TorqueLimit),
            0x7010 => Ok(Self::CurrentKp),
            0x7011 => Ok(Self::CurrentKi),
            0x7014 => Ok(Self::CurrentFilter),
            0x7016 => Ok(Self::AngleRef),
            0x7017 => Ok(Self::SpeedLimit),
            0x7018 => Ok(Self::CurrentLimit),
            0x7019 => Ok(Self::Angle),
            0x701A => Ok(Self::Iq),
            0x701B => Ok(Self::Speed),
            0x701E => Ok(Self::AngleKp),
            0x701F => Ok(Self::SpeedKp),
            0x7020 => Ok(Self::SpeedKi),
            0x7021 => Ok(Self::Spring),
            0x7022 => Ok(Self::Damping),
            0x7023 => Ok(Self::VqRef),
            _ => Err(()),
        }
    }
}

#[derive(Copy, Clone)]
pub enum ParameterValue {
    RunMode(RunMode),
    IqRef(f32),
    SpeedRef(f32),
    TorqueLimit(f32),
    CurrentKp(f32),
    CurrentKi(f32),
    CurrentFilter(f32),
    AngleRef(f32),
    SpeedLimit(f32),
    CurrentLimit(f32),
    Angle(f32),
    Iq(f32),
    Speed(f32),
    AngleKp(f32),
    SpeedKp(f32),
    SpeedKi(f32),
    Spring(f32),
    Damping(f32),
    VqRef(f32),
}

impl TryFrom<[u8; 8]> for ParameterValue {
    type Error = ();
    fn try_from(data: [u8; 8]) -> Result<Self, Self::Error> {
        let address = (data[1] as u16) << 8 | data[0] as u16;
        let index = ParameterIndex::try_from(address)?;

        let mut value_buffer = [0x00u8; 4];
        value_buffer.copy_from_slice(&data[4..8]);
        let value = f32::from_le_bytes(value_buffer);

        Ok(match index {
            ParameterIndex::RunMode => {
                let mode = RunMode::try_from(data[4])?;
                Self::RunMode(mode)
            }
            ParameterIndex::IqRef => Self::IqRef(value),
            ParameterIndex::SpeedRef => Self::SpeedRef(value),
            ParameterIndex::TorqueLimit => Self::TorqueLimit(value),
            ParameterIndex::CurrentKp => Self::CurrentKp(value),
            ParameterIndex::CurrentKi => Self::CurrentKi(value),
            ParameterIndex::CurrentFilter => Self::CurrentFilter(value),
            ParameterIndex::AngleRef => Self::AngleRef(value),
            ParameterIndex::SpeedLimit => Self::SpeedLimit(value),
            ParameterIndex::CurrentLimit => Self::CurrentLimit(value),
            ParameterIndex::Angle => Self::Angle(value),
            ParameterIndex::Iq => Self::Iq(value),
            ParameterIndex::Speed => Self::Speed(value),
            ParameterIndex::AngleKp => Self::AngleKp(value),
            ParameterIndex::SpeedKp => Self::SpeedKp(value),
            ParameterIndex::SpeedKi => Self::SpeedKi(value),

            ParameterIndex::Spring => Self::Spring(value),
            ParameterIndex::Damping => Self::Damping(value),
            ParameterIndex::VqRef => Self::VqRef(value),
        })
    }
}

impl From<ParameterValue> for [u8; 8] {
    fn from(val: ParameterValue) -> [u8; 8] {
        let mut data = [0x00u8; 8];
        let address = match val {
            ParameterValue::RunMode(_) => ParameterIndex::RunMode,
            ParameterValue::IqRef(_) => ParameterIndex::IqRef,
            ParameterValue::SpeedRef(_) => ParameterIndex::SpeedRef,
            ParameterValue::TorqueLimit(_) => ParameterIndex::TorqueLimit,
            ParameterValue::CurrentKp(_) => ParameterIndex::CurrentKp,
            ParameterValue::CurrentKi(_) => ParameterIndex::CurrentKi,
            ParameterValue::CurrentFilter(_) => ParameterIndex::CurrentFilter,
            ParameterValue::AngleRef(_) => ParameterIndex::AngleRef,
            ParameterValue::SpeedLimit(_) => ParameterIndex::SpeedLimit,
            ParameterValue::CurrentLimit(_) => ParameterIndex::CurrentLimit,
            ParameterValue::Angle(_) => ParameterIndex::Angle,
            ParameterValue::Iq(_) => ParameterIndex::Iq,
            ParameterValue::Speed(_) => ParameterIndex::Speed,
            ParameterValue::AngleKp(_) => ParameterIndex::AngleKp,
            ParameterValue::SpeedKp(_) => ParameterIndex::SpeedKp,
            ParameterValue::SpeedKi(_) => ParameterIndex::SpeedKi,

            ParameterValue::Spring(_) => ParameterIndex::Spring,
            ParameterValue::Damping(_) => ParameterIndex::Damping,
            ParameterValue::VqRef(_) => ParameterIndex::VqRef,
        };
        data[0..2].copy_from_slice(&(address as u16).to_le_bytes());

        match val {
            ParameterValue::RunMode(x) => data[4] = x as u8,
            ParameterValue::IqRef(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::SpeedRef(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::TorqueLimit(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::CurrentKp(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::CurrentKi(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::CurrentFilter(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::AngleRef(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::SpeedLimit(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::CurrentLimit(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::Angle(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::Iq(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::Speed(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::AngleKp(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::SpeedKp(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::SpeedKi(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),

            ParameterValue::Spring(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::Damping(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
            ParameterValue::VqRef(x) => data[4..8].copy_from_slice(&x.to_le_bytes()),
        };
        data
    }
}

impl ResponseMessage {
    pub fn new(motor_can_id: u8, host_can_id: u8, body: ResponseBody) -> Self {
        Self {
            motor_can_id,
            host_can_id,
            body,
        }
    }
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum FeedbackType {
    Status = 0x00,
    Current = 0x01,
}

impl TryFrom<u8> for FeedbackType {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(Self::Status),
            0x01 => Ok(Self::Current),
            _ => Err(()),
        }
    }
}

#[derive(Copy, Clone)]
pub struct MotorStatus {
    pub angle: f32,
    pub velocity: f32,
    pub torque: f32,
    pub temperature: f32,
}

impl From<[u8; 8]> for MotorStatus {
    fn from(val: [u8; 8]) -> MotorStatus {
        let mut u16_buffer = [0x00u8; 2];
        u16_buffer.copy_from_slice(&val[0..2]);
        let angle = u16_to_f32(u16::from_le_bytes(u16_buffer), ANGLE_MIN, ANGLE_MAX);
        u16_buffer.copy_from_slice(&val[2..4]);
        let velocity = u16_to_f32(u16::from_le_bytes(u16_buffer), VELOCITY_MIN, VELOCITY_MAX);
        u16_buffer.copy_from_slice(&val[4..6]);
        let torque = u16_to_f32(u16::from_le_bytes(u16_buffer), TORQUE_MIN, TORQUE_MAX);
        u16_buffer.copy_from_slice(&val[6..8]);
        let temperature = u16_to_f32(
            u16::from_le_bytes(u16_buffer),
            TEMPERATURE_MIN,
            TEMPERATURE_MAX,
        );
        MotorStatus {
            angle,
            velocity,
            torque,
            temperature,
        }
    }
}

impl From<MotorStatus> for [u8; 8] {
    fn from(val: MotorStatus) -> [u8; 8] {
        let mut data = [0x00u8; 8];
        let angle = f32_to_u16(val.angle, ANGLE_MIN, ANGLE_MAX);
        data[0..2].copy_from_slice(&angle.to_le_bytes());
        let velocity = f32_to_u16(val.velocity, VELOCITY_MIN, VELOCITY_MAX);
        data[2..4].copy_from_slice(&velocity.to_le_bytes());
        let torque = f32_to_u16(val.torque, TORQUE_MIN, TORQUE_MAX);
        data[4..6].copy_from_slice(&torque.to_le_bytes());
        let temperature = f32_to_u16(val.temperature, TEMPERATURE_MIN, TEMPERATURE_MAX);
        data[6..8].copy_from_slice(&temperature.to_le_bytes());
        data
    }
}

#[derive(Copy, Clone)]
pub struct MotorCurrent {
    pub i_q: f32,
    pub i_d: f32,
}

impl From<[u8; 8]> for MotorCurrent {
    fn from(val: [u8; 8]) -> MotorCurrent {
        let mut f32_buffer = [0x00u8; 4];
        f32_buffer.copy_from_slice(&val[0..4]);
        let i_q = f32::from_le_bytes(f32_buffer);
        f32_buffer.copy_from_slice(&val[4..8]);
        let i_d = f32::from_le_bytes(f32_buffer);
        MotorCurrent { i_q, i_d }
    }
}

impl From<MotorCurrent> for [u8; 8] {
    fn from(val: MotorCurrent) -> [u8; 8] {
        let mut data = [0x00u8; 8];
        data[0..4].copy_from_slice(&val.i_q.to_le_bytes());
        data[4..8].copy_from_slice(&val.i_d.to_le_bytes());
        data
    }
}

const ANGLE_MIN: f32 = -1000.0;
const ANGLE_MAX: f32 = 1000.0;
const VELOCITY_MIN: f32 = -500.0;
const VELOCITY_MAX: f32 = 500.0;
const TORQUE_MIN: f32 = -100.0;
const TORQUE_MAX: f32 = 100.0;
const TEMPERATURE_MIN: f32 = -100.0;
const TEMPERATURE_MAX: f32 = 100.0;

impl TryFrom<CanMessage> for ResponseMessage {
    type Error = ();
    fn try_from(val: CanMessage) -> Result<Self, Self::Error> {
        let response_type = val.id >> 24;
        let body = match response_type {
            0x02 => {
                let motor_status = MotorStatus::from(val.data);
                ResponseBody::MotorStatus(motor_status)
            }
            0x11 => {
                let pv = ParameterValue::try_from(val.data)?;
                ResponseBody::ParameterValue(pv)
            }
            0x17 => {
                let motor_current = MotorCurrent::from(val.data);
                ResponseBody::MotorCurrent(motor_current)
            }
            _ => return Err(()),
        };
        Ok(ResponseMessage {
            motor_can_id: (val.id >> 8 & 0xFF) as u8,
            host_can_id: (val.id & 0xFF) as u8,
            body,
        })
    }
}

impl TryFrom<ResponseMessage> for CanMessage {
    type Error = ();
    fn try_from(val: ResponseMessage) -> Result<Self, Self::Error> {
        let response_type = match val.body {
            ResponseBody::MotorStatus(_) => 0x02,
            ResponseBody::MotorCurrent(_) => 0x17,
            ResponseBody::ParameterValue(_) => 0x11,
        };
        let id = (response_type << 24) | (val.motor_can_id as u32) << 8 | (val.host_can_id as u32);
        let data = match val.body {
            ResponseBody::MotorStatus(x) => x.into(),
            ResponseBody::MotorCurrent(x) => x.into(),
            ResponseBody::ParameterValue(x) => x.into(),
        };
        Ok(CanMessage {
            id,
            data,
            length: 8,
        })
    }
}

fn f32_to_u16(x: f32, x_min: f32, x_max: f32) -> u16 {
    let x = x.min(x_max).max(x_min);
    let span = x_max - x_min;
    ((x - x_min) / span * (u16::MAX as f32)) as u16
}

fn u16_to_f32(x: u16, x_min: f32, x_max: f32) -> f32 {
    let span = x_max - x_min;
    x as f32 / (u16::MAX as f32) * span + x_min
}

pub enum Command {
    Stop,
    Enable,
    SetCanId(u8),
    GetParameter(ParameterIndex),
    SetParameter(ParameterValue),
    RequestStatus(u8),
    SetFeedbackInterval(u8),
    SetFeedbackType(FeedbackType),
    Recalibrate,
    SaveConfig,
}

pub enum CommandError {
    IdFormat,
    WrongCommand,
    WrongRegister,
    CanDataLength,
}

pub struct CommandMessage {
    pub motor_can_id: u8,
    pub host_can_id: u8,
    pub command: Command,
}

impl TryFrom<CanMessage> for CommandMessage {
    type Error = CommandError;

    fn try_from(message: CanMessage) -> Result<Self, Self::Error> {
        let raw_id = message.id.to_le_bytes();
        let motor_can_id = raw_id[0];
        let host_can_id = raw_id[1];
        let command_content = raw_id[2];
        let command_id = raw_id[3];
        let data = message.data;
        if message.length != 8 {
            return Err(CommandError::CanDataLength);
        }

        let command = match command_id {
            0x03 => Command::Enable,
            0x04 => Command::Stop,
            0x07 => Command::SetCanId(command_content),
            0x11 => {
                let mut u16_buffer = [0x00u8; 2];
                u16_buffer.copy_from_slice(&data[0..2]);
                let p = ParameterIndex::try_from(u16::from_le_bytes(u16_buffer))
                    .map_err(|_| CommandError::WrongRegister)?;
                Command::GetParameter(p)
            }
            0x12 => {
                let pv = ParameterValue::try_from(data).map_err(|_| CommandError::WrongRegister)?;
                Command::SetParameter(pv)
            }
            0x15 => Command::RequestStatus(host_can_id),
            0x16 => Command::SetFeedbackInterval(command_content),
            0x17 => Command::SetFeedbackType(
                command_content
                    .try_into()
                    .map_err(|_| CommandError::WrongCommand)?,
            ),
            0x18 => Command::Recalibrate,
            0x19 => Command::SaveConfig,

            _ => return Err(CommandError::WrongCommand),
        };
        Ok(CommandMessage {
            motor_can_id,
            host_can_id,
            command,
        })
    }
}

impl TryFrom<CommandMessage> for CanMessage {
    type Error = CommandError;
    fn try_from(val: CommandMessage) -> Result<Self, Self::Error> {
        let command_id: u8;
        let mut command_content: u8 = 0;
        let mut data = [0x00u8; 8];
        match val.command {
            Command::Enable => {
                command_id = 0x03;
            }
            Command::Stop => {
                command_id = 0x04;
            }
            Command::GetParameter(p) => {
                command_id = 0x11;
                data[0..2].copy_from_slice(&(p as u16).to_le_bytes());
            }
            Command::SetParameter(pv) => {
                command_id = 0x12;
                let pv_data: [u8; 8] = pv.into();
                data.copy_from_slice(&pv_data);
            }
            Command::SetCanId(x) => {
                command_id = 0x07;
                command_content = x;
            }
            Command::RequestStatus(x) => {
                command_id = 0x15;
                command_content = x;
            }
            Command::SetFeedbackInterval(x) => {
                command_id = 0x16;
                command_content = x;
            }
            Command::SetFeedbackType(x) => {
                command_id = 0x17;
                command_content = x as u8;
            }
            Command::Recalibrate => {
                command_id = 0x18;
            }
            Command::SaveConfig => {
                command_id = 0x19;
            }
        }

        let id = (command_id as u32) << 24
            | ((command_content as u32) << 16)
            | (val.host_can_id as u32)
            | (val.motor_can_id as u32);

        Ok(CanMessage {
            id,
            data,
            length: 8,
        })
    }
}
