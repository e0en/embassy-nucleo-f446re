use embassy_stm32::can::{self, enums::FrameCreateError};

pub struct StatusMessage {
    pub motor_can_id: u8,
    pub host_can_id: u8,
    pub raw_position: u16,
    pub raw_speed: u16,
    pub raw_torque: u16,
    pub raw_temperature: u16,
}

pub enum ControlMode {
    Position,
    Speed,
    Current,
}

pub enum Command {
    Stop,
    Enable,
    SetCanId(u8),
    SetMode(ControlMode),

    SetSpeedLimit(f32),
    SetCurrentLimit(f32),
    SetTorqueLimit(f32),

    SetPosition(f32),
    SetSpeed(f32),
    SetTorque(f32),
    RequestStatus,
}

pub enum CommandError {
    IdFormat,
    WrongCommand,
    WrongRegister,
}

pub struct CommandMessage {
    pub motor_can_id: u8,
    pub command: Command,
}

impl TryInto<can::Frame> for StatusMessage {
    type Error = FrameCreateError;
    fn try_into(self) -> Result<can::Frame, Self::Error> {
        let raw_id = (self.motor_can_id as u32) << 8 | (self.host_can_id as u32);
        let mut raw_data = [0x00u8; 8];
        raw_data[0..2].copy_from_slice(&self.raw_position.to_le_bytes());
        raw_data[2..4].copy_from_slice(&self.raw_speed.to_le_bytes());
        raw_data[4..6].copy_from_slice(&self.raw_torque.to_le_bytes());
        raw_data[6..8].copy_from_slice(&self.raw_temperature.to_le_bytes());
        can::Frame::new_extended(raw_id, &raw_data)
    }
}

impl TryFrom<can::Frame> for CommandMessage {
    type Error = CommandError;

    fn try_from(message: can::Frame) -> Result<Self, Self::Error> {
        let raw_id = match message.id() {
            embedded_can::Id::Extended(e) => e.as_raw().to_le_bytes(),
            _ => return Err(CommandError::IdFormat),
        };
        let motor_can_id = raw_id[0];
        let command_content = (raw_id[2] as u16) << 8 | raw_id[1] as u16;
        let command_id = raw_id[3];
        let data = message.data();
        let value: f32 = 0.0;
        value.to_le_bytes().copy_from_slice(&data[4..]);

        let command = match command_id {
            0x03 => Command::Enable,
            0x04 => Command::Stop,
            0x07 => Command::SetCanId(command_content as u8),
            0x12 => {
                let address = (data[1] as u16) << 8 | data[0] as u16;
                match address {
                    0x7005 => match data[0] {
                        0x01 => Command::SetMode(ControlMode::Position),
                        0x02 => Command::SetMode(ControlMode::Speed),
                        0x03 => Command::SetMode(ControlMode::Current),
                        _ => return Err(CommandError::WrongRegister),
                    },
                    0x7006 => Command::SetTorque(value),
                    0x700A => Command::SetSpeed(value),
                    0x700B => Command::SetTorqueLimit(value),
                    0x7016 => Command::SetPosition(value),
                    0x7017 => Command::SetSpeedLimit(value),
                    0x7018 => Command::SetCurrentLimit(value),
                    _ => return Err(CommandError::WrongRegister),
                }
            }
            0x15 => Command::RequestStatus,
            _ => return Err(CommandError::WrongCommand),
        };
        Ok(CommandMessage {
            motor_can_id,
            command,
        })
    }
}
