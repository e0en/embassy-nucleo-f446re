use embassy_stm32::can::{self, enums::FrameCreateError};
use foc::controller::RunMode;

#[derive(Copy, Clone)]
pub struct StatusMessage {
    pub motor_can_id: u8,
    pub host_can_id: u8,
    pub motor_status: MotorStatus,
}

#[derive(Copy, Clone)]
pub struct MotorStatus {
    pub raw_angle: u16,
    pub raw_velocity: u16,
    pub raw_torque: u16,
    pub raw_temperature: u16,
}

pub enum Command {
    Stop,
    Enable,
    SetCanId(u8),

    SetSpeedLimit(f32),
    SetCurrentLimit(f32),
    SetTorqueLimit(f32),

    SetAngle(f32),
    SetVelocity(f32),
    SetTorque(f32),

    SetSpring(f32),
    SetDamping(f32),
    SetVelocityKp(f32),
    SetVelocityKi(f32),
    SetVelocityGain(f32),
    SetAngleKp(f32),

    SetRunMode(RunMode),
    RequestStatus(u8),
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
        raw_data[0..2].copy_from_slice(&self.motor_status.raw_angle.to_le_bytes());
        raw_data[2..4].copy_from_slice(&self.motor_status.raw_velocity.to_le_bytes());
        raw_data[4..6].copy_from_slice(&self.motor_status.raw_torque.to_le_bytes());
        raw_data[6..8].copy_from_slice(&self.motor_status.raw_temperature.to_le_bytes());
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
        let mut value_buffer = [0x00u8; 4];
        value_buffer.copy_from_slice(&data[4..8]);
        let value = f32::from_le_bytes(value_buffer);

        let command = match command_id {
            0x03 => Command::Enable,
            0x04 => Command::Stop,
            0x07 => Command::SetCanId(command_content as u8),
            0x12 => {
                let address = (data[1] as u16) << 8 | data[0] as u16;
                match address {
                    0x7005 => match data[2] {
                        0x00 => Command::SetRunMode(RunMode::Impedance),
                        0x01 => Command::SetRunMode(RunMode::Angle),
                        0x02 => Command::SetRunMode(RunMode::Velocity),
                        0x03 => Command::SetRunMode(RunMode::Torque),
                        _ => return Err(CommandError::WrongCommand),
                    },
                    0x7006 => Command::SetTorque(value),
                    0x700A => Command::SetVelocity(value),
                    0x700B => Command::SetTorqueLimit(value),
                    0x7016 => Command::SetAngle(value),
                    0x2014 => Command::SetVelocityKp(value),
                    0x2015 => Command::SetVelocityKi(value),
                    0x2016 => Command::SetAngleKp(value),
                    0x2017 => Command::SetVelocityGain(value),
                    0x2018 => Command::SetSpeedLimit(value),
                    0x2019 => Command::SetCurrentLimit(value),
                    0x201A => Command::SetSpring(value),
                    0x201B => Command::SetDamping(value),
                    _ => return Err(CommandError::WrongRegister),
                }
            }
            0x15 => {
                let host_id = (command_content & 0xFF) as u8;
                Command::RequestStatus(host_id)
            }
            _ => return Err(CommandError::WrongCommand),
        };
        Ok(CommandMessage {
            motor_can_id,
            command,
        })
    }
}
