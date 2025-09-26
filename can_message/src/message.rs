#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum RunMode {
    Impedance,
    Angle,
    Velocity,
    Torque,
}

pub struct CanMessage {
    pub id: u32,
    pub data: [u8; 8],
    pub length: u8,
}

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

impl From<CanMessage> for StatusMessage {
    fn from(val: CanMessage) -> Self {
        let mut u16_buffer = [0x00u8; 2];
        u16_buffer.copy_from_slice(&val.data[0..2]);
        let raw_angle = u16::from_le_bytes(u16_buffer);
        u16_buffer.copy_from_slice(&val.data[2..4]);
        let raw_velocity = u16::from_le_bytes(u16_buffer);
        u16_buffer.copy_from_slice(&val.data[4..6]);
        let raw_torque = u16::from_le_bytes(u16_buffer);
        u16_buffer.copy_from_slice(&val.data[6..8]);
        let raw_temperature = u16::from_le_bytes(u16_buffer);
        let motor_status = MotorStatus {
            raw_angle,
            raw_velocity,
            raw_torque,
            raw_temperature,
        };

        StatusMessage {
            motor_can_id: (val.id >> 8 & 0xFF) as u8,
            host_can_id: (val.id & 0xFF) as u8,
            motor_status,
        }
    }
}

impl From<StatusMessage> for CanMessage {
    fn from(val: StatusMessage) -> Self {
        let id = (val.motor_can_id as u32) << 8 | (val.host_can_id as u32);
        let mut data = [0x00u8; 8];
        data[0..2].copy_from_slice(&val.motor_status.raw_angle.to_le_bytes());
        data[2..4].copy_from_slice(&val.motor_status.raw_velocity.to_le_bytes());
        data[4..6].copy_from_slice(&val.motor_status.raw_torque.to_le_bytes());
        data[6..8].copy_from_slice(&val.motor_status.raw_temperature.to_le_bytes());
        CanMessage {
            id,
            data,
            length: 8,
        }
    }
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

    SetCurrentKp(f32),
    SetCurrentKi(f32),

    SetRunMode(RunMode),
    RequestStatus(u8),

    SetMonitorInterval(u8),
}

pub enum CommandError {
    IdFormat,
    WrongCommand,
    WrongRegister,
    CanDataLength,
}

pub struct CommandMessage {
    pub motor_can_id: u8,
    pub command: Command,
}

impl TryFrom<CanMessage> for CommandMessage {
    type Error = CommandError;

    fn try_from(message: CanMessage) -> Result<Self, Self::Error> {
        let raw_id = message.id.to_le_bytes();
        let motor_can_id = raw_id[0];
        let command_content = (raw_id[2] as u16) << 8 | raw_id[1] as u16;
        let command_id = raw_id[3];
        let data = message.data;
        if message.length != 8 {
            return Err(CommandError::CanDataLength);
        }
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
                    0x7010 => Command::SetCurrentKp(value),
                    0x7011 => Command::SetCurrentKi(value),
                    0x7016 => Command::SetAngle(value),
                    0x7017 => Command::SetSpeedLimit(value),
                    0x7018 => Command::SetCurrentLimit(value),
                    0x701E => Command::SetAngleKp(value),
                    0x701F => Command::SetVelocityKp(value),
                    0x7020 => Command::SetVelocityKi(value),
                    0x2017 => Command::SetVelocityGain(value),
                    0x201A => Command::SetSpring(value),
                    0x201B => Command::SetDamping(value),
                    _ => return Err(CommandError::WrongRegister),
                }
            }
            0x15 => {
                let host_id = (command_content & 0xFF) as u8;
                Command::RequestStatus(host_id)
            }
            0x16 => Command::SetMonitorInterval(command_content as u8),

            _ => return Err(CommandError::WrongCommand),
        };
        Ok(CommandMessage {
            motor_can_id,
            command,
        })
    }
}

impl TryFrom<CommandMessage> for CanMessage {
    type Error = CommandError;
    fn try_from(val: CommandMessage) -> Result<Self, Self::Error> {
        let command_id: u8;
        let mut command_content: u16 = 0;
        let mut data = [0x00u8; 8];
        match val.command {
            Command::Enable => {
                command_id = 0x03;
            }
            Command::Stop => {
                command_id = 0x04;
            }
            Command::SetAngle(x) => {
                command_id = 0x12;
                data[0..2].copy_from_slice(&0x7016u16.to_le_bytes());
                data[4..8].copy_from_slice(&x.to_le_bytes());
            }
            Command::SetVelocity(x) => {
                command_id = 0x12;
                data[0..2].copy_from_slice(&0x700Au16.to_le_bytes());
                data[4..8].copy_from_slice(&x.to_le_bytes());
            }
            Command::SetTorque(x) => {
                command_id = 0x12;
                data[0..2].copy_from_slice(&0x7006u16.to_le_bytes());
                data[4..8].copy_from_slice(&x.to_le_bytes());
            }
            Command::SetTorqueLimit(x) => {
                command_id = 0x12;
                data[0..2].copy_from_slice(&0x700Bu16.to_le_bytes());
                data[4..8].copy_from_slice(&x.to_le_bytes());
            }
            Command::SetCurrentKp(x) => {
                command_id = 0x12;
                data[0..2].copy_from_slice(&0x7010u16.to_le_bytes());
                data[4..8].copy_from_slice(&x.to_le_bytes());
            }
            Command::SetCurrentKi(x) => {
                command_id = 0x12;
                data[0..2].copy_from_slice(&0x7011u16.to_le_bytes());
                data[4..8].copy_from_slice(&x.to_le_bytes());
            }
            Command::SetSpeedLimit(x) => {
                command_id = 0x12;
                data[0..2].copy_from_slice(&0x7017u16.to_le_bytes());
                data[4..8].copy_from_slice(&x.to_le_bytes());
            }
            Command::SetCurrentLimit(x) => {
                command_id = 0x12;
                data[0..2].copy_from_slice(&0x7018u16.to_le_bytes());
                data[4..8].copy_from_slice(&x.to_le_bytes());
            }
            Command::SetAngleKp(x) => {
                command_id = 0x12;
                data[0..2].copy_from_slice(&0x701Eu16.to_le_bytes());
                data[4..8].copy_from_slice(&x.to_le_bytes());
            }
            Command::SetVelocityKp(x) => {
                command_id = 0x12;
                data[0..2].copy_from_slice(&0x701Fu16.to_le_bytes());
                data[4..8].copy_from_slice(&x.to_le_bytes());
            }
            Command::SetVelocityKi(x) => {
                command_id = 0x12;
                data[0..2].copy_from_slice(&0x7020u16.to_le_bytes());
                data[4..8].copy_from_slice(&x.to_le_bytes());
            }
            Command::SetVelocityGain(x) => {
                command_id = 0x12;
                data[0..2].copy_from_slice(&0x2017u16.to_le_bytes());
                data[4..8].copy_from_slice(&x.to_le_bytes());
            }
            Command::SetSpring(x) => {
                command_id = 0x12;
                data[0..2].copy_from_slice(&0x201Au16.to_le_bytes());
                data[4..8].copy_from_slice(&x.to_le_bytes());
            }
            Command::SetDamping(x) => {
                command_id = 0x12;
                data[0..2].copy_from_slice(&0x201Au16.to_le_bytes());
                data[4..8].copy_from_slice(&x.to_le_bytes());
            }
            Command::SetRunMode(m) => {
                command_id = 0x12;
                data[0..2].copy_from_slice(&0x7005u16.to_le_bytes());
                data[2] = match m {
                    RunMode::Impedance => 0x00,
                    RunMode::Angle => 0x01,
                    RunMode::Velocity => 0x02,
                    RunMode::Torque => 0x03,
                };
            }
            Command::SetCanId(x) => {
                command_id = 0x07;
                command_content = x as u16;
            }
            Command::SetMonitorInterval(x) => {
                command_id = 0x16;
                command_content = x as u16;
            }
            Command::RequestStatus(x) => {
                command_id = 0x15;
                command_content = x as u16;
            }
        }

        let id =
            (command_id as u32) << 24 | ((command_content as u32) << 8) | (val.motor_can_id as u32);

        Ok(CanMessage {
            id,
            data,
            length: 8,
        })
    }
}
