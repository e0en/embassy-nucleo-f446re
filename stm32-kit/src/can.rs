use can_message::message::{CanMessage, CommandError, CommandMessage, StatusMessage};
use embassy_stm32::can::{self, enums::FrameCreateError};

pub fn convert_status_message(m: StatusMessage) -> Result<can::Frame, FrameCreateError> {
    let cm: CanMessage = m.into();
    can::Frame::new_extended(cm.id, &cm.data[0..(cm.length as usize)])
}

pub fn parse_command_frame(message: can::Frame) -> Result<CommandMessage, CommandError> {
    match message.id() {
        embedded_can::Id::Extended(e) => {
            let mut data: [u8; 8] = [0x00u8; 8];
            data.copy_from_slice(message.data());
            let cm = CanMessage {
                id: e.as_raw(),
                data,
                length: message.header().len(),
            };

            CommandMessage::try_from(cm)
        }
        _ => Err(CommandError::IdFormat),
    }
}
