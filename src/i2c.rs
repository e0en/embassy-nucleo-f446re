pub fn blocking_read(
    i2c: &mut embassy_stm32::i2c::I2c<'_, embassy_stm32::mode::Blocking>,
    address: u8,
    register: u8,
    result: &mut [u8],
) -> Result<(), embassy_stm32::i2c::Error> {
    i2c.blocking_write_read(address, &[register], result)?;
    Ok(())
}

pub fn blocking_write_byte(
    i2c: &mut embassy_stm32::i2c::I2c<'_, embassy_stm32::mode::Blocking>,
    address: u8,
    register: u8,
    content: u8,
) -> Result<(), embassy_stm32::i2c::Error> {
    let buffer = [register, content];
    i2c.blocking_write(address, &buffer)?;
    Ok(())
}
