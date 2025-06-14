use embassy_stm32::{
    i2c::{Error, I2c, Master},
    mode::Blocking,
};

pub fn blocking_read(
    i2c: &mut I2c<'_, Blocking, Master>,
    address: u8,
    register: u8,
    result: &mut [u8],
) -> Result<(), Error> {
    i2c.blocking_write_read(address, &[register], result)?;
    Ok(())
}

pub fn blocking_write_byte(
    i2c: &mut I2c<'_, Blocking, Master>,
    address: u8,
    register: u8,
    content: u8,
) -> Result<(), Error> {
    let buffer = [register, content];
    i2c.blocking_write(address, &buffer)?;
    Ok(())
}
