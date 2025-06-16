use embassy_stm32::{
    i2c::{Error, I2c, Master},
    mode::Async,
};
use embassy_time::{Duration, TimeoutError, WithTimeout};

pub async fn read(
    i2c: &mut I2c<'_, Async, Master>,
    address: u8,
    register: u8,
    result: &mut [u8],
) -> Result<(), Error> {
    match i2c
        .write_read(address, &[register], result)
        .with_timeout(Duration::from_secs(1))
        .await
    {
        Ok(Ok(_)) => Ok(()),
        Ok(Err(e)) => Err(e),
        Err(TimeoutError) => Err(Error::Timeout),
    }
}

pub async fn write_byte(
    i2c: &mut I2c<'_, Async, Master>,
    address: u8,
    register: u8,
    content: u8,
) -> Result<(), Error> {
    let buffer = [register, content];
    match i2c
        .write(address, &buffer)
        .with_timeout(Duration::from_secs(1))
        .await
    {
        Ok(Ok(_)) => Ok(()),
        Ok(Err(e)) => Err(e),
        Err(TimeoutError) => Err(Error::Timeout),
    }
}
