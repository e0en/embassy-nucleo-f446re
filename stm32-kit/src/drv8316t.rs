use embassy_stm32::gpio;

pub struct Drv8316T<'d> {
    drvoff_pin: gpio::Output<'d>,
}

impl<'d> Drv8316T<'d> {
    pub fn new(drvoff_pin: gpio::Output<'d>) -> Self {
        Drv8316T { drvoff_pin }
    }

    pub fn turn_on(&mut self) {
        self.drvoff_pin.set_low();
    }

    pub fn turn_off(&mut self) {
        self.drvoff_pin.set_high();
    }
}
