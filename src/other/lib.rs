
#![no_std]

use embedded_hal::i2c::{I2c, SevenBitAddress};

mod vl53l8cx_reg;

pub struct Vl53l8cx<B: vl53l8cx_reg::BusOperation> {
    bus: B,
}

impl<P> Vl53l8cx<vl53l8cx_reg::Vl53l8cxI2C<P>> {
    where
    P: I2c,
{
    pub fn new_i2c(i2c: P, address: I2CAddress) -> Result<Self, Error<P::Error>> {
        let bus = lps22df_reg::Lps22dfI2C::new(i2c, address as SevenBitAddress);
        let mut instance = Self { bus };
        let who = instance.who_am_i_get()?;
        if who != 0xB4 {
            return Err(Error::WhoAmIError(who));
        }
        instance.ctrl_reg2_set_boot()?;
        while instance.int_source_get_boot_on()? != 0 {}
        instance.ctrl_reg2_set_swreset()?;
        while instance.ctrl_reg2_get_swreset()? != 0 {}

        Ok(instance)
    }
}
