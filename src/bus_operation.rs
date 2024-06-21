use consts::*;
use crate::{consts, Vl53l8cx, Error, SevenBitAddress, I2c, Pin, Output, PushPull, SpiDevice, SysDelay, Operation};

pub trait BusOperation {
    type Error;
    #[allow(dead_code)]
    fn read(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error>; 
    fn write(&mut self, wbuf: &[u8]) -> Result<(), Self::Error>;
    fn write_read(&mut self, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error>;
}

pub struct Vl53l8cxI2C<P> {
    i2c: P,
    address: SevenBitAddress,
}

impl<P: I2c> Vl53l8cxI2C<P> {
    pub fn new(i2c: P, address: SevenBitAddress) -> Self {
        Self { i2c, address }
    }
}

impl<P: I2c> BusOperation for Vl53l8cxI2C<P> {
    type Error = P::Error;

    #[inline]
    fn read(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.read(self.address, rbuf)?;
        
        Ok(())
    }
    
    #[inline]
    fn write(&mut self, wbuf: &[u8]) -> Result<(), Self::Error> {
        self.i2c.write(self.address, wbuf)?;

        Ok(())
    }
    
    #[inline]
    fn write_read(&mut self, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, wbuf, rbuf)?;
        
        Ok(())
    }
}

pub struct Vl53l8cxSPI<P> {
    spi: P,
    #[allow(dead_code)]
    cs_pin: Pin<'B', 6, Output<PushPull>>
}

// new for spi
impl<P: SpiDevice> Vl53l8cxSPI<P> {
    #[allow(dead_code)]
    pub fn new(spi: P, cs_pin: Pin<'B', 6, Output<PushPull>>) -> Self {
        Self { spi, cs_pin }
    }
}

// read, write, write_read for spi
impl<P: SpiDevice> BusOperation for Vl53l8cxSPI<P> {

    type Error = P::Error;

    #[inline]
    fn read(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.spi.transaction(&mut [Operation::Read(rbuf)])?;

        Ok(())
    }

    #[inline]
    fn write(&mut self, wbuf: &[u8]) -> Result<(), Self::Error> {
        self.spi.transaction(&mut [Operation::Write(wbuf)])?;

        Ok(())
    }

    #[inline]
    fn write_read(&mut self, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.spi
            .transaction(&mut [Operation::Write(wbuf), Operation::Read(rbuf)])?;
        Ok(())
    }
}

impl<P> Vl53l8cx<Vl53l8cxI2C<P>> 
    where
    P: I2c,
{
    pub fn new_i2c(i2c: P, address: SevenBitAddress, lpn_pin: Pin<'B', 0, Output<PushPull>>, i2c_rst_pin: i8, delay: SysDelay) -> Result<Self, Error<P::Error>> {
        let streamcount: u8 = 0;
        let data_read_size: u32 = 0;
        let is_auto_stop_enabled: u8 = 0;
        let bus: Vl53l8cxI2C<P> = Vl53l8cxI2C::new(i2c, address);
        let temp_buffer: [u8; VL53L8CX_TEMPORARY_BUFFER_SIZE] = [0; VL53L8CX_TEMPORARY_BUFFER_SIZE];
        let offset_data: [u8; VL53L8CX_OFFSET_BUFFER_SIZE] = [0; VL53L8CX_OFFSET_BUFFER_SIZE];
        let xtalk_data: [u8; VL53L8CX_XTALK_BUFFER_SIZE] = [0; VL53L8CX_XTALK_BUFFER_SIZE];
        let instance: Vl53l8cx<Vl53l8cxI2C<P>> = Self { 
            temp_buffer,
            offset_data,
            xtalk_data,
            streamcount,
            data_read_size,
            is_auto_stop_enabled,
            lpn_pin,
            i2c_rst_pin,
            bus,
            delay
        };
        Ok(instance)
    }

    pub fn set_i2c_address(&mut self, i2c_address: SevenBitAddress) -> Result<(), Error<P::Error>> {
        self.write_to_register(0x7fff, 0x00)?;
        self.write_to_register(0x4, i2c_address)?;
        self.bus.address = i2c_address;
        self.write_to_register(0x7fff, 0x02)?;
        
        Ok(())
    }

    pub fn init_sensor(&mut self, address: u8) -> Result<(), Error<P::Error>>{
        self.off()?;
        self.on()?;
        if address != self.bus.address {
            self.set_i2c_address(address)?;
        }
        self.is_alive()?;
        self.init()?;
        Ok(())
    }

}

impl<P> Vl53l8cx<Vl53l8cxSPI<P>> 
    where P: SpiDevice,
{
    #[allow(dead_code)]
    pub fn new_spi(spi: P, cs_pin: Pin<'B', 6, Output<PushPull>>, lpn_pin: Pin<'B', 0, Output<PushPull>>, i2c_rst_pin: i8, delay: SysDelay) -> Result<Self, Error<P::Error>> {
        let streamcount: u8 = 0;
        let data_read_size: u32 = 0;
        let is_auto_stop_enabled: u8 = 0;
        let bus: Vl53l8cxSPI<P> = Vl53l8cxSPI::new(spi, cs_pin);
        let temp_buffer: [u8; VL53L8CX_TEMPORARY_BUFFER_SIZE] = [0; VL53L8CX_TEMPORARY_BUFFER_SIZE];
        let offset_data: [u8; VL53L8CX_OFFSET_BUFFER_SIZE] = [0; VL53L8CX_OFFSET_BUFFER_SIZE];
        let xtalk_data: [u8; VL53L8CX_XTALK_BUFFER_SIZE] = [0; VL53L8CX_XTALK_BUFFER_SIZE];
        let instance: Vl53l8cx<Vl53l8cxSPI<P>> = Self { 
            temp_buffer,
            offset_data,
            xtalk_data,
            streamcount,
            data_read_size,
            is_auto_stop_enabled,
            lpn_pin,
            i2c_rst_pin,
            bus,
            delay
        };
        
        Ok(instance)
    }

    #[allow(dead_code)]
    pub fn init_sensor(&mut self) -> Result<(), Error<P::Error>>{
        self.off()?;
        self.on()?;
        self.is_alive()?;
        self.init()?;
        Ok(())
    }
    
}

