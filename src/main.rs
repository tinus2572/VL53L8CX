#![no_main]
#![no_std]
#![allow(dead_code)]

use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::{SpiDevice, Operation};
use panic_halt as _; 
use cortex_m_rt::entry;
use core::convert::TryInto;
use crate::buffers::*;
use crate::consts::*;

mod buffers;
mod consts;

 
pub struct Vl53l8cxI2C<P> {
    i2c: P,
    address: SevenBitAddress,
}

pub struct Vl53l8cx<B: BusOperation> {
    temp_buffer: [u8;  VL53L8CX_NVM_DATA_SIZE as usize],
    bus: B,
}

pub trait BusOperation {
    type Error;
    fn read(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error>; 
    fn write(&mut self, wbuf: &[u8]) -> Result<(), Self::Error>;
    fn write_read(&mut self, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error>;
}

#[derive(Copy, Clone, Debug)]
pub enum Error<B> {
    Bus(B),
    Other,
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
    fn write_read(
        &mut self,
        wbuf: &[u8],
        rbuf: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, wbuf, rbuf)?;
        
        Ok(())
    }
}

pub struct Vl53l8cxSPI<P> {
    spi: P,
}

// new for spi
impl<P: SpiDevice> Vl53l8cxSPI<P> {
    pub fn new(spi: P) -> Self {
        Self { spi }
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
    fn write_read(
        &mut self,
        wbuf: &[u8],
        rbuf: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.spi
            .transaction(&mut [Operation::Write(wbuf), Operation::Read(rbuf)])?;
        Ok(())
    }
}


impl<B: BusOperation> Vl53l8cx<B> {
    #[inline]
    fn read_from_register(&mut self, reg: u16, rbuf: &mut [u8]) -> Result<(), Error<B::Error>> {
        let a: u8 = (reg >> 8).try_into().unwrap();
        let b: u8 = (reg & 0xFF).try_into().unwrap(); 
        self.bus
            .write_read(&[a, b], rbuf)
            .map_err(Error::Bus)?;

        Ok(())
    }
    
    #[inline]
    fn write_to_register(&mut self, reg: u16, val: u8) -> Result<(), Error<B::Error>> {
        let a: u8 = (reg >> 8).try_into().unwrap();
        let b: u8 = (reg & 0xFF).try_into().unwrap(); 
        self.bus
            .write(&[a, b, val])
            .map_err(Error::Bus)?;
       
        Ok(())
    }

    fn write_multi_to_register(&mut self, reg: u16, wbuf: &[u8]) -> Result<(), Error<B::Error>> {
        let a: u8 = (reg >> 8).try_into().unwrap();
        let b: u8 = (reg & 0xFF).try_into().unwrap(); 
        self.bus
            .write(&[a, b])
            .map_err(Error::Bus)?;
        self.bus
            .write(wbuf)
            .map_err(Error::Bus)?;

        Ok(())
    }
}

// TODO
impl<P> Vl53l8cx<Vl53l8cxI2C<P>> 
    where
    P: I2c,
{
    pub fn new_i2c(i2c: P, address: u8) -> Result<Self, Error<P::Error>> {
        let bus: Vl53l8cxI2C<P> = Vl53l8cxI2C::new(i2c, address as SevenBitAddress);
        let mut temp_buffer: [u8; VL53L8CX_NVM_DATA_SIZE as usize] = [0; VL53L8CX_NVM_DATA_SIZE as usize];
        let instance: Vl53l8cx<Vl53l8cxI2C<P>> = Self { bus, temp_buffer };
        Ok(instance)
    }
}

impl<P> Vl53l8cx<Vl53l8cxSPI<P>> 
    where P: SpiDevice,
{
    pub fn new_spi(spi: P) -> Result<Self, Error<P::Error>> {
        let bus: Vl53l8cxSPI<P> = Vl53l8cxSPI::new(spi);
        let temp_buffer: [u8; VL53L8CX_NVM_DATA_SIZE as usize] = [0; VL53L8CX_NVM_DATA_SIZE as usize];
        let instance: Vl53l8cx<Vl53l8cxSPI<P>> = Self { bus, temp_buffer };
        
        Ok(instance)
    }
}



impl<B: BusOperation> Vl53l8cx<B> {

    fn poll_for_answer(&mut self, size: u8, pos: u8, reg: u16, mask: u8, expected_val: u8) -> Result<(), Error<B::Error>> {
        Ok(())
    }

    fn poll_for_mcu_boot(&mut self) -> Result<(), Error<B::Error>> {
        Ok(())
    }

    fn send_offset_data(&mut self, resolution: u8) -> Result<(), Error<B::Error>> {
        Ok(())
    }   

    fn send_xtalk_data(&mut self, resolution: u8) -> Result<(), Error<B::Error>> {
        Ok(())
    }  
    
    fn dci_read_data(&mut self, data: &[u8], index: u16) -> Result<(), Error<B::Error>> {
        let mut cmd: [u8; 12] = [
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x0f,
            0x00, 0x02, 0x00, 0x08
        ];
        if data.len() + 12 > VL53L8CX_TEMPORARY_BUFFER_SIZE as usize{
            return Err(Error::Other);
        } else {
            cmd[0] = (index >> 8) as u8;
            cmd[1] = (index & 0xff) as u8;
            cmd[2] = ((data.len() & 0xff0) >> 4) as u8;
            cmd[3] = ((data.len() & 0xf) << 4) as u8;
            /* Request data reading from FW */
            self.write_multi_to_register(VL53L8CX_UI_CMD_END - 11, &cmd);
            self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xff, 0x03);

            /* Read new data sent (4 bytes header + data_size + 8 bytes footer) */
            self.read_from_register(VL53L8CX_UI_CMD_START, &mut self.temp_buffer);
            self.swap_buffer(&mut self.temp_buffer);

            /* Copy data from FW into input structure (-4 bytes to remove header) */
// TODO
            // for (i = 0 ; i < (int16_t)data_size; i++) {
            //     data[i] = p_dev->temp_buffer[i + 4];
            // }
        }
       
        Ok(())
    }   
    
    fn dci_write_data(&mut self, data: &[u8], index: u16) -> Result<(), Error<B::Error>> {
        Ok(())
    }   

    fn dci_replace_data(&mut self, data: &[u8], index: u16, new_data: &[u8], new_data_pos: u16) -> Result<(), Error<B::Error>> {
        self.dci_read_data(data, index)?;
// TODO
        // (void)memcpy(&(data[new_data_pos]), new_data, new_data_size);
        self.dci_write_data(data, index)?;
        
        
        Ok(())
    }   

    pub fn swap_buffer(&mut self, buf: &[u8]) -> Result<(), Error<B::Error>> {
// TODO
        // uint32_t i, tmp;
        // /* Example of possible implementation using <string.h> */
        // for (i = 0; i < size; i = i + 4) {
        //   tmp = (
        //           buffer[i] << 24)
        //         | (buffer[i + 1] << 16)
        //         | (buffer[i + 2] << 8)
        //         | (buffer[i + 3]);
        //   memcpy(&(buffer[i]), &tmp, 4);

        Ok(())
    }



    pub fn init(&mut self) -> Result<(), Error<B::Error>> {
        let mut tmp: [u8; 1] = [0];    
        let mut pipe_ctrl: [u8; 4] = [VL53L8CX_NB_TARGET_PER_ZONE as u8, 0x00, 0x01, 0x00];
        let mut single_range: [u8; 1] = [0x01];

        self.write_to_register(0x7fff, 0x00)?;
        self.write_to_register(0x0009, 0x04)?;
        self.write_to_register(0x000F, 0x40)?;
        self.write_to_register(0x000A, 0x03)?;
        self.read_from_register(0x7FFF, &mut tmp)?;
        self.write_to_register(0x000C, 0x01)?;

        self.write_to_register(0x0101, 0x00)?;
        self.write_to_register(0x0102, 0x00)?;
        self.write_to_register(0x010A, 0x01)?;
        self.write_to_register(0x4002, 0x01)?;
        self.write_to_register(0x4002, 0x00)?;
        self.write_to_register(0x010A, 0x03)?;
        self.write_to_register(0x0103, 0x01)?;
        self.write_to_register(0x000C, 0x00)?;
        self.write_to_register(0x000F, 0x43)?;
        // delay_ms(1);

        self.write_to_register(0x000F, 0x40)?;
        self.write_to_register(0x000A, 0x01)?;
        // delay_ms(100);

        self.write_to_register(0x7fff, 0x00)?;
        self.poll_for_answer(1, 0, 0x06, 0xff, 1)?;

        self.write_to_register(0x000E, 0x01)?;
        self.write_to_register(0x7fff, 0x02)?;

        /* Enable FW access */
        self.write_to_register(0x7fff, 0x01)?;
        self.write_to_register(0x06, 0x01)?;
        self.poll_for_answer(1, 0, 0x21, 0xFF, 0x4)?;

        self.write_to_register(0x7fff, 0x00)?;

        /* Enable host access to GO1 */
        self.read_from_register(0x7fff, &mut tmp)?;
        self.write_to_register(0x0C, 0x01)?;

        /* Power ON status */
        self.write_to_register(0x7fff, 0x00)?;
        self.write_to_register(0x101, 0x00)?;
        self.write_to_register(0x102, 0x00)?;
        self.write_to_register(0x010A, 0x01)?;
        self.write_to_register(0x4002, 0x01)?;
        self.write_to_register(0x4002, 0x00)?;
        self.write_to_register(0x010A, 0x03)?;
        self.write_to_register(0x103, 0x01)?;
        self.write_to_register(0x400F, 0x00)?;
        self.write_to_register(0x21A, 0x43)?;
        self.write_to_register(0x21A, 0x03)?;
        self.write_to_register(0x21A, 0x01)?;
        self.write_to_register(0x21A, 0x00)?;
        self.write_to_register(0x219, 0x00)?;
        self.write_to_register(0x21B, 0x00)?;

        /* Wake up MCU */
        self.write_to_register(0x7fff, 0x00)?;
        self.read_from_register(0x7fff, &mut tmp)?;

        self.write_to_register(0x7fff, 0x01)?;
        self.write_to_register(0x20, 0x07)?;
        self.write_to_register(0x20, 0x06)?;

        /* Download FW into VL53L8CX */
        self.write_to_register(0x7fff, 0x09)?;
        self.write_multi_to_register(0, &VL53L8CX_FIRMWARE[0..0x8000])?;
        self.write_to_register(0x7fff, 0x0a)?;
        self.write_multi_to_register(0, &VL53L8CX_FIRMWARE[0x8000..0x10000])?;
        self.write_to_register(0x7fff, 0x0b)?;
        self.write_multi_to_register(0, &VL53L8CX_FIRMWARE[0x10000..])?;

        self.write_to_register(0x7fff, 0x01)?;

        /* Check if FW correctly downloaded */
        self.write_to_register(0x7fff, 0x01)?;
        self.write_to_register(0x06, 0x03)?;
// TODO
        // delay_ms(5);
        self.write_to_register(0x7fff, 0x00)?;
        self.read_from_register(0x7fff, &mut tmp)?;
        self.write_to_register(0x0C, 0x01)?;

        /* Reset MCU and wait boot */
        self.write_to_register(0x7FFF, 0x00)?;
        self.write_to_register(0x114, 0x00)?;
        self.write_to_register(0x115, 0x00)?;
        self.write_to_register(0x116, 0x42)?;
        self.write_to_register(0x117, 0x00)?;
        self.write_to_register(0x0B, 0x00)?;
        self.read_from_register(0x7fff, &mut tmp)?;
        self.write_to_register(0x0C, 0x00)?;
        self.write_to_register(0x0B, 0x01)?;

        self.poll_for_mcu_boot()?;

        self.write_to_register(0x7fff, 0x02)?;

        /* Get offset NVM data and store them into the offset buffer */
        self.write_multi_to_register(0x2fd8, &VL53L8CX_GET_NVM_CMD)?;
        self.poll_for_answer(4, 0, VL53L8CX_UI_CMD_STATUS, 0xff, 2)?;


        self.read_from_register(VL53L8CX_UI_CMD_START,  &mut self.temp_buffer)?;
// TODO
        // (void)memcpy(p_dev->offset_data, p_dev->temp_buffer,VL53L8CX_OFFSET_BUFFER_SIZE);
        self.send_offset_data(VL53L8CX_RESOLUTION_4X4)?;

        /* Set default Xtalk shape. Send Xtalk to sensor */
// TODO
        // (void)memcpy(p_dev->xtalk_data, (uint8_t *)VL53L8CX_DEFAULT_XTALK,VL53L8CX_XTALK_BUFFER_SIZE);
      
        self.send_xtalk_data(VL53L8CX_RESOLUTION_4X4)?;
      
        /* Send default configuration to VL53L8CX firmware */
        self.write_multi_to_register(0x2c34,&VL53L8CX_DEFAULT_CONFIGURATION)?;
        self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xff, 0x03)?;
        self.dci_write_data(&mut pipe_ctrl, VL53L8CX_DCI_PIPE_CONTROL)?;
      
        if VL53L8CX_NB_TARGET_PER_ZONE != 1 {
            tmp[0] = VL53L8CX_NB_TARGET_PER_ZONE as u8;
            self.dci_replace_data(&mut self.temp_buffer, VL53L8CX_DCI_FW_NB_TARGET, &mut tmp, 0x0C)?;
        }
      
        self.dci_write_data(&mut single_range, VL53L8CX_DCI_SINGLE_RANGE)?;
      

        Ok(())
    }
}
    
    // pub fn set_i2c_address(&mut self, i2c_address: u16) -> Result<(), Error<B::Error>> {
    //         self.write_to_register(0x7fff, 0x00)?;
    //         self.write_to_register(0x4, (i2c_address >> 1).try_into().unwrap())?;
    //         self.address = i2c_address;
    //         self.write_to_register(0x7fff, 0x02)?;
        
    //         Ok(())
    //     }
        
        //     pub fn is_alive(&mut self) -> Result<(), Error<B::Error>> {
            //         let device_id: [u8, 1] = [0];
            //         let revision_id: [u8, 1] = [0];
            //         self.write_to_register(0x7fff, 0x00)?;
//         self.read_from_register(0, &mut device_id)?;
//         self.read_from_register(1, &mut revision_id)?;
//         self.write_to_register(0x7fff, 0x02)?;
//         if (device_id == 0xF0 as u8) && (revision_id == 0x0C as u8) {
//             return Ok(());
//           } else {
//             return Error();
//           }
//     }



#[entry]
fn main() -> ! {
    
    loop {
        
    }
}