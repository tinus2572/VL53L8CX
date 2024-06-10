#![no_main]
#![no_std]
#![allow(dead_code)]

use embedded_hal::i2c::{self, I2c, SevenBitAddress};
use embedded_hal::spi::{SpiDevice, Operation};
use panic_halt as _; 
use cortex_m_rt::entry;
use stm32f4xx_hal::pac::stk::calib::TENMS_R;
use core::convert::TryInto;
use core::f64::consts::E;
use core::mem::size_of;
use core::mem::size_of_val;
use core::ptr::copy_nonoverlapping;
use crate::buffers::*;
use crate::consts::*;

mod buffers;
mod consts;

 
pub struct Vl53l8cxI2C<P> {
    i2c: P,
    address: SevenBitAddress,
}

pub struct Vl53l8cx<B: BusOperation> {
    temp_buffer: [u8;  VL53L8CX_TEMPORARY_BUFFER_SIZE],
    offset_data: [u8;  VL53L8CX_OFFSET_BUFFER_SIZE],
    xtalk_data: [u8; VL53L8CX_XTALK_BUFFER_SIZE],

    streamcount: u8,
    data_read_size: u32,

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

pub struct BlockHeader {
    bh_type: u32,
    bh_size: u32,
    bh_idx: u32 
}

impl BlockHeader {
    pub fn new() -> Self {
        let bh_type = 4;
        let bh_size = 12;
        let bh_idx = 16;
        Self {bh_type, bh_size, bh_idx }
    }
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
    
    fn read_from_register_to_temp_buffer(&mut self, reg: u16, size: usize) -> Result<(), Error<B::Error>> {
        let a: u8 = (reg >> 8).try_into().unwrap();
        let b: u8 = (reg & 0xFF).try_into().unwrap(); 
        self.bus
            .write_read(&[a, b], &mut self.temp_buffer[..size])
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
    
    fn write_multi_to_register_temp_buffer(&mut self, reg: u16, size: usize) -> Result<(), Error<B::Error>> {
        let a: u8 = (reg >> 8).try_into().unwrap();
        let b: u8 = (reg & 0xFF).try_into().unwrap(); 
        self.bus
            .write(&[a, b])
            .map_err(Error::Bus)?;
        self.bus
            .write(&self.temp_buffer[..size])
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
        let streamcount: u8 = 0;
        let data_read_size: u32 = 0;
        let bus: Vl53l8cxI2C<P> = Vl53l8cxI2C::new(i2c, address as SevenBitAddress);
        let temp_buffer: [u8; VL53L8CX_TEMPORARY_BUFFER_SIZE] = [0; VL53L8CX_TEMPORARY_BUFFER_SIZE];
        let offset_data: [u8; VL53L8CX_OFFSET_BUFFER_SIZE] = [0; VL53L8CX_OFFSET_BUFFER_SIZE];
        let xtalk_data: [u8; VL53L8CX_XTALK_BUFFER_SIZE] = [0; VL53L8CX_XTALK_BUFFER_SIZE];
        let instance: Vl53l8cx<Vl53l8cxI2C<P>> = Self { bus, temp_buffer, offset_data, xtalk_data, streamcount, data_read_size };
        Ok(instance)
    }

    pub fn set_i2c_address(&mut self, i2c_address: u8) -> Result<(), Error<P::Error>> {
        self.write_to_register(0x7fff, 0x00)?;
        self.write_to_register(0x4, i2c_address >> 1)?;
        self.bus.address = i2c_address;
        self.write_to_register(0x7fff, 0x02)?;
        
        Ok(())
    }
}

impl<P> Vl53l8cx<Vl53l8cxSPI<P>> 
    where P: SpiDevice,
{
    pub fn new_spi(spi: P) -> Result<Self, Error<P::Error>> {
        let streamcount: u8 = 0;
        let data_read_size: u32 = 0;
        let bus: Vl53l8cxSPI<P> = Vl53l8cxSPI::new(spi);
        let temp_buffer: [u8; VL53L8CX_TEMPORARY_BUFFER_SIZE] = [0; VL53L8CX_TEMPORARY_BUFFER_SIZE];
        let offset_data: [u8; VL53L8CX_OFFSET_BUFFER_SIZE] = [0; VL53L8CX_OFFSET_BUFFER_SIZE];
        let xtalk_data: [u8; VL53L8CX_XTALK_BUFFER_SIZE] = [0; VL53L8CX_XTALK_BUFFER_SIZE];
        let instance: Vl53l8cx<Vl53l8cxSPI<P>> = Self { bus, temp_buffer, offset_data, xtalk_data, streamcount, data_read_size };
        
        Ok(instance)
    }
}



impl<B: BusOperation> Vl53l8cx<B> {

    fn poll_for_answer(&mut self, size: usize, pos: u8, reg: u16, mask: u8, expected_val: u8) -> Result<(), Error<B::Error>> {
        let mut timeout: u8 = 0;

        loop {
            if self.temp_buffer[pos as usize] & mask == expected_val {
                return Ok(());
            }
            self.read_from_register_to_temp_buffer(reg, size)?;
// TODO
            // delay_ms(10);
            if timeout >= 200 { /* 2s timeout */
                return Err(Error::Other);
            } else if size >= 4 && self.temp_buffer[2] >= 0x7F {
                return Err(Error::Other);
            } else {
                timeout+=1;
            }

        }
    }

    fn poll_for_mcu_boot(&mut self) -> Result<(), Error<B::Error>> {
        let mut go2_status0: [u8; 1] = [0];
        let mut go2_status1: [u8; 1] = [0];
        let mut timeout: u16 = 0;
        
        loop {
            if timeout >= 500 {
                return Ok(());
            }
            self.read_from_register(0x06, &mut go2_status0)?;
            if go2_status0[0] & 0x80 != 0 {
                self.read_from_register(0x07, &mut go2_status1)?;
                if go2_status1[0] & 0x01 != 0 {
                    return Ok(());
                }
            }
// TODO
            // delay_ms(1);
            timeout+=1;
            if go2_status0[0] & 0x01 != 0 {
                return Ok(());
            }
        }
    }

    fn send_offset_data(&mut self, resolution: u8) -> Result<(), Error<B::Error>> {
        let mut signal_grid: [u32; 64] = [0; 64];
        let mut range_grid: [u16; 64] = [0; 64];
        let signal_grid_size: usize = size_of_val(&signal_grid) / size_of::<u32>();
        let range_grid_size: usize = size_of_val(&range_grid) / size_of::<u16>();
        let dss_4x4: [u8; 8] = [0x0F, 0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07];
        let footer: [u8; 8] = [0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x01, 0xE4];

        self.temp_buffer[..VL53L8CX_OFFSET_BUFFER_SIZE].copy_from_slice(&self.offset_data);

        /* Data extrapolation is required for 4X4 offset */
        if resolution == VL53L8CX_RESOLUTION_4X4 {
            self.temp_buffer[0x10..0x10+dss_4x4.len()].copy_from_slice(&dss_4x4);
            self.swap_temp_buffer(VL53L8CX_OFFSET_BUFFER_SIZE)?;
            
            unsafe {
                let src: *const u32 = self.temp_buffer.as_ptr().add(0x3C) as *const u32;
                let dst: *mut u32 = signal_grid.as_mut_ptr();
                copy_nonoverlapping(src, dst, signal_grid_size);
            }
            
            unsafe {
                let src: *const u16 = self.temp_buffer.as_ptr().add(0x140) as *const u16;
                let dst: *mut u16 = range_grid.as_mut_ptr();
                copy_nonoverlapping(src, dst, range_grid_size);
            }

            for j in 0..4 {
                for i in 0..4 {
                    signal_grid[i + (4 * j)] = (
                          signal_grid[(2 * i) + (16 * j) + 0]
                        + signal_grid[(2 * i) + (16 * j) + 1]
                        + signal_grid[(2 * i) + (16 * j) + 8]
                        + signal_grid[(2 * i) + (16 * j) + 9]
                    ) / 4;
                    range_grid[i + (4 * j)] = (
                          range_grid[(2 * i) + (16 * j) + 0]
                        + range_grid[(2 * i) + (16 * j) + 1]
                        + range_grid[(2 * i) + (16 * j) + 8]
                        + range_grid[(2 * i) + (16 * j) + 9]
                    ) / 4;
                }
            }

            unsafe {
                let src: *const u8 = signal_grid.as_ptr() as *const u8;
                let dst: *mut u8 = self.temp_buffer.as_mut_ptr().add(0x3C);
                copy_nonoverlapping(src, dst, signal_grid_size);
            }
            
            unsafe {
                let src: *const u8 = range_grid.as_ptr() as *const u8;
                let dst: *mut u8 = self.temp_buffer.as_mut_ptr().add(0x140);
                copy_nonoverlapping(src, dst, range_grid_size);
            }

            self.swap_temp_buffer(VL53L8CX_OFFSET_BUFFER_SIZE)?;
        }

        for i in 0..VL53L8CX_OFFSET_BUFFER_SIZE-4 {
            self.temp_buffer[i] = self.temp_buffer[i+8];
        }

        self.temp_buffer[0x1E0..0x1E0+footer.len()].copy_from_slice(&footer);
        self.write_multi_to_register_temp_buffer(0x2E18, VL53L8CX_OFFSET_BUFFER_SIZE)?;
        self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xFF, 0x03)?;

        Ok(())
    }   

    fn send_xtalk_data(&mut self, resolution: u8) -> Result<(), Error<B::Error>> {
        let res4x4: [u8; 8] = [0x0F, 0x04, 0x04, 0x17, 0x08, 0x10, 0x10, 0x07];
        let dss_4x4: [u8; 8] = [0x00, 0x78, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08];
        let profile_4x4: [u8; 4] = [0xA0, 0xFC, 0x01, 0x00];
        let mut signal_grid: [u32; 64] = [0; 64];
        let signal_grid_size: usize = size_of_val(&signal_grid) / size_of::<u32>();


        self.temp_buffer[..VL53L8CX_XTALK_BUFFER_SIZE].copy_from_slice(&self.xtalk_data);

        /* Data extrapolation is required for 4X4 Xtalk */
        if resolution == VL53L8CX_RESOLUTION_4X4 {
            self.temp_buffer[0x8..0x8 + res4x4.len()].copy_from_slice(&res4x4);
            self.temp_buffer[0x020..0x020 + dss_4x4.len()].copy_from_slice(&dss_4x4);

            self.swap_temp_buffer(VL53L8CX_XTALK_BUFFER_SIZE)?;

            unsafe {
                let src: *const u32 = self.temp_buffer.as_ptr().add(0x34) as *const u32;
                let dst: *mut u32 = signal_grid.as_mut_ptr();
                copy_nonoverlapping(src, dst, signal_grid_size);
            }

            for j in 0..3 {
                for i in 0..3 {
                    signal_grid[i + (4 * j)] = (
                        signal_grid[(2 * i) + (16 * j) + 0]
                      + signal_grid[(2 * i) + (16 * j) + 1]
                      + signal_grid[(2 * i) + (16 * j) + 8]
                      + signal_grid[(2 * i) + (16 * j) + 9]
                  ) / 4;
                }
            }

            unsafe {
                let src: *const u8 = signal_grid.as_ptr() as *const u8;
                let dst: *mut u8 = self.temp_buffer.as_mut_ptr().add(0x34);
                copy_nonoverlapping(src, dst, signal_grid_size);
            }
            self.swap_temp_buffer(VL53L8CX_XTALK_BUFFER_SIZE)?;
            self.temp_buffer[0x134..0x134+profile_4x4.len()].copy_from_slice(&profile_4x4);
            let tmp: [u8; 4] = [0; 4];
            self.temp_buffer[0x078..0x078+4].copy_from_slice(&tmp);
        }

        self.write_multi_to_register_temp_buffer(0x2CF8, VL53L8CX_XTALK_BUFFER_SIZE)?;
        self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xFF, 0x03)?;

        Ok(())
    }  


    pub fn is_alive(&mut self) -> Result<(), Error<B::Error>> {
        let mut device_id: [u8; 1] = [0];
        let mut revision_id: [u8; 1] = [0];
        self.write_to_register(0x7fff, 0x00)?;
        self.read_from_register(0, &mut device_id)?;
        self.read_from_register(1, &mut revision_id)?;
        self.write_to_register(0x7fff, 0x02)?;
        if (device_id[0] != 0xF0 as u8) || (revision_id[0] != 0x0C as u8) {
            return Err(Error::Other);
        }

        Ok(())
    }


    
    fn dci_read_data(&mut self, data: &mut [u8], index: u16, data_size: usize) -> Result<(), Error<B::Error>> {
        let mut cmd: [u8; 12] = [
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x0f,
            0x00, 0x02, 0x00, 0x08
        ];

        if data_size + 12 > VL53L8CX_TEMPORARY_BUFFER_SIZE {
            return Err(Error::Other);
        } else {
            cmd[0] = (index >> 8) as u8;
            cmd[1] = (index & 0xff) as u8;
            cmd[2] = ((data_size & 0xff0) >> 4) as u8;
            cmd[3] = ((data_size & 0xf) << 4) as u8;

            /* Request data reading from FW */
            self.write_multi_to_register(VL53L8CX_UI_CMD_END - 11, &cmd)?;
            self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xFF, 0x03)?;

            /* Read new data sent (4 bytes header + data_size + 8 bytes footer) */
            self.read_from_register_to_temp_buffer(VL53L8CX_UI_CMD_START, VL53L8CX_TEMPORARY_BUFFER_SIZE)?;
            self.swap_temp_buffer(data_size)?;

            /* Copy data from FW into input structure (-4 bytes to remove header) */
            for i in 0..(data_size-1) {
                data[i] = self.temp_buffer[i+4];
            }
        }
       
        Ok(())
    }   
    
    fn dci_read_data_temp_buffer(&mut self, index: u16, data_size: usize) -> Result<(), Error<B::Error>> {
        let mut cmd: [u8; 12] = [
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x0f,
            0x00, 0x02, 0x00, 0x08
        ];

        if data_size + 12 > VL53L8CX_TEMPORARY_BUFFER_SIZE {
            return Err(Error::Other);
        } else {
            cmd[0] = (index >> 8) as u8;
            cmd[1] = (index & 0xff) as u8;
            cmd[2] = ((data_size & 0xff0) >> 4) as u8;
            cmd[3] = ((data_size & 0xf) << 4) as u8;

            /* Request data reading from FW */
            self.write_multi_to_register(VL53L8CX_UI_CMD_END - 11, &cmd)?;
            self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xFF, 0x03)?;

            /* Read new data sent (4 bytes header + data_size + 8 bytes footer) */
            self.read_from_register_to_temp_buffer(VL53L8CX_UI_CMD_START, VL53L8CX_TEMPORARY_BUFFER_SIZE)?;
            self.swap_temp_buffer(data_size)?;

            /* Copy data from FW into input structure (-4 bytes to remove header) */
            for i in 0..(data_size-1) {
                self.temp_buffer[i] = self.temp_buffer[i+4];
            }
        }
       
        Ok(())
    }   
    
    fn dci_write_data(&mut self, data: &mut [u8], index: u16, data_size: usize) -> Result<(), Error<B::Error>> {
        let mut headers: [u8; 4] = [0x00, 0x00, 0x00, 0x00];
        let footer: [u8; 8] = [0x00, 0x00, 0x00, 0x0f, 0x05, 0x01,
            ((data_size + 8) >> 8) as u8,
            ((data_size + 8) & 0xFF) as u8
        ];
        
        let address: u16 = VL53L8CX_UI_CMD_END - (data_size as u16 + 12) + 1;

        /* Check if cmd buffer is large enough */
        if (data_size + 12) > VL53L8CX_TEMPORARY_BUFFER_SIZE {
            return Err(Error::Other);
        } else {
            headers[0] = (index >> 8) as u8;
            headers[1] = (index & 0xff) as u8;
            headers[2] = ((data_size & 0xff0) >> 4) as u8;
            headers[3] = ((data_size & 0xf) << 4) as u8;

            /* Copy data from structure to FW format (+4 bytes to add header) */
            self.swap_buffer(data, data_size)?;
            for i in 0..(data_size-1) {
                self.temp_buffer[data_size-1 - i+4] = data[data_size-1 - i];
            }

            /* Add headers and footer */
            self.temp_buffer[..headers.len()].copy_from_slice(&headers);
            self.temp_buffer[(data_size + footer.len())..].copy_from_slice(&footer);

            /* Send data to FW */
            self.write_multi_to_register_temp_buffer(address, data_size + 12)?;
            self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xff, 0x03)?;

            self.swap_buffer(data, data_size)?;
        }

        Ok(())
    }
    
    fn dci_write_data_temp_buffer(&mut self, index: u16, data_size: usize) -> Result<(), Error<B::Error>> {
        let mut headers: [u8; 4] = [0x00, 0x00, 0x00, 0x00];
        let footer: [u8; 8] = [0x00, 0x00, 0x00, 0x0f, 0x05, 0x01,
            ((data_size + 8) >> 8) as u8,
            ((data_size + 8) & 0xFF) as u8
        ];
        
        let address: u16 = VL53L8CX_UI_CMD_END - (data_size as u16 + 12) + 1;

        /* Check if cmd buffer is large enough */
        if (data_size + 12) > VL53L8CX_TEMPORARY_BUFFER_SIZE {
            return Err(Error::Other);
        } else {
            headers[0] = (index >> 8) as u8;
            headers[1] = (index & 0xff) as u8;
            headers[2] = ((data_size & 0xff0) >> 4) as u8;
            headers[3] = ((data_size & 0xf) << 4) as u8;

            /* Copy data from structure to FW format (+4 bytes to add header) */
            self.swap_temp_buffer(data_size)?;
            for i in 0..(data_size-1) {
                self.temp_buffer[data_size-1 - i+4] = self.temp_buffer[data_size-1 - i];
            }

            /* Add headers and footer */
            self.temp_buffer[..headers.len()].copy_from_slice(&headers);
            self.temp_buffer[(data_size + footer.len())..].copy_from_slice(&footer);

            /* Send data to FW */
            self.write_multi_to_register_temp_buffer(address, data_size + 12)?;
            self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xff, 0x03)?;

            self.swap_temp_buffer(data_size)?;
        }

        Ok(())
    }   


    fn dci_replace_data(&mut self, data: &mut [u8], index: u16, data_size: usize, new_data: &[u8], new_data_size: usize, new_data_pos: usize) -> Result<(), Error<B::Error>> {
        self.dci_read_data(data, index, data_size)?;
        data[new_data_pos..].copy_from_slice(&new_data[..new_data_size]);
        self.dci_write_data(data, index, data_size)?;
        
        Ok(())
    }   
    
    fn dci_replace_data_temp_buffer(&mut self, index: u16, data_size: usize, new_data: &[u8], new_data_size: usize, new_data_pos: usize) -> Result<(), Error<B::Error>> {
        self.dci_read_data_temp_buffer(index, data_size)?;
        self.temp_buffer[new_data_pos..].copy_from_slice(&new_data[..new_data_size]);
        self.dci_write_data_temp_buffer(index, data_size)?;
        
        Ok(())
    }   

    pub fn swap_buffer(&mut self, buf: &mut [u8], size: usize) -> Result<(), Error<B::Error>> {
        for chunk in buf[..size].chunks_exact_mut(4) {
            let tmp: u32 = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            chunk.copy_from_slice(&tmp.to_le_bytes());
        }

        Ok(())
    }

    pub fn swap_temp_buffer(&mut self, size: usize) -> Result<(), Error<B::Error>> {
        for chunk in self.temp_buffer[..size].chunks_exact_mut(4) {
            let tmp: u32 = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            chunk.copy_from_slice(&tmp.to_le_bytes());
        }

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

        self.read_from_register_to_temp_buffer(VL53L8CX_UI_CMD_START,VL53L8CX_NVM_DATA_SIZE)?;
        self.offset_data.copy_from_slice(&self.temp_buffer[..VL53L8CX_OFFSET_BUFFER_SIZE]);
        self.send_offset_data(VL53L8CX_RESOLUTION_4X4)?;

        /* Set default Xtalk shape. Send Xtalk to sensor */
        self.xtalk_data.copy_from_slice(&VL53L8CX_DEFAULT_XTALK);
        self.send_xtalk_data(VL53L8CX_RESOLUTION_4X4)?;
      
        /* Send default configuration to VL53L8CX firmware */
        self.write_multi_to_register(0x2c34,&VL53L8CX_DEFAULT_CONFIGURATION)?;
        self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xff, 0x03)?;
        self.dci_write_data(&mut pipe_ctrl, VL53L8CX_DCI_PIPE_CONTROL, size_of::<u8>())?;
      
        if VL53L8CX_NB_TARGET_PER_ZONE != 1 {
            tmp[0] = VL53L8CX_NB_TARGET_PER_ZONE as u8;
            self.dci_replace_data_temp_buffer(VL53L8CX_DCI_FW_NB_TARGET, 16, &mut tmp, 1, 0x0C)?;
        }
      
        self.dci_write_data(&mut single_range, VL53L8CX_DCI_SINGLE_RANGE, size_of::<u8>())?;
      

        Ok(())
    }

    
    fn get_power_mode(&mut self) -> Result<u8, Error<B::Error>> {
        let mut power_mode: u8 = 0;
        let mut tmp: [u8; 1] = [0];
        self.write_to_register(0x7fff, 0x00)?;
        self.read_from_register(0x009, &mut tmp)?;    
        if tmp[0] == 0x4 {
            power_mode = VL53L8CX_POWER_MODE_WAKEUP;
        } else if tmp[0] == 0x2 {
            power_mode = VL53L8CX_POWER_MODE_SLEEP;
        } else {
            return Err(Error::Other);
        }
        self.write_to_register(0x7fff, 0x02)?;
        
        Ok(power_mode)
    }
    
    fn set_power_mode(&mut self, power_mode: u8) -> Result<(), Error<B::Error>> {
        let current_power_mode: u8 = self.get_power_mode()?;
        if power_mode != current_power_mode {
            if power_mode == VL53L8CX_POWER_MODE_WAKEUP {
                self.write_to_register(0x7fff, 0x00)?;
                self.write_to_register(0x09, 0x04)?;
                self.poll_for_answer(1, 0, 0x06, 0x01, 1)?;
            } else if power_mode == VL53L8CX_POWER_MODE_SLEEP {
                self.write_to_register(0x7fff, 0x00)?;
                self.write_to_register(0x09, 0x02)?;
                self.poll_for_answer(1, 0, 0x06, 0x01, 1)?;
            } else {
                return Err(Error::Other);
            }
        }
        self.write_to_register(0x7fff, 0x02)?;
        
        Ok(())
    }

    fn get_resolution(&mut self) -> Result<u8, Error<B::Error>> {
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_ZONE_CONFIG, 8)?;
        let resolution: u8 = self.temp_buffer[0x00] * self.temp_buffer[0x01];

        Ok(resolution)
    }

    fn set_resolution(&mut self, resolution: u8) -> Result<(), Error<B::Error>> {

        if resolution == VL53L8CX_RESOLUTION_4X4 {
            self.dci_read_data_temp_buffer(VL53L8CX_DCI_DSS_CONFIG, 16)?;
            self.temp_buffer[0x04] = 64;
            self.temp_buffer[0x06] = 64;
            self.temp_buffer[0x09] = 4;
            self.dci_write_data_temp_buffer(VL53L8CX_DCI_DSS_CONFIG, 16)?;
            self.dci_read_data_temp_buffer(VL53L8CX_DCI_DSS_CONFIG, 16)?;
            self.temp_buffer[0x00] = 4;
            self.temp_buffer[0x01] = 4;
            self.temp_buffer[0x04] = 8;
            self.temp_buffer[0x05] = 8;
            self.dci_write_data_temp_buffer(VL53L8CX_DCI_ZONE_CONFIG, 8)?;
        } else if resolution == VL53L8CX_RESOLUTION_8X8 {
            self.dci_read_data_temp_buffer(VL53L8CX_DCI_DSS_CONFIG, 16)?;
            self.temp_buffer[0x04] = 16;
            self.temp_buffer[0x06] = 16;
            self.temp_buffer[0x09] = 1;
            self.dci_write_data_temp_buffer(VL53L8CX_DCI_DSS_CONFIG, 16)?;
            self.dci_read_data_temp_buffer(VL53L8CX_DCI_DSS_CONFIG, 8)?;
            self.temp_buffer[0x00] = 8;
            self.temp_buffer[0x01] = 8;
            self.temp_buffer[0x04] = 4;
            self.temp_buffer[0x05] = 4;
            self.dci_write_data_temp_buffer(VL53L8CX_DCI_ZONE_CONFIG, 8)?;
        } else {
            return Err(Error::Other);
        }
        self.send_offset_data(resolution)?;
        self.send_xtalk_data(resolution)?;

        Ok(())
    }



    fn start_ranging(&mut self) -> Result<(), Error<B::Error>> {
        let resolution: u8 = self.get_resolution()?;
        let tmp: [u16; 1] = [0];
        let mut header_config: [u32; 2] = [0, 0];
        let cmd: [u8; 4] = [0x00, 0x03, 0x00, 0x00];

        self.data_read_size = 0;
        self.streamcount = 255;
        let bh: BlockHeader = BlockHeader::new();

        let mut output_bh_enable: [u32; 4] = [0x00000007, 0x00000000, 0x00000000, 0x00000000];

        let output: [u32; 12] = [
            VL53L8CX_START_BH,
            VL53L8CX_METADATA_BH,
            VL53L8CX_COMMONDATA_BH,
            VL53L8CX_AMBIENT_RATE_BH,
            VL53L8CX_SPAD_COUNT_BH,
            VL53L8CX_NB_TARGET_DETECTED_BH,
            VL53L8CX_SIGNAL_RATE_BH,
            VL53L8CX_RANGE_SIGMA_MM_BH,
            VL53L8CX_DISTANCE_BH,
            VL53L8CX_REFLECTANCE_BH,
            VL53L8CX_TARGET_STATUS_BH,
            VL53L8CX_MOTION_DETECT_BH
        ];

        output_bh_enable[0] += 
            VL53L8CX_DISABLE_AMBIENT_PER_SPAD * 8
            + VL53L8CX_DISABLE_NB_SPADS_ENABLED * 16
            + VL53L8CX_DISABLE_NB_TARGET_DETECTED * 32
            + VL53L8CX_DISABLE_SIGNAL_PER_SPAD * 64
            + VL53L8CX_DISABLE_RANGE_SIGMA_MM * 128
            + VL53L8CX_DISABLE_DISTANCE_MM * 256
            + VL53L8CX_DISABLE_REFLECTANCE_PERCENT * 512
            + VL53L8CX_DISABLE_TARGET_STATUS * 1024
            + VL53L8CX_DISABLE_MOTION_INDICATOR * 2048;

        /* Update data size */
        let upper_bound = size_of_val(&output) / size_of::<u32>();
        for i in 0..upper_bound {
            if output[i] == 0 || output_bh_enable[i/32] & (1 << (i%32)) == 0 {
                continue;
            }
// ??????????????????????
            // bh_ptr = (union Block_header *) & (output[i]);
            if bh.bh_type >= 0x01 && bh.bh_type < 0x0d {
                if bh.bh_idx >= 0x54d0 && bh.bh_idx < 0x54d0 + 960 {
                    bh.bh_size = resolution;
                } else {
                    bh.bh_size = (resolution as u16) * (VL53L8CX_NB_TARGET_PER_ZONE as u16);
                }
                self.data_read_size += bh.bh_type * bh.bh_size;
            } else {
                self.data_read_size += bh.bh_size;
            }
            self.data_read_size += 4;
        }
        self.data_read_size += 24;

        self.dci_write_data(&mut output, VL53L8CX_DCI_OUTPUT_LIST, output.len())?;
        
        header_config[0] = self.data_read_size;
        header_config[1] = upper_bound;

        self.dci_write_data(&mut header_config, VL53L8CX_DCI_OUTPUT_CONFIG, header_config.len())?;
        self.dci_write_data(&mut output_bh_enable, VL53L8CX_DCI_OUTPUT_ENABLES, output_bh_enable.len())?;

        /* Start xshut bypass (interrupt mode) */
        self.write_to_register(0x7fff, 0x00)?;
        self.write_to_register(0x09, 0x05)?;
        self.write_to_register(0x7fff, 0x02)?;

        /* Start ranging session */
        self.write_multi_to_register(VL53L8CX_UI_CMD_END - (4-1), &cmd)?;

        self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xff, 0x03)?;

        /* Read ui range data content and compare if data size is the correct one */
        self.dci_read_data_temp_buffer(0x5440, 12)?;

        let tmp_size: usize = size_of_val(&tmp) / size_of::<u16>();
        unsafe {
            let src: *const u16 = self.temp_buffer.as_ptr().add(0x8) as *const u16;
            let dst: *mut u16 = tmp.as_mut_ptr();
            copy_nonoverlapping(src, dst, tmp_size);
        }

        if tmp[0] != self.data_read_size as u16 {
            return Err(Error::Other);
        }

        self.dci_read_data_temp_buffer(0xe0c4, 8)?;
        if self.temp_buffer[0x6] != 0 {
            return Err(Error::Other);
        }

        Ok(())
    }


    fn stop_ranging(&mut self) -> Result<(), Error<B::Error>> {
        let mut timeout: u16 = 0;
        let mut auto_flag_stop: [u32; 1] = [0];
        let mut buf: [u8; 4] = [0; 4];
        let mut tmp: [u8; 1] = [0];
        self.read_from_register(0x2ffc, &mut buf)?;

        unsafe {
            let src: *const u32 = buf.as_ptr() as *const u32;
            let dst: *mut u32 = auto_flag_stop.as_mut_ptr();
            copy_nonoverlapping(src, dst, size_of::<u32>());
        }

        if auto_flag_stop[0] != 0x4ff {
            self.write_to_register(0x7fff, 0x00)?;

            /* Provoke MCU stop */
            self.write_to_register(0x15, 0x16)?;
            self.write_to_register(0x14, 0x01)?;

            /* Poll for G02 status 0 MCU stop */
            loop {
                if tmp & (0x80 as u8) >> 7 == 0x00 {
                    break;
                }

                self.read_from_register(0x6, &mut tmp)?;
// TODO
                // delay(10);
                timeout += 1;

                if timeout > 500 {
                    break;
                }
            }
        }

        /* Check GO2 status 1 if status is still OK */
        self.read_from_register(0x6, &mut tmp)?;
        if tmp[0] & 0x80 != 0 {
            self.read_from_register(0x7, &mut tmp)?;
            if tmp[0] != 0x84 && tmp[0] != 0x85 {
                return Ok(());
            }
        }

        /* Undo MCU stop */
        self.write_to_register(0x7fff, 0x00)?;
        self.write_to_register(0x14, 0x00)?;
        self.write_to_register(0x15, 0x00)?;

        /* Stop xshut bypass */
        self.write_to_register(0x09, 0x04)?;
        self.write_to_register(0x7fff, 0x02)?;

        Ok(())
    }

    fn set_external_sync_pin_enable(&mut self, enable_sync_pin: u8) -> Result<(), Error<B::Error>> {
        let mut tmp: [u32; 1] = [0];
        self.read_from_register_to_temp_buffer(VL53L8CX_DCI_SYNC_PIN, 4)?;

        unsafe {
            let src: *const u32 = self.temp_buffer[3..3+size_of::<u32>()].as_ptr() as *const u32;
            let dst: *mut u32 = tmp.as_mut_ptr();
            copy_nonoverlapping(src, dst, size_of::<u32>());
        }

        /* Update bit 1 with mask (set sync pause bit) */
        if enable_sync_pin == 0 {
            tmp[0] &= !(1 << 1);
        } else {
            tmp[0] |= 1 << 1;
        }

        self.temp_buffer[3] = tmp[0] as u8;
        self.dci_write_data_temp_buffer(VL53L8CX_DCI_SYNC_PIN, 4)?;

        Ok(())
    }

    fn get_external_sync_pin_enable(&mut self) -> Result<u8, Error<B::Error>> {
        let mut is_sync_pin_enabled: u8 = 0;
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_SYNC_PIN, 4)?;

        /* Check bit 1 value (get sync pause bit) */
        if self.temp_buffer[3] & 0x2 != 0 {
            is_sync_pin_enabled = 1;
        } else {
            is_sync_pin_enabled = 0;
        }
        
        Ok(is_sync_pin_enabled)
    }

    fn get_target_order(&mut self) -> Result<u8, Error<B::Error>> {
        let mut target_order: u8 = 0;
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_TARGET_ORDER, 4)?;
        target_order = self.temp_buffer[0];

        Ok(target_order)
    }

    fn set_target_order(&mut self, target_order: u8) -> Result<(), Error<B::Error>> {

        if target_order == VL53L8CX_TARGET_ORDER_CLOSEST || target_order == VL53L8CX_TARGET_ORDER_STRONGEST {
            self.dci_replace_data_temp_buffer(VL53L8CX_DCI_TARGET_ORDER, 4, &target_order, 1, 0x0)?;
        } else {
            return Err(Error::Other);
        }
        Ok(())
    }

    fn get_sharpener_percent(&mut self) -> Result<u8, Error<B::Error>> {
        let mut sharpener_percent: u8 = 0;
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_SHARPENER, 16)?;
        sharpener_percent = self.temp_buffer[0xD] * 100 / 255;

        Ok((sharpener_percent))
    }

    fn set_sharpener_percent(&mut self, sharpener_percent: u8) -> Result<(), Error<B::Error>> {
        let mut sharpener: u8 = 0;
        if sharpener_percent >= 100 {
            return Err(Error::Other);
        }
        sharpener = sharpener_percent * 255 / 100;
        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_SHARPENER, 16, &sharpener, 1, 0xd)?;

        Ok(())
    }

    fn get_integration_time(&mut self) -> Result<u32, Error<B::Error>> {
        let mut time_ms: [u32; 1] = [0];
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_INT_TIME, 20)?;

        unsafe {
            let src: *const u32 = self.temp_buffer[..size_of::<u32>()].as_ptr() as *const u32;
            let dst: *mut u32 = time_ms.as_mut_ptr();
            copy_nonoverlapping(src, dst, size_of::<u32>());
        }
        time_ms[0] /= 1000;
        
        Ok(time_ms[0])
    }

    fn set_integration_time(&mut self, integration_time_ms: u32) -> Result<(), Error<B::Error>> {
        let mut integration: u32 = integration_time_ms;

        /* Integration time must be between 2ms and 1000ms */
        if integration < 2 || integration > 1000 {
            return Err(Error::Other);
        }
        integration *= 1000;

        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_INT_TIME, 20, &integration, 4, 0x00)?;

        Ok(())
    }


}



















#[entry]
fn main() -> ! {
    
    loop {
        
    }
}