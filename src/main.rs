#![no_std]
#![no_main]

use panic_halt as _; 
use cortex_m_rt::entry;
use embedded_hal_bus::i2c;
use bitfield::bitfield;

use stm32f4xx_hal::{
    i2c::{I2c1, Mode},
    serial::{Config, Tx},
    prelude::*,
    pac::{self, USART2}
};

use embedded_hal::{
    i2c::{I2c, SevenBitAddress},
    spi::{SpiDevice, Operation}
};

use core::{
    fmt::Write, 
    convert::TryInto,
    mem::{size_of, size_of_val},
    cell::RefCell
};

use buffers::*;
use consts::*;

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

    lpn_pin: i8,
    i2c_rst_pin: i8,
    
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

#[repr(C)]
pub struct MotionIndicator {
    global_indicator_1: u32,
    global_indicator_2: u32,
    status: u8,
    nb_of_detected_aggregates: u8,
    nb_of_aggregates: u8,
    spare: u8,
    motion: [u32; 32],
}

impl MotionIndicator {
    pub fn new() -> Self {
        let global_indicator_1: u32 = 0;
        let global_indicator_2: u32 = 0;
        let status: u8 = 0;
        let nb_of_detected_aggregates: u8 = 0;
        let nb_of_aggregates: u8 = 0;
        let spare: u8 = 0;
        let motion: [u32; 32] = [0; 32];
        Self { global_indicator_1, global_indicator_2, status, nb_of_detected_aggregates, nb_of_aggregates, spare, motion }
    }
}

#[repr(C)]
pub struct ResultsData {
    silicon_temp_degc: i8,
    ambient_per_spad: [u32; VL53L8CX_RESOLUTION_8X8 as usize],
    nb_target_detected: [u8; VL53L8CX_RESOLUTION_8X8 as usize],
    nb_spads_enabled: [u32; VL53L8CX_RESOLUTION_8X8 as usize],
    signal_per_spad: [u32; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
    range_sigma_mm: [u16; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
    distance_mm: [i16; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
    reflectance: [u8; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
    target_status: [u8; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
    motion_indicator: MotionIndicator
} 

impl ResultsData {
    pub fn new() -> Self {
        let silicon_temp_degc: i8 = 0;
        let ambient_per_spad: [u32; VL53L8CX_RESOLUTION_8X8 as usize] = [0; VL53L8CX_RESOLUTION_8X8 as usize];
        let nb_target_detected:[u8; VL53L8CX_RESOLUTION_8X8 as usize] = [0; VL53L8CX_RESOLUTION_8X8 as usize];
        let nb_spads_enabled:[u32; VL53L8CX_RESOLUTION_8X8 as usize] = [0; VL53L8CX_RESOLUTION_8X8 as usize];
        let signal_per_spad: [u32; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)] = [0; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)];
        let range_sigma_mm: [u16; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)] = [0; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)];
        let distance_mm: [i16; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)] = [0; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)];
        let reflectance: [u8; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)] = [0; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)];
        let target_status: [u8; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)] = [0; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)];
        let motion_indicator: MotionIndicator = MotionIndicator::new();
        Self { silicon_temp_degc, ambient_per_spad, nb_spads_enabled, nb_target_detected, signal_per_spad, range_sigma_mm, distance_mm, reflectance, target_status, motion_indicator }
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
    fn write_read(&mut self, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, wbuf, rbuf)?;
        
        Ok(())
    }
}

pub struct Vl53l8cxSPI<P> {
    spi: P,
    cs_pin: i8
}

// new for spi
impl<P: SpiDevice> Vl53l8cxSPI<P> {
    pub fn new(spi: P, cs_pin: i8) -> Self {
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

impl<P> Vl53l8cx<Vl53l8cxI2C<P>> 
    where
    P: I2c,
{
    pub fn new_i2c(i2c: P, address: u8, lpn_pin: i8, i2c_rst_pin: i8) -> Result<Self, Error<P::Error>> {
        let streamcount: u8 = 0;
        let data_read_size: u32 = 0;
        let bus: Vl53l8cxI2C<P> = Vl53l8cxI2C::new(i2c, address as SevenBitAddress);
        let temp_buffer: [u8; VL53L8CX_TEMPORARY_BUFFER_SIZE] = [0; VL53L8CX_TEMPORARY_BUFFER_SIZE];
        let offset_data: [u8; VL53L8CX_OFFSET_BUFFER_SIZE] = [0; VL53L8CX_OFFSET_BUFFER_SIZE];
        let xtalk_data: [u8; VL53L8CX_XTALK_BUFFER_SIZE] = [0; VL53L8CX_XTALK_BUFFER_SIZE];
        let instance: Vl53l8cx<Vl53l8cxI2C<P>> = Self { 
            bus, 
            temp_buffer, 
            offset_data, 
            xtalk_data, 
            streamcount, 
            data_read_size, 
            lpn_pin, 
            i2c_rst_pin 
        };
        Ok(instance)
    }

    pub fn set_i2c_address(&mut self, i2c_address: u8) -> Result<(), Error<P::Error>> {
        self.write_to_register(0x7fff, 0x00)?;
        self.write_to_register(0x4, i2c_address >> 1)?;
        self.bus.address = i2c_address;
        self.write_to_register(0x7fff, 0x02)?;
        
        Ok(())
    }

    pub fn init_sensor(&mut self, address: u8) -> Result<(), Error<P::Error>>{
        self.off()?;
        self.on()?;
        self.set_i2c_address(address)?;
        self.is_alive()?;
        self.init()?;
        Ok(())
    }

    pub fn begin(&mut self) -> Result<(), Error<P::Error>> {
        // self.lpn_pin.into_push_pull_output().set_low();
        // self.i2c_rst_pin.into_push_pull_output().set_low();
        Ok(())
    }

    pub fn end(&mut self) -> Result<(), Error<P::Error>> {
        // self.lpn_pin.into_input();
        // self.i2c_rst_pin.into_input();
        Ok(())
    }
}

impl<P> Vl53l8cx<Vl53l8cxSPI<P>> 
    where P: SpiDevice,
{
    pub fn new_spi(spi: P, cs_pin: i8, lpn_pin: i8, i2c_rst_pin: i8) -> Result<Self, Error<P::Error>> {
        let streamcount: u8 = 0;
        let data_read_size: u32 = 0;
        let bus: Vl53l8cxSPI<P> = Vl53l8cxSPI::new(spi, cs_pin);
        let temp_buffer: [u8; VL53L8CX_TEMPORARY_BUFFER_SIZE] = [0; VL53L8CX_TEMPORARY_BUFFER_SIZE];
        let offset_data: [u8; VL53L8CX_OFFSET_BUFFER_SIZE] = [0; VL53L8CX_OFFSET_BUFFER_SIZE];
        let xtalk_data: [u8; VL53L8CX_XTALK_BUFFER_SIZE] = [0; VL53L8CX_XTALK_BUFFER_SIZE];
        let instance: Vl53l8cx<Vl53l8cxSPI<P>> = Self { 
            bus, 
            temp_buffer, 
            offset_data, 
            xtalk_data, 
            streamcount, 
            data_read_size,
            lpn_pin,
            i2c_rst_pin
        };
        
        Ok(instance)
    }

    pub fn init_sensor(&mut self) -> Result<(), Error<P::Error>>{
        self.off()?;
        self.on()?;
        self.is_alive()?;
        self.init()?;
        Ok(())
    }
    
    pub fn begin(&mut self) -> Result<(), Error<P::Error>> {
        // self.lpn_pin.into_push_pull_output().set_low();
        // self.i2c_rst_pin.into_push_pull_output().set_low();
        // self.bus.cs_pin.into_push_pull_output().set_high();
        Ok(())
    }

    pub fn end(&mut self) -> Result<(), Error<P::Error>> {
        // self.lpn_pin.into_input();
        // self.i2c_rst_pin.into_input();
        // self.bus.cs_pin.into_input();
        Ok(())
    }
}



impl<B: BusOperation> Vl53l8cx<B> {



    pub fn on(&mut self) -> Result<(), Error<B::Error>>{
        // self.lpn_pin.set_high();
        let dp = pac::Peripherals::take().unwrap();
        let cp = cortex_m::Peripherals::take().unwrap();
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();
        let mut delay = cp.SYST.delay(&clocks);
        delay.delay_ms(10);
        Ok(())
    }

    pub fn off(&mut self) -> Result<(), Error<B::Error>>{
        // self.lpn_pin.set_low();
        let dp = pac::Peripherals::take().unwrap();
        let cp = cortex_m::Peripherals::take().unwrap();
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();
        let mut delay = cp.SYST.delay(&clocks);
        delay.delay_ms(10);
        Ok(())
    }

    pub fn i2c_reset(&mut self) -> Result<(), Error<B::Error>>{
        let dp = pac::Peripherals::take().unwrap();
        let cp = cortex_m::Peripherals::take().unwrap();
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();
        let mut delay = cp.SYST.delay(&clocks);
        // self.i2c_rst_pin.set_low();
        delay.delay_ms(10);
        // self.i2c_rst_pin.set_high();
        delay.delay_ms(10);
        // self.i2c_rst_pin.set_low();
        delay.delay_ms(10);
        Ok(())
    }

    


    fn poll_for_answer(&mut self, size: usize, pos: u8, reg: u16, mask: u8, expected_val: u8) -> Result<(), Error<B::Error>> {
        let mut timeout: u8 = 0;
        let dp = pac::Peripherals::take().unwrap();
        let cp = cortex_m::Peripherals::take().unwrap();
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();
        let mut delay = cp.SYST.delay(&clocks);

        loop {
            if self.temp_buffer[pos as usize] & mask == expected_val {
                return Ok(());
            }
            self.read_from_register_to_temp_buffer(reg, size)?;
            delay.delay_ms(10);
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
        let dp = pac::Peripherals::take().unwrap();
        let cp = cortex_m::Peripherals::take().unwrap();
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();
        let mut delay = cp.SYST.delay(&clocks);
        
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
            delay.delay_ms(1);
            timeout += 1;
            if go2_status0[0] & 0x01 != 0 {
                return Ok(());
            }
        }
    }

    fn send_offset_data(&mut self, resolution: u8) -> Result<(), Error<B::Error>> {
        let mut signal_grid: [u32; 64] = [0; 64];
        let mut range_grid: [u16; 64] = [0; 64];
        let dss_4x4: [u8; 8] = [0x0F, 0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07];
        let footer: [u8; 8] = [0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x01, 0xE4];

        self.temp_buffer[..VL53L8CX_OFFSET_BUFFER_SIZE].copy_from_slice(&self.offset_data);

        /* Data extrapolation is required for 4X4 offset */
        if resolution == VL53L8CX_RESOLUTION_4X4 {
            self.temp_buffer[0x10..0x10+dss_4x4.len()].copy_from_slice(&dss_4x4);
            self.swap_temp_buffer(VL53L8CX_OFFSET_BUFFER_SIZE)?;
            for (i, chunk) in self.temp_buffer[0x3c..0x3c+256].chunks(4).enumerate() {
                signal_grid[i] = (chunk[0] as u32) << 24 | (chunk[1] as u32) << 16 | (chunk[2] as u32) << 8 | (chunk[3] as u32);
            }
            for (i, chunk) in self.temp_buffer[0x140..0x140+128].chunks(2).enumerate() {
                range_grid[i] = (chunk[0] as u16) << 8 | (chunk[1] as u16);
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
            for (i, &num) in signal_grid.iter().enumerate() {
                self.temp_buffer[0x3c+ i*4..0x3c+ i*4 + 4].copy_from_slice(&num.to_ne_bytes()); 
            }

            for (i, &num) in range_grid.iter().enumerate() {
                self.temp_buffer[0x140+ i*4..0x140+ i*4 + 4].copy_from_slice(&num.to_ne_bytes()); 
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

        self.temp_buffer[..VL53L8CX_XTALK_BUFFER_SIZE].copy_from_slice(&self.xtalk_data);

        /* Data extrapolation is required for 4X4 Xtalk */
        if resolution == VL53L8CX_RESOLUTION_4X4 {
            self.temp_buffer[0x8..0x8 + res4x4.len()].copy_from_slice(&res4x4);
            self.temp_buffer[0x020..0x020 + dss_4x4.len()].copy_from_slice(&dss_4x4);

            self.swap_temp_buffer(VL53L8CX_XTALK_BUFFER_SIZE)?;
            for (i, chunk) in self.temp_buffer[0x34..0x34+256].chunks(4).enumerate() {
                signal_grid[i] = (chunk[0] as u32) << 24 | (chunk[1] as u32) << 16 | (chunk[2] as u32) << 8 | (chunk[3] as u32);
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
            for (i, &num) in signal_grid.iter().enumerate() {
                self.temp_buffer[0x34+ i*4..0x34+ i*4 + 4].copy_from_slice(&num.to_ne_bytes()); 
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
        let dp = pac::Peripherals::take().unwrap();
        let cp = cortex_m::Peripherals::take().unwrap();
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();
        let mut delay = cp.SYST.delay(&clocks);

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
        delay.delay_ms(1);

        self.write_to_register(0x000F, 0x40)?;
        self.write_to_register(0x000A, 0x01)?;
        delay.delay_ms(100);

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
        delay.delay_ms(5);
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
        let power_mode: u8;
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

    fn get_resolution(&mut self) -> Result<u32, Error<B::Error>> {
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_ZONE_CONFIG, 8)?;
        let resolution: u32 = (self.temp_buffer[0x00] * self.temp_buffer[0x01]) as u32;

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
        let resolution: u32 = self.get_resolution()?;
        let mut tmp: [u16; 1] = [0];
        let mut header_config: [u32; 2] = [0, 0];
        let cmd: [u8; 4] = [0x00, 0x03, 0x00, 0x00];

        self.data_read_size = 0;
        self.streamcount = 255;
        let mut bh: BlockHeader;

        let mut output_bh_enable: [u32; 4] = [0x00000007, 0x00000000, 0x00000000, 0x00000000];

        let mut output: [u32; 12] = [
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
            (1-VL53L8CX_DISABLE_AMBIENT_PER_SPAD) * 8
            + (1-VL53L8CX_DISABLE_NB_SPADS_ENABLED) * 16
            + (1-VL53L8CX_DISABLE_NB_TARGET_DETECTED) * 32
            + (1-VL53L8CX_DISABLE_SIGNAL_PER_SPAD) * 64
            + (1-VL53L8CX_DISABLE_RANGE_SIGMA_MM) * 128
            + (1-VL53L8CX_DISABLE_DISTANCE_MM) * 256
            + (1-VL53L8CX_DISABLE_REFLECTANCE_PERCENT) * 512
            + (1-VL53L8CX_DISABLE_TARGET_STATUS) * 1024
            + (1-VL53L8CX_DISABLE_MOTION_INDICATOR) * 2048;

        /* Update data size */
        let upper_bound = size_of_val(&output) / size_of::<u32>(); // 48 ?
        for i in 0..upper_bound {
            if output[i] == 0 || output_bh_enable[i/32] & (1 << (i%32)) == 0 {
                continue;
            }
            bh = BlockHeader(output[i]);
            if bh.bh_type() >= 0x01 && bh.bh_type() < 0x0d {
                if bh.bh_idx() >= 0x54d0 && bh.bh_idx() < 0x54d0 + 960 {
                    bh.set_bh_size(resolution);
                } else {
                    bh.set_bh_size(resolution * VL53L8CX_NB_TARGET_PER_ZONE);
                }
                self.data_read_size += bh.bh_type() * bh.bh_size();
            } else {
                self.data_read_size += bh.bh_size();
            }
            self.data_read_size += 4;
        }
        self.data_read_size += 24;

        let mut buf: [u8; 48] = [0; 48];
        for (i, &num) in output.iter().enumerate() {
            buf[i*4..i*4 + 4].copy_from_slice(&num.to_ne_bytes()); 
        }
        self.dci_write_data(&mut buf, VL53L8CX_DCI_OUTPUT_LIST, 48)?;
        for (i, chunk) in buf.chunks(4).enumerate() {
            output[i] = (chunk[0] as u32) << 24 | (chunk[1] as u32) << 16 | (chunk[2] as u32) << 8 | (chunk[3] as u32);
        }
        
        header_config[0] = self.data_read_size;
        header_config[1] = upper_bound as u32;

        let mut buf: [u8; 8] = [0; 8];
        for (i, &num) in header_config.iter().enumerate() {
            buf[i*4..i*4 + 4].copy_from_slice(&num.to_ne_bytes()); 
        }
        self.dci_write_data(&mut buf, VL53L8CX_DCI_OUTPUT_CONFIG, 8)?;
        for (i, chunk) in buf.chunks(4).enumerate() {
            header_config[i] = (chunk[0] as u32) << 24 | (chunk[1] as u32) << 16 | (chunk[2] as u32) << 8 | (chunk[3] as u32);
        }
        for (i, &num) in output_bh_enable.iter().enumerate() {
            buf[i*4..i*4 + 4].copy_from_slice(&num.to_ne_bytes()); 
        }
        self.dci_write_data(&mut buf, VL53L8CX_DCI_OUTPUT_ENABLES, output_bh_enable.len())?;
        for (i, chunk) in buf.chunks(4).enumerate() {
            output_bh_enable[i] = (chunk[0] as u32) << 24 | (chunk[1] as u32) << 16 | (chunk[2] as u32) << 8 | (chunk[3] as u32);
        }
        /* Start xshut bypass (interrupt mode) */
        self.write_to_register(0x7fff, 0x00)?;
        self.write_to_register(0x09, 0x05)?;
        self.write_to_register(0x7fff, 0x02)?;

        /* Start ranging session */
        self.write_multi_to_register(VL53L8CX_UI_CMD_END - (4-1), &cmd)?;

        self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xff, 0x03)?;

        /* Read ui range data content and compare if data size is the correct one */
        self.dci_read_data_temp_buffer(0x5440, 12)?;

        for (i, chunk) in self.temp_buffer[..2].chunks(2).enumerate() {
            tmp[i] = (chunk[0] as u16) << 8 | (chunk[1] as u16);
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
        let dp = pac::Peripherals::take().unwrap();
        let cp = cortex_m::Peripherals::take().unwrap();
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();
        let mut delay = cp.SYST.delay(&clocks);

        self.read_from_register(0x2ffc, &mut buf)?;
        for (i, chunk) in buf.chunks(4).enumerate() {
            auto_flag_stop[i] = (chunk[0] as u32) << 24 | (chunk[1] as u32) << 16 | (chunk[2] as u32) << 8 | (chunk[3] as u32);
        }

        if auto_flag_stop[0] != 0x4ff {
            self.write_to_register(0x7fff, 0x00)?;

            /* Provoke MCU stop */
            self.write_to_register(0x15, 0x16)?;
            self.write_to_register(0x14, 0x01)?;

            /* Poll for G02 status 0 MCU stop */
            loop {
                if tmp[0] & (0x80 as u8) >> 7 == 0x00 {
                    break;
                }

                self.read_from_register(0x6, &mut tmp)?;
                delay.delay_ms(10);
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
        for (i, chunk) in self.temp_buffer[3..3+4].chunks(4).enumerate() {
            tmp[i] = (chunk[0] as u32) << 24 | (chunk[1] as u32) << 16 | (chunk[2] as u32) << 8 | (chunk[3] as u32);
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
        let is_sync_pin_enabled: u8;
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
        let target_order: u8;
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_TARGET_ORDER, 4)?;
        target_order = self.temp_buffer[0];

        Ok(target_order)
    }

    fn set_target_order(&mut self, target_order: u8) -> Result<(), Error<B::Error>> {
        if target_order == VL53L8CX_TARGET_ORDER_CLOSEST || target_order == VL53L8CX_TARGET_ORDER_STRONGEST {
            self.dci_replace_data_temp_buffer(VL53L8CX_DCI_TARGET_ORDER, 4, &[target_order], 1, 0x0)?;
        } else {
            return Err(Error::Other);
        }
        Ok(())
    }

    fn get_sharpener_percent(&mut self) -> Result<u8, Error<B::Error>> {
        let sharpener_percent: u8;
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_SHARPENER, 16)?;
        sharpener_percent = self.temp_buffer[0xD] * 100 / 255;

        Ok(sharpener_percent)
    }

    fn set_sharpener_percent(&mut self, sharpener_percent: u8) -> Result<(), Error<B::Error>> {
        let sharpener: u8;
        if sharpener_percent >= 100 {
            return Err(Error::Other);
        }
        sharpener = sharpener_percent * 255 / 100;
        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_SHARPENER, 16, &[sharpener], 1, 0xd)?;

        Ok(())
    }

    fn get_integration_time(&mut self) -> Result<u32, Error<B::Error>> {
        let mut time_ms: [u32; 1] = [0];
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_INT_TIME, 20)?;
        for (i, chunk) in self.temp_buffer[..4].chunks(4).enumerate() {
            time_ms[i] = (chunk[0] as u32) << 24 | (chunk[1] as u32) << 16 | (chunk[2] as u32) << 8 | (chunk[3] as u32);
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
        
        let mut buf: [u8; 4] = [0; 4];
        let tmp: [u32; 1] = [integration];
        for (i, &num) in tmp.iter().enumerate() {
            buf[i*4..i*4 + 4].copy_from_slice(&num.to_ne_bytes()); 
        }
        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_INT_TIME, 20, &buf, 4, 0x00)?;

        Ok(())
    }

    fn get_integration_timranging_mode(&mut self) -> Result<u8, Error<B::Error>> {
        let ranging_mode: u8;
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_RANGING_MODE, 8)?;
        if self.temp_buffer[1] == 1 {
            ranging_mode = VL53L8CX_RANGING_MODE_CONTINUOUS;
        } else {
            ranging_mode = VL53L8CX_RANGING_MODE_AUTONOMOUS;
        }
        Ok(ranging_mode)
    }

    fn set_ranging_mode(&mut self, ranging_mode: u8) -> Result<(), Error<B::Error>> {
        let single_range: u32;
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_RANGING_MODE, 8)?;

        if ranging_mode == VL53L8CX_RANGING_MODE_CONTINUOUS {
            self.temp_buffer[1] = 1;
            self.temp_buffer[3] = 3;
            single_range = 0;
        } else if ranging_mode == VL53L8CX_RANGING_MODE_CONTINUOUS {
            self.temp_buffer[1] = 3;
            self.temp_buffer[3] = 2;
            single_range = 1;
        } else {
            return Err(Error::Other);
        }

        self.dci_write_data_temp_buffer(VL53L8CX_DCI_RANGING_MODE, 8)?;
// TODO 
        let mut buf: [u8; 4] = [0; 4];
        let tmp: [u32; 1] = [single_range]; 
        for (i, &num) in tmp.iter().enumerate() {
            buf[i*4..i*4 + 4].copy_from_slice(&num.to_ne_bytes()); 
        }
        self.dci_write_data(&mut buf, VL53L8CX_DCI_SINGLE_RANGE, 4)?;
        
        Ok(())
    }

    fn get_frequency_hz(&mut self) -> Result<u8, Error<B::Error>> {
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_FREQ_HZ, 4)?;
        let frequency_hz: u8 = self.temp_buffer[0x01];

        Ok(frequency_hz)
    }

    fn set_frequency_hz(&mut self, frequency_hz: u8) -> Result<(), Error<B::Error>> {
        let tmp: [u8; 1] = [frequency_hz];
        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_FREQ_HZ, 4, &tmp, 1, 0x01)?;

        Ok(())
    }

    fn check_data_ready(&mut self) -> Result<u8, Error<B::Error>> {
        let is_ready: u8;
        self.read_from_register_to_temp_buffer(0, 4)?;
        if (self.temp_buffer[0] != self.streamcount)
        && (self.temp_buffer[1] == 5)
            && (self.temp_buffer[2] & 5 == 5)
            && (self.temp_buffer[3] & 10 == 10) {
                is_ready = 1;
                self.streamcount = self.temp_buffer[0];
        } else {
            if self.temp_buffer[3] & 0x80 != 0 {
// TODO
                return Err(Error::Other);
            }
            is_ready = 0;
        }

        Ok(is_ready)
    }

    fn get_ranging_data(&mut self) -> Result<ResultsData, Error<B::Error>> {
        let mut result: ResultsData = ResultsData::new();
        let mut msize: u32;
        let mut header_id: u16;
        let mut footer_id: u16;
        let mut bh: BlockHeader;
        self.read_from_register_to_temp_buffer(0, self.data_read_size as usize)?;
        self.streamcount = self.temp_buffer[0];
        self.swap_temp_buffer(self.data_read_size as usize)?;

  /* Start conversion at position 16 to avoid headers */
        let mut i: usize = 16;
        loop {
            if i >= self.data_read_size as usize {
                break;
            }

            bh = BlockHeader(
                (self.temp_buffer[i  ] as u32) << 24 | 
                (self.temp_buffer[i+1] as u32) << 16 | 
                (self.temp_buffer[i+2] as u32) <<  8 | 
                (self.temp_buffer[i+3] as u32)
            );

            if bh.bh_type() > 0x1 && bh.bh_type() < 0xd {
                msize = bh.bh_type() * bh.bh_size();
            } else  {
                msize = bh.bh_size();
            }

            if bh.bh_idx() == VL53L8CX_METADATA_IDX as u32 {
                result.silicon_temp_degc = self.temp_buffer[i+12] as i8;
            } else if VL53L8CX_DISABLE_AMBIENT_PER_SPAD == 0 && bh.bh_idx() == VL53L8CX_AMBIENT_RATE_IDX as u32 {
                for (j, chunk) in self.temp_buffer[i+4..i+4+msize as usize].chunks(4).enumerate() {
                    result.ambient_per_spad[j] = (chunk[0] as u32) << 24 | (chunk[1] as u32) << 16 | (chunk[2] as u32) << 8 | (chunk[3] as u32);
                }
            } else if VL53L8CX_DISABLE_NB_SPADS_ENABLED == 0 && bh.bh_idx() == VL53L8CX_SPAD_COUNT_IDX as u32 {
                for (i, chunk) in self.temp_buffer[i+4..i+4+msize as usize].chunks(4).enumerate() {
                    result.nb_spads_enabled[i] = (chunk[0] as u32) << 24 | (chunk[1] as u32) << 16 | (chunk[2] as u32) << 8 | (chunk[3] as u32);
                }
            } else if VL53L8CX_DISABLE_NB_TARGET_DETECTED == 0 && bh.bh_idx() == VL53L8CX_NB_TARGET_DETECTED_IDX as u32 {
                for (j, &num) in self.temp_buffer[i+4..i+4+msize as usize].iter().enumerate() {
                    result.nb_target_detected[j*4..j*4 + 4].copy_from_slice(&num.to_ne_bytes()); 
                }
            } else if VL53L8CX_DISABLE_SIGNAL_PER_SPAD == 0 && bh.bh_idx() == VL53L8CX_SIGNAL_RATE_IDX as u32 {
                for (i, chunk) in self.temp_buffer[i+4..i+4+msize as usize].chunks(4).enumerate() {
                    result.signal_per_spad[i] = (chunk[0] as u32) << 24 | (chunk[1] as u32) << 16 | (chunk[2] as u32) << 8 | (chunk[3] as u32);
                }
            } else if VL53L8CX_DISABLE_RANGE_SIGMA_MM == 0 && bh.bh_idx() == VL53L8CX_RANGE_SIGMA_MM_IDX as u32 {
                for (i, chunk) in self.temp_buffer[i+4..i+4+msize as usize].chunks(2).enumerate() {
                    result.range_sigma_mm[i] = (chunk[0] as u16) << 8 | (chunk[1] as u16);
                }
            } else if VL53L8CX_DISABLE_DISTANCE_MM == 0 && bh.bh_idx() == VL53L8CX_DISTANCE_IDX as u32 {
                for (i, chunk) in self.temp_buffer[i+4..i+4+msize as usize].chunks(2).enumerate() {
                    result.distance_mm[i] = (chunk[0] as i16) << 8 | (chunk[1] as i16);
                }
            } else if VL53L8CX_DISABLE_REFLECTANCE_PERCENT == 0 && bh.bh_idx() == VL53L8CX_REFLECTANCE_EST_PC_IDX as u32 {
                for (j, &num) in self.temp_buffer[i+4..i+4+msize as usize].iter().enumerate() {
                    result.reflectance[j*4..j*4 + 4].copy_from_slice(&num.to_ne_bytes()); 
                }
            } else if VL53L8CX_DISABLE_TARGET_STATUS == 0 && bh.bh_idx() == VL53L8CX_TARGET_STATUS_IDX as u32 {
                for (j, &num) in self.temp_buffer[i+4..i+4+msize as usize].iter().enumerate() {
                    result.target_status[j*4..j*4 + 4].copy_from_slice(&num.to_ne_bytes()); 
                }
            } else if VL53L8CX_DISABLE_MOTION_INDICATOR == 0 && bh.bh_idx() == VL53L8CX_MOTION_DETEC_IDX as u32 {
                let ptr: *const MotionIndicator = self.temp_buffer[i+4..i+4+msize as usize].as_ptr() as *const MotionIndicator;
                result.motion_indicator = unsafe { ptr.read() };
            }
            i += 4+msize as usize;
        }
        if VL53L8CX_USE_RAW_FORMAT == 0 {
            /* Convert data into their real format */
            if VL53L8CX_DISABLE_AMBIENT_PER_SPAD == 0 {
                for i in 0..VL53L8CX_RESOLUTION_8X8 as usize {
                    result.ambient_per_spad[i] /= 2048;
                }
            }
            for i in 0..(VL53L8CX_RESOLUTION_8X8 as usize)*(VL53L8CX_NB_TARGET_PER_ZONE as usize) {
                if VL53L8CX_DISABLE_DISTANCE_MM == 0 {
                    result.distance_mm[i] /= 4;
                    if result.distance_mm[i] < 0 {
                        result.distance_mm[i] = 0;
                    }
                }
                if VL53L8CX_DISABLE_REFLECTANCE_PERCENT == 0 {
                    result.reflectance[i] /= 128;
                }
                if VL53L8CX_DISABLE_SIGNAL_PER_SPAD == 0 {
                    result.signal_per_spad[i] /= 2048;
                }
            }
            /* Set target status to 255 if no target is detected for this zone */
            if VL53L8CX_DISABLE_NB_TARGET_DETECTED == 0 {
                for i in 0..VL53L8CX_RESOLUTION_8X8 as usize {
                    if result.nb_target_detected[i] == 0 {
                        for j in 0..VL53L8CX_NB_TARGET_PER_ZONE as usize {
                            if VL53L8CX_DISABLE_TARGET_STATUS == 0 {
                                result.target_status[VL53L8CX_NB_TARGET_PER_ZONE as usize*i + j] = 255;
                            }
                        }
                    }
                }
            }

            if VL53L8CX_DISABLE_MOTION_INDICATOR == 0 {
                for i in 0..32 {
                    result.motion_indicator.motion[i] /= 65535;
                }
            }
        }
        
        /* Check if footer id and header id are matching. This allows to detect corrupted frames */
        header_id = (self.temp_buffer[8] as u16) << 8 & 0xff00;
        header_id |= (self.temp_buffer[9] as u16) & 0x00ff;

        footer_id = (self.temp_buffer[self.data_read_size as usize - 4] as u16) << 8 & 0xff00;
        footer_id |= (self.temp_buffer[self.data_read_size as usize - 3] as u16) & 0x00ff;

        if header_id != footer_id {
            return Err(Error::Other);
        }

        Ok(result)
    }    
    
}

fn write_first_line(tx: &mut Tx<USART2>, width: usize) {
    write!(tx, "+-----------").unwrap();
    for _ in 1..width {
        write!(tx, "+-----------").unwrap();
    }
    writeln!(tx, "+").unwrap();
}

fn write_middle_line(tx: &mut Tx<USART2>, width: usize) {
    write!(tx, "+-----------").unwrap();
    for _ in 1..width {
        write!(tx, "+-----------").unwrap();
    }
    writeln!(tx, "+").unwrap();
}

fn write_last_line(tx: &mut Tx<USART2>, width: usize) {
    write!(tx, "+-----------").unwrap();
    for _ in 1..width {
        write!(tx, "+-----------").unwrap();
    }
    writeln!(tx, "+").unwrap();
}

fn write_results(tx: &mut Tx<USART2>, results: &ResultsData, width: usize) {
    
    // ┼ ├ ┤ ┬ ┴ ┌ └ ┐ ┘ │ ─

    writeln!(tx, "\x1B[2J").unwrap();

    writeln!(tx, "\x1b[1m53L8A1 Simple Ranging demo application\x1b[0m\n").unwrap();
    writeln!(tx, "\x1b[4mCell Format :\x1b[0m\n").unwrap();
    writeln!(tx, "\x1b[96m{dis:20}\x1b[0m : \x1b[92m{sta:20}\x1b[0m", dis="Distance [mm]", sta="Status").unwrap();
    writeln!(tx, "\x1b[93m{sig:20}\x1b[0m : \x1b[91m{amb:20}\x1b[0m", sig="Signal [kcps/spad]", amb="Ambient [kcps/spad]").unwrap();

    write_first_line(tx, width);
    for j in 0..width {
        for i in 0..width {
            write!(tx, "|\x1b[96m{dis:4}\x1b[0m : \x1b[92m{sta:4}\x1b[0m", dis=results.distance_mm[width*j+i], sta=results.target_status[width*j+i]).unwrap();
        } write!(tx, "|\n").unwrap();

        for i in 0..width {
            write!(tx, "|\x1b[93m{sig:4}\x1b[0m : \x1b[91m{amb:4}\x1b[0m", sig=results.signal_per_spad[width*j+i], amb=results.ambient_per_spad[width*j+i]).unwrap();
        } write!(tx, "|\n").unwrap();
        
        if j != width-1 {
            write_middle_line(tx, width);
        } else {
            write_last_line(tx, width);
        }
    }
}



bitfield! {
    struct BlockHeader(u32);
    bh_idx, set_bh_idx: 16, 12;
    bh_size, set_bh_size: 12, 4;
    bh_type, set_bh_type: 4, 0;
}


#[entry]
fn main() -> ! {
    let mut results = ResultsData::new();
    
    let width = 4;
    
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let scl = gpiob.pb8;
    let sda = gpiob.pb9;
    let tx_pin = gpioa.pa2.into_alternate();
    let mut delay = cp.SYST.delay(&clocks);
    
    let mut tx: Tx<pac::USART2> = dp
    .USART2
    .tx(tx_pin,
        Config::default()
        .baudrate(115200.bps())
        .wordlength_8()
        .parity_none(),
        &clocks)
        .unwrap(); 
    
    let i2c = I2c1::new(
        dp.I2C1,
        (scl, sda),
        Mode::Standard { frequency:  100.kHz() },
        &clocks);
        
    write_results(&mut tx, &results, width);

    let i2c_bus = RefCell::new(i2c);
    let mut sensor = Vl53l8cx::new_i2c(i2c::RefCellDevice::new(&i2c_bus), 0x52, -1, -1).unwrap();
    let _ = sensor.begin();
    let _ = sensor.init_sensor(0x52);
    let _ = sensor.start_ranging();
    let mut ready: u8 = 0;
    

    loop {
        loop {  
            if ready != 0 { break; }
            ready = sensor.check_data_ready().unwrap();
        }
        results = sensor.get_ranging_data().unwrap();
        write_results(&mut tx, &results, width);
        delay.delay_ms(1000);
    }
}


