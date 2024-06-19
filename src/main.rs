#![no_std]
#![no_main]

use panic_halt as _; 
use cortex_m_rt::entry;
use embedded_hal_bus::i2c;
use bitfield::bitfield;

use stm32f4xx_hal::{
    gpio::{DynamicPin, Output, Pin, PinState::{High, Low}, PushPull}, i2c::{I2c1, Mode}, pac::{self, USART2}, prelude::*, serial::{Config, Tx}, timer::SysDelay
};

use embedded_hal::{
    i2c::{I2c, SevenBitAddress},
    spi::{SpiDevice, Operation}
};

use core::{
    cell::RefCell, convert::TryInto, fmt::Write, mem::{size_of, size_of_val}
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

    lpn_pin: Pin<'B', 0, Output<PushPull>>,
    i2c_rst_pin: i8,
    
    bus: B,
    delay: SysDelay
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

#[repr(u8)]
pub enum Status {
    StatusOk = VL53L8CX_STATUS_OK,
    StatusTimeOutError = VL53L8CX_STATUS_TIMEOUT_ERROR,
    StatusCorruptedFrame = VL53L8CX_STATUS_CORRUPTED_FRAME,
    StatusLaserSafety = VL53L8CX_STATUS_LASER_SAFETY,
    StatusMcuError = VL53L8CX_MCU_ERROR,
    StatusInvalidParam = VL53L8CX_STATUS_INVALID_PARAM,
    StatusError = VL53L8CX_STATUS_ERROR
}

bitfield! {
    struct BlockHeader(u32);
    bh_idx, set_bh_idx: 16, 12;
    bh_size, set_bh_size: 12, 4;
    bh_type, set_bh_type: 4, 0;
}

#[repr(C)]
pub struct DetectionThresholds {
    param_low_thresh: i32,
    param_high_thresh: i32,
    measurement: u8,
    th_type: u8,
    zone_num: u8,
    math_op: u8
}

#[repr(C)]
pub struct MotionConfiguration {
    ref_bin_offset: i32,
    detection_threshold: u32,
    extra_noise_sigma: u32,
    null_den_clip_value: u32,
    mem_update_mode: u8,
    mem_update_choice: u8,
    sum_span: u8,
    feature_length: u8,
    nb_of_aggregates: u8,
    nb_of_temporal_accumulations: u8,
    min_nb_for_global_detection: u8,
    global_indicator_format_1: u8,
    global_indicator_format_2: u8,
    spare_1: u8,
    spare_2: u8,
    spare_3: u8,
    map_id: [i8; 64],
    indicator_format_1: [u8; 32],
    indicator_format_2: [u8; 32],
} 

impl MotionConfiguration {
    pub fn new() -> Self {
        let ref_bin_offset = 13633;
        let detection_threshold = 2883584;
        let extra_noise_sigma = 0;
        let null_den_clip_value = 0;
        let mem_update_mode = 6;
        let mem_update_choice = 2;
        let sum_span = 4;
        let feature_length = 9;
        let nb_of_aggregates = 16;
        let nb_of_temporal_accumulations = 16;
        let min_nb_for_global_detection = 1;
        let global_indicator_format_1 = 8;
        let global_indicator_format_2 = 0;
        let spare_1 = 0;
        let spare_2 = 0;
        let spare_3 = 0;
        let map_id = [0; 64];
        let indicator_format_1 = [0; 32];
        let indicator_format_2 = [0; 32];
        Self { 
            ref_bin_offset, 
            detection_threshold, 
            extra_noise_sigma, 
            null_den_clip_value, 
            mem_update_mode, 
            mem_update_choice, 
            sum_span, 
            feature_length, 
            nb_of_aggregates, 
            nb_of_temporal_accumulations, 
            min_nb_for_global_detection, 
            global_indicator_format_1, 
            global_indicator_format_2, 
            spare_1, 
            spare_2, 
            spare_3, 
            map_id, 
            indicator_format_1, 
            indicator_format_2 
        }
    }
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
    cs_pin: DynamicPin<'B', 6>
}

// new for spi
impl<P: SpiDevice> Vl53l8cxSPI<P> {
    pub fn new(spi: P, cs_pin: DynamicPin<'B', 6>) -> Self {
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
        self.bus.write_read(&[a, b], rbuf).map_err(Error::Bus)?;

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
        let wbuf = [a, b, val];
        self.bus.write(&wbuf).map_err(Error::Bus)?;
       
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
    pub fn new_i2c(i2c: P, address: SevenBitAddress, lpn_pin: Pin<'B', 0, Output<PushPull>>, i2c_rst_pin: i8, delay: SysDelay) -> Result<Self, Error<P::Error>> {
        let streamcount: u8 = 0;
        let data_read_size: u32 = 0;
        let bus: Vl53l8cxI2C<P> = Vl53l8cxI2C::new(i2c, address);
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
            i2c_rst_pin,
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
        self.set_i2c_address(address)?;
        self.is_alive()?;
        self.init()?;
        Ok(())
    }

    pub fn begin(&mut self) -> Result<(), Error<P::Error>> {
        self.lpn_pin.set_low();
        // self.i2c_rst_pin.into_push_pull_output().set_low();
        Ok(())
    }

    pub fn end(&mut self) -> Result<(), Error<P::Error>> {
        // self.lpn_pin.make_floating_input();
        // self.i2c_rst_pin.into_input();
        Ok(())
    }
}

impl<P> Vl53l8cx<Vl53l8cxSPI<P>> 
    where P: SpiDevice,
{
    pub fn new_spi(spi: P, cs_pin: DynamicPin<'B', 6>, lpn_pin: Pin<'B', 0, Output<PushPull>>, i2c_rst_pin: i8, delay: SysDelay) -> Result<Self, Error<P::Error>> {
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
            i2c_rst_pin,
            delay
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
        self.lpn_pin.set_low();
        // self.i2c_rst_pin.into_push_pull_output().set_low();
        self.bus.cs_pin.make_push_pull_output_in_state(High);
        Ok(())
    }

    pub fn end(&mut self) -> Result<(), Error<P::Error>> {
        // self.lpn_pin.make_floating_input();
        // self.i2c_rst_pin.into_input();
        self.bus.cs_pin.make_floating_input();
        Ok(())
    }
}



impl<B: BusOperation> Vl53l8cx<B> {

    pub fn on(&mut self) -> Result<(), Error<B::Error>>{
        self.lpn_pin.set_high();
        self.delay(10);
        Ok(())
    }

    pub fn off(&mut self) -> Result<(), Error<B::Error>>{
        self.lpn_pin.set_low();
        self.delay(10);
        Ok(())
    }

    pub fn i2c_reset(&mut self) -> Result<(), Error<B::Error>>{
        // self.i2c_rst_pin.set_low();
        self.delay(10);
        // self.i2c_rst_pin.set_high();
        self.delay(10);
        // self.i2c_rst_pin.set_low();
        self.delay(10);
        Ok(())
    }

    fn poll_for_answer(&mut self, size: usize, pos: u8, reg: u16, mask: u8, expected_val: u8) -> Result<(), Error<B::Error>> {
        let mut timeout: u8 = 0;

        loop {
            if self.temp_buffer[pos as usize] & mask == expected_val {
                return Ok(());
            }
            self.read_from_register_to_temp_buffer(reg, size)?;
            self.delay(10);
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
            self.delay(1);
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
        self.delay(1);

        self.write_to_register(0x000F, 0x40)?;
        self.write_to_register(0x000A, 0x01)?;
        self.delay(100);

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
        self.delay(5);
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
                    bh.set_bh_size(resolution as u32);
                } else {
                    bh.set_bh_size(resolution as u32 * VL53L8CX_NB_TARGET_PER_ZONE);
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
        let mut buf: [u8; 16] = [0; 16];
        for (i, &num) in output_bh_enable.iter().enumerate() {
            buf[i*4..i*4 + 4].copy_from_slice(&num.to_ne_bytes()); 
        }
        self.dci_write_data(&mut buf, VL53L8CX_DCI_OUTPUT_ENABLES, 16)?;
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
                self.delay(10);
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

    fn get_ranging_mode(&mut self) -> Result<u8, Error<B::Error>> {
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

    pub fn delay(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }

    fn get_detection_threshholds_enable(&mut self) -> Result<u8, Error<B::Error>> {
        let enabled: u8;
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_DET_THRESH_GLOBAL_CONFIG, 8)?;
        enabled = self.temp_buffer[0x1];
        Ok(enabled)
    }

    fn set_detection_threshholds_enable(&mut self, enabled: u8) -> Result<(), Error<B::Error>> {
        let mut grp_global_config: [u8; 4] = [0x01, 0x00, 0x01, 0x00];
        let mut tmp: [u8; 1] = [0];
        if enabled == 1 {
            grp_global_config[0x01] = 0x01;
            tmp[0] = 0x04;
        } else {
            grp_global_config[0x01] = 0x00;
            tmp[0] = 0x0C;
        }
        /* Set global interrupt config */
        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_DET_THRESH_GLOBAL_CONFIG, 8, &grp_global_config, 4, 0x00)?;
        /* Update interrupt config */
        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_DET_THRESH_CONFIG, 20, &tmp, 1, 0x11)?;
        Ok(())
    }

    fn get_detection_threshholds(&mut self, thresholds: &mut [DetectionThresholds; VL53L8CX_NB_THRESHOLDS as usize] ) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; VL53L8CX_NB_THRESHOLDS as usize * 12] = [0; VL53L8CX_NB_THRESHOLDS as usize * 12];
        for i in 0..VL53L8CX_NB_THRESHOLDS as usize {
            arr[i..i+4].copy_from_slice(&thresholds[i].param_low_thresh.to_ne_bytes());
            arr[i+4..i+8].copy_from_slice(&thresholds[i].param_high_thresh.to_ne_bytes());
            arr[i+8] = thresholds[i].measurement;
            arr[i+9] = thresholds[i].th_type;
            arr[i+10] = thresholds[i].zone_num;
            arr[i+11] = thresholds[i].math_op;
        }

        self.dci_read_data(&mut arr, VL53L8CX_DCI_DET_THRESH_START, VL53L8CX_NB_THRESHOLDS as usize * 12)?;
        
        for i in 0..VL53L8CX_NB_THRESHOLDS as usize {
            thresholds[i].param_low_thresh = (arr[i] as i32) << 24 | (arr[i+1] as i32) << 16 | (arr[i+2] as i32) << 8 | (arr[i+3] as i32);
            thresholds[i].param_high_thresh = (arr[i+4] as i32) << 24 | (arr[i+5] as i32) << 16 | (arr[i+6] as i32) << 8 | (arr[i+7] as i32);
            thresholds[i].measurement = arr[i+8];
            thresholds[i].th_type = arr[i+9];
            thresholds[i].zone_num = arr[i+10];
            thresholds[i].math_op = arr[i+11];
        }
        
        for i in 0..VL53L8CX_NB_THRESHOLDS as usize {
            if thresholds[i].measurement == VL53L8CX_DISTANCE_MM {
                thresholds[i].param_low_thresh  /= 4;
                thresholds[i].param_high_thresh /= 4;
            } else if thresholds[i].measurement == VL53L8CX_SIGNAL_PER_SPAD_KCPS {
                thresholds[i].param_low_thresh  /= 2048;
                thresholds[i].param_high_thresh /= 2048;
            } else if thresholds[i].measurement == VL53L8CX_RANGE_SIGMA_MM {
                thresholds[i].param_low_thresh  /= 128;
                thresholds[i].param_high_thresh /= 128;
            } else if thresholds[i].measurement == VL53L8CX_AMBIENT_PER_SPAD_KCPS {
                thresholds[i].param_low_thresh  /= 2048;
                thresholds[i].param_high_thresh /= 2048;
            } else if thresholds[i].measurement == VL53L8CX_NB_SPADS_ENABLED {
                thresholds[i].param_low_thresh  /= 256;
                thresholds[i].param_high_thresh /= 256;
            } else if thresholds[i].measurement == VL53L8CX_MOTION_INDICATOR {
                thresholds[i].param_low_thresh  /= 65535;
                thresholds[i].param_high_thresh /= 65535;
            }
        }
        Ok(())
    }

    fn set_detection_threshholds(&mut self, thresholds: &mut [DetectionThresholds; VL53L8CX_NB_THRESHOLDS as usize] ) -> Result<(), Error<B::Error>> {
        let mut grp_valid_target_cfg: [u8; 8] = [0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05];
        for i in 0..VL53L8CX_NB_THRESHOLDS as usize {
            if thresholds[i].measurement == VL53L8CX_DISTANCE_MM {
                thresholds[i].param_low_thresh  *= 4;
                thresholds[i].param_high_thresh *= 4;
            } else if thresholds[i].measurement == VL53L8CX_SIGNAL_PER_SPAD_KCPS {
                thresholds[i].param_low_thresh  *= 2048;
                thresholds[i].param_high_thresh *= 2048;
            } else if thresholds[i].measurement == VL53L8CX_RANGE_SIGMA_MM {
                thresholds[i].param_low_thresh  *= 128;
                thresholds[i].param_high_thresh *= 128;
            } else if thresholds[i].measurement == VL53L8CX_AMBIENT_PER_SPAD_KCPS {
                thresholds[i].param_low_thresh  *= 2048;
                thresholds[i].param_high_thresh *= 2048;
            } else if thresholds[i].measurement == VL53L8CX_NB_SPADS_ENABLED {
                thresholds[i].param_low_thresh  *= 256;
                thresholds[i].param_high_thresh *= 256;
            } else if thresholds[i].measurement == VL53L8CX_MOTION_INDICATOR {
                thresholds[i].param_low_thresh  *= 65535;
                thresholds[i].param_high_thresh *= 65535;
            }
        } 
        self.dci_write_data(&mut grp_valid_target_cfg, VL53L8CX_DCI_DET_THRESH_VALID_STATUS, 8)?;
        
        
        let mut arr: [u8; VL53L8CX_NB_THRESHOLDS as usize * 12] = [0; VL53L8CX_NB_THRESHOLDS as usize * 12];
        for i in 0..VL53L8CX_NB_THRESHOLDS as usize {
            arr[i..i+4].copy_from_slice(&thresholds[i].param_low_thresh.to_ne_bytes());
            arr[i+4..i+8].copy_from_slice(&thresholds[i].param_high_thresh.to_ne_bytes());
            arr[i+8] = thresholds[i].measurement;
            arr[i+9] = thresholds[i].th_type;
            arr[i+10] = thresholds[i].zone_num;
            arr[i+11] = thresholds[i].math_op;
        }

        self.dci_write_data(&mut arr, VL53L8CX_DCI_DET_THRESH_START, VL53L8CX_NB_THRESHOLDS as usize * 12)?;
        
        for i in 0..VL53L8CX_NB_THRESHOLDS as usize {
            thresholds[i].param_low_thresh = (arr[i] as i32) << 24 | (arr[i+1] as i32) << 16 | (arr[i+2] as i32) << 8 | (arr[i+3] as i32);
            thresholds[i].param_high_thresh = (arr[i+4] as i32) << 24 | (arr[i+5] as i32) << 16 | (arr[i+6] as i32) << 8 | (arr[i+7] as i32);
            thresholds[i].measurement = arr[i+8];
            thresholds[i].th_type = arr[i+9];
            thresholds[i].zone_num = arr[i+10];
            thresholds[i].math_op = arr[i+11];
        }

        Ok(())
    }

    fn get_detection_threshholds_auto_stop(&mut self) -> Result<u8, Error<B::Error>> {
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_PIPE_CONTROL, 4)?;
        let auto_stop: u8 = self.temp_buffer[0x03];
        Ok(auto_stop)
    }
    
    fn set_detection_threshholds_auto_stop(&mut self, auto_stop: u8) -> Result<u8, Error<B::Error>> {
        let tmp: [u8; 1] = [auto_stop];
        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_PIPE_CONTROL, 4, &tmp, 1, 0x03)?;
        Ok(auto_stop)
    }

    fn motion_indicator_init(&mut self, resolution: u8) -> Result<(), Error<B::Error>> {
        let mut motion_config = MotionConfiguration::new();
        self.motion_indicator_set_resolution(&mut motion_config, resolution)?;
        Ok(())
    }

    fn motion_indicator_set_resolution(&mut self, motion_config: &mut MotionConfiguration, resolution: u8) -> Result<(), Error<B::Error>> {
        if resolution == VL53L8CX_RESOLUTION_4X4 {
            for i in 0..VL53L8CX_RESOLUTION_4X4 as usize {
                motion_config.map_id[i] = i as i8;
            }
            for i in 16..VL53L8CX_RESOLUTION_4X4 as usize {
                motion_config.map_id[i] = -1;
            }
        } else if resolution == VL53L8CX_RESOLUTION_8X8 {
            for i in 0..VL53L8CX_RESOLUTION_8X8 as usize {
                motion_config.map_id[i] =  (i as i8 % 8) / 2 + 4 * (i as i8 / 16);
            } 
        } else {
            return Err(Error::Other);
        }

        let mut arr: [u8; 156] = [0; 156];

        arr[..4].copy_from_slice(&motion_config.ref_bin_offset.to_ne_bytes());
        arr[4..8].copy_from_slice(&motion_config.detection_threshold.to_ne_bytes());
        arr[8..12].copy_from_slice(&motion_config.extra_noise_sigma.to_ne_bytes());
        arr[12..16].copy_from_slice(&motion_config.null_den_clip_value.to_ne_bytes());
        arr[16] = motion_config.mem_update_mode;
        arr[17] = motion_config.mem_update_choice;
        arr[18] = motion_config.sum_span;
        arr[19] = motion_config.feature_length;
        arr[20] = motion_config.nb_of_aggregates;
        arr[21] = motion_config.nb_of_temporal_accumulations;
        arr[22] = motion_config.min_nb_for_global_detection;
        arr[23] = motion_config.global_indicator_format_1;
        arr[24] = motion_config.global_indicator_format_2;
        arr[25] = motion_config.spare_1;
        arr[26] = motion_config.spare_2;
        arr[27] = motion_config.spare_3;
        for i in 0..64 {
            arr[28+i] = motion_config.map_id[i] as u8;
        }
        for i in 0..32 {
            arr[92+i] = motion_config.indicator_format_1[i];
            arr[124+i] = motion_config.indicator_format_2[i];
        }

        self.dci_write_data(&mut arr, VL53L8CX_DCI_MOTION_DETECTOR_CFG, 156)?;
        
        motion_config.ref_bin_offset = (arr[0] as i32) << 24 | (arr[1] as i32) << 16 | (arr[2] as i32) << 8 | (arr[3] as i32);
        motion_config.detection_threshold = (arr[4] as u32) << 24 | (arr[5] as u32) << 16 | (arr[6] as u32) << 8 | (arr[7] as u32);
        motion_config.extra_noise_sigma = (arr[8] as u32) << 24 | (arr[9] as u32) << 16 | (arr[10] as u32) << 8 | (arr[11] as u32);
        motion_config.null_den_clip_value = (arr[12] as u32) << 24 | (arr[13] as u32) << 16 | (arr[14] as u32) << 8 | (arr[15] as u32);
        motion_config.mem_update_mode = arr[16];
        motion_config.mem_update_choice = arr[17];
        motion_config.sum_span = arr[18];
        motion_config.feature_length = arr[19];
        motion_config.nb_of_aggregates = arr[20];
        motion_config.nb_of_temporal_accumulations = arr[21];
        motion_config.min_nb_for_global_detection = arr[22];
        motion_config.global_indicator_format_1 = arr[23];
        motion_config.global_indicator_format_2 = arr[24];
        motion_config.spare_1 = arr[25];
        motion_config.spare_2 = arr[26];
        motion_config.spare_3 = arr[27];
        for i in 0..64 {
            motion_config.map_id[i] = arr[28+i] as i8;
        }
        for i in 0..32 {
            motion_config.indicator_format_1[i] = arr[92+i];
            motion_config.indicator_format_2[i] = arr[124+i];
        }
        
        Ok(())
    }

    fn poll_for_answer_xtalk(&mut self, address: u16, expected_val: u8) -> Result<(), Error<B::Error>> {
        let mut timeout: u32 = 0;
        loop {
            if self.temp_buffer[1] == expected_val {
                break;
            }
            self.read_from_register_to_temp_buffer(address, 4)?;
            self.delay(10);
            if timeout >= 200 || self.temp_buffer[2] >= 0x7f {
                return Err(Error::Other);
            } 
            timeout += 1; 
        }
        Ok(())
    }

    fn program_output_config(&mut self) -> Result<(), Error<B::Error>> {
        let mut header_config: [u32; 2] = [0, 0];
        let resolution = self.get_resolution()?;
        let mut bh: BlockHeader;
        self.data_read_size = 0;
        /* Enable mandatory output (meta and common data) */
        let mut output_bh_enable: [u32; 4] = [
            0x0001FFFF,
            0x00000000,
            0x00000000,
            0xC0000000
            ];
            
            /* Send addresses of possible output */
        let mut output: [u32; 17] = [
            0x0000000D,
            0x54000040,
            0x9FD800C0,
            0x9FE40140,
            0x9FF80040,
            0x9FFC0404,
            0xA0FC0100,
            0xA10C0100,
            0xA11C00C0,
            0xA1280902,
            0xA2480040,
            0xA24C0081,
            0xA2540081,
            0xA25C0081,
            0xA2640081,
            0xA26C0084,
            0xA28C0082
        ];
        
        for i in 0..17 {
            if output[i] == 0 || (output_bh_enable[0] & 1 << i) == 0 { 
                continue;
            }
            bh = BlockHeader(output[i]);
            if bh.bh_type() >= 0x1 && bh.bh_type() < 0x0d {
                if bh.bh_idx() >= 0x54d0 && bh.bh_idx() < 0x54d0 + 960 {
                    bh.set_bh_size(resolution as u32);
                } else {
                    bh.set_bh_size(resolution as u32 * VL53L8CX_NB_TARGET_PER_ZONE);
                }
                self.data_read_size += bh.bh_type() * bh.bh_size();
            } else {
                self.data_read_size += bh.bh_size();
            }
            self.data_read_size += 4;
        }
        self.data_read_size += 24;

        let mut buf: [u8; 68] = [0; 68];
        for (i, &num) in output.iter().enumerate() {
            buf[i*4..i*4 + 4].copy_from_slice(&num.to_ne_bytes()); 
        }
        self.dci_write_data(&mut buf, VL53L8CX_DCI_OUTPUT_LIST, 68)?;
        for (i, chunk) in buf.chunks(4).enumerate() {
            output[i] = (chunk[0] as u32) << 24 | (chunk[1] as u32) << 16 | (chunk[2] as u32) << 8 | (chunk[3] as u32);
        }

        header_config[0] = self.data_read_size;
        header_config[1] = 17;

        
        let mut buf: [u8; 8] = [0; 8];
        for (i, &num) in header_config.iter().enumerate() {
            buf[i*4..i*4 + 4].copy_from_slice(&num.to_ne_bytes()); 
        }
        self.dci_write_data(&mut buf, VL53L8CX_DCI_OUTPUT_CONFIG, 8)?;
        for (i, chunk) in buf.chunks(4).enumerate() {
            header_config[i] = (chunk[0] as u32) << 24 | (chunk[1] as u32) << 16 | (chunk[2] as u32) << 8 | (chunk[3] as u32);
        }
        let mut buf: [u8; 16] = [0; 16];
        for (i, &num) in output_bh_enable.iter().enumerate() {
            buf[i*4..i*4 + 4].copy_from_slice(&num.to_ne_bytes()); 
        }
        self.dci_write_data(&mut buf, VL53L8CX_DCI_OUTPUT_ENABLES, 16)?;
        for (i, chunk) in buf.chunks(4).enumerate() {
            output_bh_enable[i] = (chunk[0] as u32) << 24 | (chunk[1] as u32) << 16 | (chunk[2] as u32) << 8 | (chunk[3] as u32);
        }
                
        Ok(())
    }

    fn get_xtalk_margin(&mut self) -> Result<u32, Error<B::Error>> {
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_XTALK_CFG, 16)?;
        let mut xtalk_margin: u32 = (self.temp_buffer[0] as u32) << 24 | (self.temp_buffer[1] as u32) << 16 | (self.temp_buffer[2] as u32) << 8 | self.temp_buffer[3] as u32;
        xtalk_margin /= 2048;
        Ok(xtalk_margin)
    }
     
    fn set_xtalk_margin(&mut self, xtalk_margin: u32) -> Result<(), Error<B::Error>> {
        let mut margin_kcps: [u8; 4] = [0; 4];
        margin_kcps.copy_from_slice(&(xtalk_margin<<11).to_ne_bytes());
        if xtalk_margin > 10000 {
            return Err(Error::Other);
        }
        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_XTALK_CFG, 16, &margin_kcps, 4, 0)?;

        Ok(())
    }
    
    fn calibrate_xtalk(&mut self, reflectance_percent: u16, nb_samples: u8, distance_mm: u16) -> Result<(), Error<B::Error>> {
        let mut timeout: u16 = 0;
        let mut continue_loop: u8 = 1;
        let cmd: [u8; 4] = [0x00, 0x03, 0x00, 0x00];
        let footer: [u8; 8] = [0x00, 0x00, 0x00, 0x0F, 0x00, 0x01, 0x03, 0x04];
        let mut reflectance = reflectance_percent;
        let mut distance = distance_mm;
        let mut samples = nb_samples;
        let resolution = self.get_resolution()?;
        let frequency = self.get_frequency_hz()?;
        let sharpener_percent = self.get_sharpener_percent()?;
        let integration_time_ms = self.get_integration_time()?;
        let target_order = self.get_target_order()?;
        let xtalk_margin = self.get_xtalk_margin()?;
        let ranging_mode = self.get_ranging_mode()?;

        /* Check input arguments validity */
        if reflectance < 1 || reflectance > 99
            || distance < 600 || distance > 3000
            || samples < 1 || samples > 16 {
            return Err(Error::Other);
        }
        self.set_resolution(VL53L8CX_RESOLUTION_8X8)?;

        /* Send Xtalk calibration buffer */
        self.temp_buffer[..984].copy_from_slice(&VL53L8CX_CALIBRATE_XTALK);
        self.write_multi_to_register_temp_buffer(0x2c28, 984)?;
        self.poll_for_answer_xtalk(VL53L8CX_UI_CMD_STATUS, 3)?;

        /* Format input argument */
        reflectance *= 16;
        distance *= 4;

        /* Update required fields */
        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_CAL_CFG, 8, &distance.to_ne_bytes(), 2, 0)?;
        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_CAL_CFG, 8, &reflectance.to_ne_bytes(), 2, 2)?;
        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_CAL_CFG, 8, &[samples], 1, 4)?;

        /* Program output for Xtalk calibration */
        self.program_output_config()?;

        /* Start ranging session */
        self.write_multi_to_register(VL53L8CX_UI_CMD_END - (4-1), &cmd)?;
        self.poll_for_answer_xtalk(VL53L8CX_UI_CMD_STATUS, 3)?;

        /* Wait for end of calibration */
        loop {
            if continue_loop != 1 {
                break;
            }
            self.read_from_register_to_temp_buffer(0, 4)?;
            if self.temp_buffer[0] != VL53L8CX_STATUS_ERROR {
                /* Coverglass too good for Xtalk calibration */
                if self.temp_buffer[2] >= 0x7f && self.temp_buffer[3] & 0x80 >> 7 == 1 {
// TODO
                    // self.default_xtalk = 
                }
                continue_loop = 0;
            } else if timeout >= 400 {
// TODO         
                continue_loop = 0;
                return Err(Error::Other);
            } else {
                timeout += 1;
                self.delay(50);
            }
        }

        /* Save Xtalk data into the Xtalk buffer */
        self.temp_buffer[..72].copy_from_slice(&VL53L8CX_GET_XTALK_CMD);
        self.write_multi_to_register_temp_buffer(0x2fb8, 72)?;
        self.poll_for_answer_xtalk(VL53L8CX_UI_CMD_STATUS, 3)?;
        self.read_from_register_to_temp_buffer(VL53L8CX_UI_CMD_START, VL53L8CX_XTALK_BUFFER_SIZE+4)?;
        self.xtalk_data[..VL53L8CX_XTALK_BUFFER_SIZE-8].copy_from_slice(&self.temp_buffer[8..VL53L8CX_XTALK_BUFFER_SIZE]);
        self.xtalk_data[VL53L8CX_XTALK_BUFFER_SIZE-8..].copy_from_slice(&footer);

        /* Reset default buffer */
        self.write_multi_to_register(0x2c34, &VL53L8CX_DEFAULT_CONFIGURATION)?;
        self.poll_for_answer_xtalk(VL53L8CX_UI_CMD_STATUS, 3)?;

        /* Reset initial configuration */
        self.set_resolution(resolution)?;
        self.set_frequency_hz(frequency)?;
        self.set_integration_time(integration_time_ms)?;
        self.set_sharpener_percent(sharpener_percent)?;
        self.set_target_order(target_order)?;
        self.set_xtalk_margin(xtalk_margin)?;
        self.set_ranging_mode(ranging_mode)?;

        Ok(())
    }

    fn get_caldata_xtalk(&mut self) -> Result<[u8; VL53L8CX_XTALK_BUFFER_SIZE], Error<B::Error>> {
        let footer: [u8; 8] = [0x00, 0x00, 0x00, 0x0F, 0x00, 0x01, 0x03, 0x04];
        let mut xtalk_data: [u8; VL53L8CX_XTALK_BUFFER_SIZE] = [0; VL53L8CX_XTALK_BUFFER_SIZE];
        let resolution = self.get_resolution()?;
        self.set_resolution(VL53L8CX_RESOLUTION_8X8)?;

        self.temp_buffer[..72].copy_from_slice(&VL53L8CX_GET_XTALK_CMD);
        self.write_multi_to_register_temp_buffer(0x2fb8, 72)?;
        self.poll_for_answer_xtalk(VL53L8CX_UI_CMD_STATUS, 3)?;
        self.read_from_register_to_temp_buffer(VL53L8CX_UI_CMD_START, VL53L8CX_XTALK_BUFFER_SIZE+4)?;
        xtalk_data[..VL53L8CX_XTALK_BUFFER_SIZE-8].copy_from_slice(&self.temp_buffer[8..VL53L8CX_XTALK_BUFFER_SIZE]);
        xtalk_data[VL53L8CX_XTALK_BUFFER_SIZE-8..].copy_from_slice(&footer);

        self.set_resolution(resolution)?;

        Ok(xtalk_data)
    }

    fn set_caldata_xtalk(&mut self, xtalk_data: [u8; VL53L8CX_XTALK_BUFFER_SIZE]) -> Result<(), Error<B::Error>> {
        let resolution = self.get_resolution()?;
        self.xtalk_data.copy_from_slice(&xtalk_data);
        self.set_resolution(resolution)?;

        Ok(())
    }

}



//================== Main and related functions ==================

fn write_results(tx: &mut Tx<USART2>, results: &ResultsData, width: usize) {

    // ┼ ├ ┤ ┬ ┴ ┌ └ ┐ ┘ │ ─

    writeln!(tx, "\x1B[2J").unwrap();

    writeln!(tx, "VL53L8A1 Simple Ranging demo application\n").unwrap();
    writeln!(tx, "Cell Format :\n").unwrap();
    writeln!(
        tx, 
        "\x1b[96m{dis:>20}\x1b[0m : \x1b[92m{sta:<20}\x1b[0m", 
        dis="Distance [mm]", 
        sta="Status"
    ).unwrap();
    writeln!(
        tx, 
        "\x1b[93m{sig:>20}\x1b[0m : \x1b[91m{amb:<20}\x1b[0m", 
        sig="Signal [kcps/spad]", 
        amb="Ambient [kcps/spad]"
    ).unwrap();

    for j in 0..width {
        for _ in 0..width { write!(tx, "+-----------").unwrap(); } writeln!(tx, "+").unwrap();
        
        for i in 0..width {
            write!(
                tx, 
                "|\x1b[96m{dis:>4}\x1b[0m : \x1b[92m{sta:<4}\x1b[0m", 
                dis=results.distance_mm[width*j+i], 
                sta=results.target_status[width*j+i]
            ).unwrap();
        } write!(tx, "|\n").unwrap();

        for i in 0..width {
            write!(
                tx, 
                "|\x1b[93m{sig:>4}\x1b[0m : \x1b[91m{amb:<4}\x1b[0m", 
                sig=results.signal_per_spad[width*j+i], 
                amb=results.ambient_per_spad[width*j+i]
            ).unwrap();
        } write!(tx, "|\n").unwrap();
    }
    for _ in 0..width { write!(tx, "+-----------").unwrap(); } writeln!(tx, "+").unwrap();

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
    let mut pwr_pin= gpioa.pa7.into_push_pull_output();
    let scl: Pin<'B', 8> = gpiob.pb8;
    let sda: Pin<'B', 9> = gpiob.pb9;
    let lpn_pin= gpiob.pb0.into_push_pull_output();
    let tx_pin = gpioa.pa2.into_alternate();
    let delay = cp.SYST.delay(&clocks);
    
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
        Mode::Standard { frequency:  400.kHz() },
        &clocks);

    write_results(&mut tx, &results, width);

    let i2c_bus = RefCell::new(i2c);
    let address: SevenBitAddress = VL53L8CX_DEFAULT_I2C_ADDRESS;
    let i2c_rst_pin = -1;
    let mut sensor = 
        Vl53l8cx::new_i2c(
            i2c::RefCellDevice::new(&i2c_bus), 
            address, 
            lpn_pin, 
            i2c_rst_pin, 
            delay
        ).unwrap();

    pwr_pin.set_high();

    sensor.begin().unwrap();
    
    sensor.init_sensor(VL53L8CX_DEFAULT_I2C_ADDRESS).unwrap();
    
    sensor.start_ranging().unwrap();
    
    // let mut ready: u8 = 0;
    
    loop {
        // loop {  
        //     if ready != 0 { break; }
        //     ready = sensor.check_data_ready().unwrap();
        // }
        // results = sensor.get_ranging_data().unwrap();
        // write_results(&mut tx, &results, width);
        // sensor.delay(1000);
    }
}


