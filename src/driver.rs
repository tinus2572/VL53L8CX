
use consts::*;
use buffers::*;

use stm32f4xx_hal::prelude::*;

use crate::{consts, buffers, BusOperation, bitfield, MotionIndicator, SysDelay, Output, PushPull, Pin};

bitfield! {
    struct BlockHeader(u32);
    bh_bytes, _: 31, 0;
    bh_idx, set_bh_idx: 31, 15;
    bh_size, set_bh_size: 15, 4;
    bh_type, set_bh_type: 4, 0;
}

pub struct Vl53l8cx<B: BusOperation> {
    pub temp_buffer: [u8;  VL53L8CX_TEMPORARY_BUFFER_SIZE],
    pub offset_data: [u8;  VL53L8CX_OFFSET_BUFFER_SIZE],
    pub xtalk_data: [u8; VL53L8CX_XTALK_BUFFER_SIZE],
    pub streamcount: u8,
    pub data_read_size: u32,
    pub is_auto_stop_enabled: u8,

    pub lpn_pin: Pin<'B', 0, Output<PushPull>>,
    pub i2c_rst_pin: i8,
    
    pub bus: B,
    pub delay: SysDelay
}

#[derive(Copy, Clone, Debug)]
pub enum Error<B> {
    Bus(B),
    Other,
    TimeoutPollForAnswer
}

#[repr(C)]
pub struct ResultsData {
    pub silicon_temp_degc: i8,
    pub ambient_per_spad: [u32; VL53L8CX_RESOLUTION_8X8 as usize],
    pub nb_target_detected: [u8; VL53L8CX_RESOLUTION_8X8 as usize],
    pub nb_spads_enabled: [u32; VL53L8CX_RESOLUTION_8X8 as usize],
    pub signal_per_spad: [u32; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
    pub range_sigma_mm: [u16; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
    pub distance_mm: [i16; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
    pub reflectance: [u8; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
    pub target_status: [u8; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
    pub motion_indicator: MotionIndicator
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

fn from_u8_to_u16(src: &[u8], dst: &mut[u16]) {
    for (i, chunk) in src.chunks(2).enumerate() {
        dst[i] = (chunk[0] as u16) | (chunk[1] as u16) << 8;
    }
}
fn from_u8_to_u32(src: &[u8], dst: &mut[u32]) {
    for (i, chunk) in src.chunks(4).enumerate() {
        dst[i] = (chunk[0] as u32) | (chunk[1] as u32) << 16 | (chunk[2] as u32) << 24 | (chunk[3] as u32) << 24;
    }
}
fn from_u16_to_u8(src: &[u16], dst: &mut[u8]) {
    for (i, &num) in src.iter().enumerate() {
        dst[i*2..(i+1)*2].copy_from_slice(&num.to_le_bytes()); 
    }
}
fn from_u32_to_u8(src: &[u32], dst: &mut[u8]) {
    for (i, &num) in src.iter().enumerate() {
        dst[i*4..(i+1)*4].copy_from_slice(&num.to_le_bytes()); 
    }
}

impl<B: BusOperation> Vl53l8cx<B> {

    fn poll_for_answer(&mut self, size: usize, pos: u8, reg: u16, mask: u8, expected_val: u8) -> Result<(), Error<B::Error>> {
        let mut timeout: u8 = 0;

        while timeout <= 200 {
            self.read_from_register_to_temp_buffer(reg, size)?;
            self.delay(10);
            
            if size >= 4 && self.temp_buffer[2] >= 0x7F {
                return Err(Error::Other);
            }
            if self.temp_buffer[pos as usize] & mask == expected_val {
                return Ok(());
            }
            timeout+=1;
        }
        Err(Error::TimeoutPollForAnswer)
    }

    fn poll_for_mcu_boot(&mut self) -> Result<(), Error<B::Error>> {
        let mut go2_status0: [u8; 1] = [0];
        let mut go2_status1: [u8; 1] = [0];
        let mut timeout: u16 = 0;

        while timeout <= 500 {
            self.read_from_register(0x06, &mut go2_status0)?;
            if go2_status0[0] & 0x80 != 0 {
                self.read_from_register(0x07, &mut go2_status1)?;
                if go2_status1[0] & 0x01 != 0 {
                    return Ok(());
                }
            }
            self.delay(1);
            if go2_status0[0] & 0x01 != 0 {
                return Ok(());
            }
            timeout += 1;
        }
        Err(Error::TimeoutPollForAnswer)
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
            from_u8_to_u32(&mut self.temp_buffer[0x3c..0x3c+256], &mut signal_grid);
            from_u8_to_u16(&mut self.temp_buffer[0x140..0x140+128], &mut range_grid);
            
            for j in 0..4 {
                for i in 0..4 {
                    signal_grid[i + (4 * j)] = signal_grid[(2 * i) + (16 * j) + 0]/4 + signal_grid[(2 * i) + (16 * j) + 1]/4 + signal_grid[(2 * i) + (16 * j) + 8]/4 + signal_grid[(2 * i) + (16 * j) + 9]/4;
                    range_grid[i + (4 * j)] = range_grid[(2 * i) + (16 * j) + 0]/4 + range_grid[(2 * i) + (16 * j) + 1]/4 + range_grid[(2 * i) + (16 * j) + 8]/4 + range_grid[(2 * i) + (16 * j) + 9]/4;
                }
            }
            from_u32_to_u8(&mut signal_grid, &mut self.temp_buffer[0x3c..0x3c+256]);
            from_u16_to_u8(&mut range_grid, &mut self.temp_buffer[0x140..0x140+128]);

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
            from_u8_to_u32(&mut self.temp_buffer[0x34..0x34+256], &mut signal_grid);

            for j in 0..4 {
                for i in 0..4 {
                    signal_grid[i + (4 * j)] = signal_grid[(2 * i) + (16 * j) + 0]/4 + signal_grid[(2 * i) + (16 * j) + 1]/4 + signal_grid[(2 * i) + (16 * j) + 8]/4 + signal_grid[(2 * i) + (16 * j) + 9]/4;
                }
            }

            from_u32_to_u8(&mut signal_grid, &mut self.temp_buffer[0x34..0x34+256]);
            
            self.swap_temp_buffer(VL53L8CX_XTALK_BUFFER_SIZE)?;
            self.temp_buffer[0x134..0x134+profile_4x4.len()].copy_from_slice(&profile_4x4);
            let tmp: [u8; 4] = [0; 4];
            self.temp_buffer[0x078..0x078+4].copy_from_slice(&tmp);
        }

        self.write_multi_to_register_temp_buffer(0x2CF8, VL53L8CX_XTALK_BUFFER_SIZE)?;
        self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xFF, 0x03)?;

        Ok(())
    }  

    fn swap_buffer(&mut self, buf: &mut [u8], size: usize) -> Result<(), Error<B::Error>> {
        for chunk in buf[..size].chunks_exact_mut(4) {
            let tmp: u32 = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            chunk.copy_from_slice(&tmp.to_le_bytes());
        }

        Ok(())
    }

    fn swap_temp_buffer(&mut self, size: usize) -> Result<(), Error<B::Error>> {
        for chunk in self.temp_buffer[..size].chunks_exact_mut(4) {
            let tmp: u32 = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            chunk.copy_from_slice(&tmp.to_le_bytes());
        }

        Ok(())
    }

    #[inline]
    pub fn read_from_register(&mut self, reg: u16, rbuf: &mut [u8]) -> Result<(), Error<B::Error>> {
        let size = rbuf.len();
        let mut read_size: usize;

        for i in (0..size).step_by(DEFAULT_I2C_BUFFER_LEN) {
            read_size = if size - i > DEFAULT_I2C_BUFFER_LEN { DEFAULT_I2C_BUFFER_LEN } else { size - i };
            let a: u8 = (reg + i as u16 >> 8) as u8;
            let b: u8 = (reg + i as u16 & 0xFF) as u8; 
            self.bus.write_read(&[a, b], &mut rbuf[i..i+read_size]).map_err(Error::Bus)?;
        }
        Ok(())
    }
    
    #[inline]
    pub fn read_from_register_to_temp_buffer(&mut self, reg: u16, size: usize) -> Result<(), Error<B::Error>> {
            let mut read_size: usize;
            for i in (0..size).step_by(DEFAULT_I2C_BUFFER_LEN) {
                read_size = if size - i > DEFAULT_I2C_BUFFER_LEN { DEFAULT_I2C_BUFFER_LEN } else { size - i };
                let a: u8 = (reg + i as u16 >> 8) as u8;
                let b: u8 = (reg + i as u16 & 0xFF) as u8; 
                self.bus.write_read(&[a, b], &mut self.temp_buffer[i..i+read_size]).map_err(Error::Bus)?;
            }
        Ok(())
    }

    #[inline]
    pub fn write_to_register(&mut self, reg: u16, val: u8) -> Result<(), Error<B::Error>> {
        let a: u8 = (reg >> 8) as u8;
        let b: u8 = (reg & 0xFF) as u8; 
        self.bus.write(&[a, b, val]).map_err(Error::Bus)?;
       
        Ok(())
    }
    
    #[inline]
    #[allow(dead_code)]
    pub fn write_to_register_check(&mut self, reg: u16, val: u8) -> Result<(), Error<B::Error>> {
        let a: u8 = (reg >> 8) as u8;
        let b: u8 = (reg & 0xFF) as u8; 
        self.bus.write(&[a, b, val]).map_err(Error::Bus)?;
        
        // Check
        let mut arr: [u8; 1] = [0];
        self.read_from_register(reg, &mut arr)?;
        if arr[0] != val {
            return Err(Error::Other);
        }
        Ok(())
    }

    #[inline]
    pub fn write_multi_to_register(&mut self, reg: u16, wbuf: &[u8]) -> Result<(), Error<B::Error>> {
        let size = wbuf.len();
        let mut write_size: usize;
        let mut tmp: [u8; DEFAULT_I2C_BUFFER_LEN] = [0; DEFAULT_I2C_BUFFER_LEN];

        for i in (0..size).step_by(DEFAULT_I2C_BUFFER_LEN-2) {
            write_size = if size - i > DEFAULT_I2C_BUFFER_LEN-2 { DEFAULT_I2C_BUFFER_LEN-2 } else { size - i };
            tmp[0] = (reg + i as u16 >> 8) as u8;
            tmp[1] = (reg + i as u16 & 0xFF) as u8;
            tmp[2..2+write_size].copy_from_slice(&wbuf[i..i+write_size]);
            self.bus.write(&tmp[..2+write_size]).map_err(Error::Bus)?;    
        }   
        Ok(())
    }
    
    #[inline]
    #[allow(dead_code)]
    pub fn write_multi_to_register_check(&mut self, reg: u16, wbuf: &[u8]) -> Result<(), Error<B::Error>> {
        let size = wbuf.len();
        let mut write_size: usize;
        let mut arr: [u8; DEFAULT_I2C_BUFFER_LEN] = [0; DEFAULT_I2C_BUFFER_LEN];
        let mut tmp: [u8; DEFAULT_I2C_BUFFER_LEN] = [0; DEFAULT_I2C_BUFFER_LEN];
        
        for i in (0..size).step_by(DEFAULT_I2C_BUFFER_LEN-2) {
            write_size = if size - i > DEFAULT_I2C_BUFFER_LEN-2 { DEFAULT_I2C_BUFFER_LEN-2 } else { size - i };
            tmp[0] = (reg + i as u16 >> 8) as u8;
            tmp[1] = (reg + i as u16 & 0xFF) as u8;
            tmp[2..2+write_size].copy_from_slice(&wbuf[i..i+write_size]);
            self.bus.write(&tmp[..2+write_size]).map_err(Error::Bus)?;   
            
            // Check
            self.read_from_register(reg + i as u16, &mut arr[..write_size])?;
            if arr[..write_size] != wbuf[i..i+write_size] {
                return Err(Error::Other);
            }    
        }   
        Ok(())
    }
    
    #[inline]
    pub fn write_multi_to_register_temp_buffer(&mut self, reg: u16, size: usize) -> Result<(), Error<B::Error>> {       
        let mut write_size: usize;
        let mut tmp: [u8; DEFAULT_I2C_BUFFER_LEN] = [0; DEFAULT_I2C_BUFFER_LEN];

        for i in (0..size).step_by(DEFAULT_I2C_BUFFER_LEN-2) {
            write_size = if size - i > DEFAULT_I2C_BUFFER_LEN-2 { DEFAULT_I2C_BUFFER_LEN-2 } else { size - i };
            tmp[0] = (reg + i as u16 >> 8) as u8;
            tmp[1] = (reg + i as u16 & 0xFF) as u8;
            tmp[2..2+write_size].copy_from_slice(&self.temp_buffer[i..i+write_size]);
            self.bus.write(&tmp[..2+write_size]).map_err(Error::Bus)?;   
        }
        Ok(())
    } 
    
    #[inline]
    #[allow(dead_code)]
    pub fn write_multi_to_register_temp_buffer_check(&mut self, reg: u16, size: usize) -> Result<(), Error<B::Error>> {       
        let mut write_size: usize;
        let mut arr: [u8; DEFAULT_I2C_BUFFER_LEN] = [0; DEFAULT_I2C_BUFFER_LEN];
        let mut tmp: [u8; DEFAULT_I2C_BUFFER_LEN] = [0; DEFAULT_I2C_BUFFER_LEN];
        
        for i in (0..size).step_by(DEFAULT_I2C_BUFFER_LEN-2) {
            write_size = if size - i > DEFAULT_I2C_BUFFER_LEN-2 { DEFAULT_I2C_BUFFER_LEN-2 } else { size - i };
            tmp[0] = (reg + i as u16 >> 8) as u8;
            tmp[1] = (reg + i as u16 & 0xFF) as u8;
            tmp[2..2+write_size].copy_from_slice(&self.temp_buffer[i..i+write_size]);
            self.bus.write(&tmp[..2+write_size]).map_err(Error::Bus)?;   
            
            // Check
            self.read_from_register(reg + i as u16, &mut arr[..write_size])?;
            if arr[..write_size] != self.temp_buffer[i..i+write_size] {
                return Err(Error::Other);
            }    
        }  
        Ok(())
    }

    #[inline]
    pub fn delay(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }

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

    #[allow(dead_code)]
    pub fn i2c_reset(&mut self) -> Result<(), Error<B::Error>>{
        // self.i2c_rst_pin.set_low();
        self.delay(10);
        // self.i2c_rst_pin.set_high();
        self.delay(10);
        // self.i2c_rst_pin.set_low();
        self.delay(10);
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
   
    #[allow(dead_code)]
    pub fn dci_read_data(&mut self, data: &mut [u8], index: u16, data_size: usize) -> Result<(), Error<B::Error>> {
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
    
    #[allow(dead_code)]
    fn dci_write_data_u16(&mut self, data: &mut [u16], buf: &mut[u8], index: u16, data_size: usize)  -> Result<(), Error<B::Error>> {
        from_u16_to_u8(data, buf);
        self.dci_write_data(buf, index, data_size)?;
        from_u8_to_u16(buf, data);
        Ok(())
    }
    fn dci_write_data_u32(&mut self, data: &mut [u32], buf: &mut[u8], index: u16, data_size: usize)  -> Result<(), Error<B::Error>> {
        from_u32_to_u8(data, buf);
        self.dci_write_data(buf, index, data_size)?;
        from_u8_to_u32(buf, data);
        Ok(())
    }

    pub fn dci_read_data_temp_buffer(&mut self, index: u16, data_size: usize) -> Result<(), Error<B::Error>> {
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
    
    pub fn dci_write_data(&mut self, data: &mut [u8], index: u16, data_size: usize) -> Result<(), Error<B::Error>> {
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
                self.temp_buffer[data_size-1 - i + 4] = data[data_size-1 - i];
            }

            /* Add headers and footer */
            self.temp_buffer[..headers.len()].copy_from_slice(&headers);
            self.temp_buffer[data_size+4..data_size+4+footer.len()].copy_from_slice(&footer);

            /* Send data to FW */
            self.write_multi_to_register_temp_buffer(address, data_size + 12)?;
            self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xff, 0x03)?;

            self.swap_buffer(data, data_size)?;
        }

        Ok(())
    }
    
    pub fn dci_write_data_temp_buffer(&mut self, index: u16, data_size: usize) -> Result<(), Error<B::Error>> {
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
            self.temp_buffer[data_size+4..data_size+4+footer.len()].copy_from_slice(&footer);

            /* Send data to FW */
            self.write_multi_to_register_temp_buffer(address, data_size + 12)?;
            self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xff, 0x03)?;

            self.swap_temp_buffer(data_size)?;
        }

        Ok(())
    }   

    #[allow(dead_code)]
    pub fn dci_replace_data(&mut self, data: &mut [u8], index: u16, data_size: usize, new_data: &[u8], new_data_size: usize, new_data_pos: usize) -> Result<(), Error<B::Error>> {
        self.dci_read_data(data, index, data_size)?;
        data[new_data_pos..].copy_from_slice(&new_data[..new_data_size]);
        self.dci_write_data(data, index, data_size)?;
        
        Ok(())
    }   
    
    pub fn dci_replace_data_temp_buffer(&mut self, index: u16, data_size: usize, new_data: &[u8], new_data_size: usize, new_data_pos: usize) -> Result<(), Error<B::Error>> {
        self.dci_read_data_temp_buffer(index, data_size)?;
        self.temp_buffer[new_data_pos..].copy_from_slice(&new_data[..new_data_size]);
        self.dci_write_data_temp_buffer(index, data_size)?;
        
        Ok(())
    }   

    pub fn init(&mut self) -> Result<(), Error<B::Error>> {
        let mut tmp: [u8; 1] = [0];    
        let mut pipe_ctrl: [u8; 4] = [VL53L8CX_NB_TARGET_PER_ZONE as u8, 0x00, 0x01, 0x00];
        let mut single_range: [u8; 4] = [0,0,0,0x01];

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
        self.dci_write_data(&mut pipe_ctrl, VL53L8CX_DCI_PIPE_CONTROL, 4)?;
      
        if VL53L8CX_NB_TARGET_PER_ZONE != 1 {
            tmp[0] = VL53L8CX_NB_TARGET_PER_ZONE as u8;
            self.dci_replace_data_temp_buffer(VL53L8CX_DCI_FW_NB_TARGET, 16, &mut tmp, 1, 0x0C)?;
        }
      
        self.dci_write_data(&mut single_range, VL53L8CX_DCI_SINGLE_RANGE, 4)?;
      

        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_power_mode(&mut self) -> Result<u8, Error<B::Error>> {
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
    
    #[allow(dead_code)]
    pub fn set_power_mode(&mut self, power_mode: u8) -> Result<(), Error<B::Error>> {
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

    #[allow(dead_code)]
    pub fn get_resolution(&mut self) -> Result<u8, Error<B::Error>> {
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_ZONE_CONFIG, 8)?;
        let resolution: u8 = self.temp_buffer[0x00] * self.temp_buffer[0x01];

        Ok(resolution)
    }

    #[allow(dead_code)]
    pub fn set_resolution(&mut self, resolution: u8) -> Result<(), Error<B::Error>> {

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

    pub fn start_ranging(&mut self) -> Result<(), Error<B::Error>> {
        let resolution: u8 = self.get_resolution()?;
        let mut tmp: [u16; 1] = [0];
        let mut header_config: [u32; 2] = [0, 0];
        let cmd: [u8; 4] = [0x00, 0x03, 0x00, 0x00];

        self.temp_buffer = [0; VL53L8CX_TEMPORARY_BUFFER_SIZE];
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
        for i in 0..12 {
            // if output[i] == 0 || output_bh_enable[0] & 1 << i == 0 {
            if output[i] == 0 || output_bh_enable[i/32] & (1 << (i%32)) == 0 {
                continue;
            }
            bh = BlockHeader(output[i]);
            if bh.bh_type() >= 0x01 && bh.bh_type() < 0x0d {
                if bh.bh_idx() >= 0x54d0 && bh.bh_idx() < 0x54d0 + 960 {
                    bh.set_bh_size(resolution as u32);} 
                else {
                    bh.set_bh_size(resolution as u32 * VL53L8CX_NB_TARGET_PER_ZONE);}
                self.data_read_size += bh.bh_type() * bh.bh_size();} 
            else {
                self.data_read_size += bh.bh_size();}
            self.data_read_size += 4;
        }
        self.data_read_size += 24;

        self.dci_write_data_u32(&mut output, &mut [0; 48], VL53L8CX_DCI_OUTPUT_LIST, 48)?;
        
        header_config[0] = self.data_read_size;
        header_config[1] = 12+1 as u32;
        self.dci_write_data_u32(&mut header_config, &mut [0; 8], VL53L8CX_DCI_OUTPUT_CONFIG, 8)?;

        self.dci_write_data_u32(&mut output_bh_enable, &mut [0; 16], VL53L8CX_DCI_OUTPUT_ENABLES, 16)?;
        
        /* Start xshut bypass (interrupt mode) */
        self.write_to_register(0x7fff, 0x00)?;
        self.write_to_register(0x09, 0x05)?;
        self.write_to_register(0x7fff, 0x02)?;

        /* Start ranging session */
        self.write_multi_to_register_check(VL53L8CX_UI_CMD_END - (4-1), &cmd)?;
        self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xff, 0x03)?;

        /* Read ui range data content and compare if data size is the correct one */
        self.dci_read_data_temp_buffer(0x5440, 12)?;
        from_u8_to_u16(&self.temp_buffer[0x8..0x8+2], &mut tmp);
        if tmp[0] != self.data_read_size as u16 {       
            return Err(Error::Other);
        }   

        /* Ensure that there is no laser safety fault */
        self.dci_read_data_temp_buffer(0xe0c4, 8)?;
        if self.temp_buffer[0x6] != 0 {
            return Err(Error::Other);
        }

        Ok(())
    }

    #[allow(dead_code)]
    pub fn stop_ranging(&mut self) -> Result<(), Error<B::Error>> {
        let mut timeout: u16 = 0;
        let mut auto_flag_stop: [u32; 1] = [0];
        let mut buf: [u8; 4] = [0; 4];
        let mut tmp: [u8; 1] = [0];

        self.read_from_register(0x2ffc, &mut buf)?;
        from_u8_to_u32(&buf, &mut auto_flag_stop);

        if auto_flag_stop[0] != 0x4ff {
            self.write_to_register(0x7fff, 0x00)?;

            /* Provoke MCU stop */
            self.write_to_register(0x15, 0x16)?;
            self.write_to_register(0x14, 0x01)?;

            /* Poll for G02 status 0 MCU stop */
            while tmp[0] & (0x80 as u8) >> 7 == 0x00 && timeout <= 500{
                self.read_from_register(0x6, &mut tmp)?;
                self.delay(10);

                timeout += 1;
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
    
    #[allow(dead_code)]
    pub fn get_external_sync_pin_enable(&mut self) -> Result<u8, Error<B::Error>> {
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

    #[allow(dead_code)]
    pub fn set_external_sync_pin_enable(&mut self, enable_sync_pin: u8) -> Result<(), Error<B::Error>> {
        let mut tmp: [u32; 1] = [0];
        self.read_from_register_to_temp_buffer(VL53L8CX_DCI_SYNC_PIN, 4)?;
        from_u8_to_u32(&self.temp_buffer[3..3+4], &mut tmp);
        
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

    #[allow(dead_code)]
    pub fn get_target_order(&mut self) -> Result<u8, Error<B::Error>> {
        let target_order: u8;
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_TARGET_ORDER, 4)?;
        target_order = self.temp_buffer[0];

        Ok(target_order)
    }

    #[allow(dead_code)]
    pub fn set_target_order(&mut self, target_order: u8) -> Result<(), Error<B::Error>> {
        if target_order == VL53L8CX_TARGET_ORDER_CLOSEST || target_order == VL53L8CX_TARGET_ORDER_STRONGEST {
            self.dci_replace_data_temp_buffer(VL53L8CX_DCI_TARGET_ORDER, 4, &[target_order], 1, 0x0)?;
        } else {
            return Err(Error::Other);
        }
        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_sharpener_percent(&mut self) -> Result<u8, Error<B::Error>> {
        let sharpener_percent: u8;
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_SHARPENER, 16)?;
        sharpener_percent = self.temp_buffer[0xD] * 100 / 255;

        Ok(sharpener_percent)
    }

    #[allow(dead_code)]
    pub fn set_sharpener_percent(&mut self, sharpener_percent: u8) -> Result<(), Error<B::Error>> {
        let sharpener: u8;
        if sharpener_percent >= 100 {
            return Err(Error::Other);
        }
        sharpener = sharpener_percent * 255 / 100;
        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_SHARPENER, 16, &[sharpener], 1, 0xd)?;

        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_integration_time(&mut self) -> Result<u32, Error<B::Error>> {
        let mut time_ms: [u32; 1] = [0];
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_INT_TIME, 20)?;
        from_u8_to_u32(&self.temp_buffer[..4], &mut time_ms);
        
        time_ms[0] /= 1000;
        
        Ok(time_ms[0])
    }

    #[allow(dead_code)]
    pub fn set_integration_time(&mut self, integration_time_ms: u32) -> Result<(), Error<B::Error>> {
        let mut integration: u32 = integration_time_ms;

        /* Integration time must be between 2ms and 1000ms */
        if integration < 2 || integration > 1000 {
            return Err(Error::Other);
        }
        integration *= 1000;
        
        let mut buf: [u8; 4] = [0; 4];
        let tmp: [u32; 1] = [integration];
        from_u32_to_u8(&tmp, &mut buf);
        
        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_INT_TIME, 20, &buf, 4, 0x00)?;

        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_ranging_mode(&mut self) -> Result<u8, Error<B::Error>> {
        let ranging_mode: u8;
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_RANGING_MODE, 8)?;
        if self.temp_buffer[1] == 1 {
            ranging_mode = VL53L8CX_RANGING_MODE_CONTINUOUS;
        } else {
            ranging_mode = VL53L8CX_RANGING_MODE_AUTONOMOUS;
        }
        Ok(ranging_mode)
    }

    #[allow(dead_code)]
    pub fn set_ranging_mode(&mut self, ranging_mode: u8) -> Result<(), Error<B::Error>> {
        let mut single_range: [u32; 1] = [0];
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_RANGING_MODE, 8)?;

        if ranging_mode == VL53L8CX_RANGING_MODE_CONTINUOUS {
            self.temp_buffer[1] = 1;
            self.temp_buffer[3] = 3;
            single_range[0] = 0;
        } else if ranging_mode == VL53L8CX_RANGING_MODE_CONTINUOUS {
            self.temp_buffer[1] = 3;
            self.temp_buffer[3] = 2;
            single_range[0] = 1;
        } else {
            return Err(Error::Other);
        }

        self.dci_write_data_temp_buffer(VL53L8CX_DCI_RANGING_MODE, 8)?;

        self.dci_write_data_u32(&mut single_range, &mut [0; 4], VL53L8CX_DCI_SINGLE_RANGE, 4)?;
        
        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_frequency_hz(&mut self) -> Result<u8, Error<B::Error>> {
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_FREQ_HZ, 4)?;
        let frequency_hz: u8 = self.temp_buffer[0x01];

        Ok(frequency_hz)
    }
    
    #[allow(dead_code)]
    pub fn set_frequency_hz(&mut self, frequency_hz: u8) -> Result<(), Error<B::Error>> {
        let tmp: [u8; 1] = [frequency_hz];
        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_FREQ_HZ, 4, &tmp, 1, 0x01)?;

        Ok(())
    }

    pub fn check_data_ready(&mut self) -> Result<u8, Error<B::Error>> {
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

    pub fn get_ranging_data(&mut self) -> Result<ResultsData, Error<B::Error>> {
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
        while i < self.data_read_size as usize {

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

            let src = &self.temp_buffer[i+4..i+4+msize as usize];

            if bh.bh_idx() == VL53L8CX_METADATA_IDX as u32 {
                result.silicon_temp_degc = self.temp_buffer[i+12] as i8;
            } else if VL53L8CX_DISABLE_AMBIENT_PER_SPAD == 0 && bh.bh_idx() == VL53L8CX_AMBIENT_RATE_IDX as u32 {
                from_u8_to_u32(src, &mut result.ambient_per_spad);
            
            } else if VL53L8CX_DISABLE_NB_SPADS_ENABLED == 0 && bh.bh_idx() == VL53L8CX_SPAD_COUNT_IDX as u32 {
                from_u8_to_u32(src, &mut result.nb_spads_enabled);

            } else if VL53L8CX_DISABLE_NB_TARGET_DETECTED == 0 && bh.bh_idx() == VL53L8CX_NB_TARGET_DETECTED_IDX as u32 {
                result.nb_target_detected.copy_from_slice(src); 
                
            } else if VL53L8CX_DISABLE_SIGNAL_PER_SPAD == 0 && bh.bh_idx() == VL53L8CX_SIGNAL_RATE_IDX as u32 {
                from_u8_to_u32(src, &mut result.signal_per_spad);
                
            } else if VL53L8CX_DISABLE_RANGE_SIGMA_MM == 0 && bh.bh_idx() == VL53L8CX_RANGE_SIGMA_MM_IDX as u32 {
                from_u8_to_u16(src, &mut result.range_sigma_mm);
                
            } else if VL53L8CX_DISABLE_DISTANCE_MM == 0 && bh.bh_idx() == VL53L8CX_DISTANCE_IDX as u32 {
                for (i, chunk) in self.temp_buffer[i+4..i+4+msize as usize].chunks(2).enumerate() {
                    result.distance_mm[i] = (chunk[0] as i16) << 8 | (chunk[1] as i16);
                }
            } else if VL53L8CX_DISABLE_REFLECTANCE_PERCENT == 0 && bh.bh_idx() == VL53L8CX_REFLECTANCE_EST_PC_IDX as u32 {
                result.reflectance.copy_from_slice(src); 

            } else if VL53L8CX_DISABLE_TARGET_STATUS == 0 && bh.bh_idx() == VL53L8CX_TARGET_STATUS_IDX as u32 {
                result.target_status.copy_from_slice(src); 

            } else if VL53L8CX_DISABLE_MOTION_INDICATOR == 0 && bh.bh_idx() == VL53L8CX_MOTION_DETEC_IDX as u32 {
                let ptr: *const MotionIndicator = src.as_ptr() as *const MotionIndicator;
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