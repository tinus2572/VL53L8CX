#![no_std]

pub mod accessors;
pub mod buffers;
pub mod bus_operation;
pub mod consts;
pub mod detection_thresholds;
pub mod motion_indicator;
pub mod utils;
pub mod xtalk;

#[allow(unused_imports)]
use accessors::*;
use buffers::*;
use bus_operation::*;
use consts::*;
#[allow(unused_imports)]
use detection_thresholds::*;
use motion_indicator::*;
use utils::*;
#[allow(unused_imports)]
use xtalk::*;

use embedded_hal::{
    i2c::{I2c, SevenBitAddress},
    spi::{SpiDevice, Operation},
    digital::OutputPin, 
    delay::DelayNs
};

use bitfield::bitfield;

bitfield! {
    pub struct BlockHeader(u32);
    pub bh_bytes, _: 31, 0;
    pub bh_idx, set_bh_idx: 31, 16;
    pub bh_size, set_bh_size: 15, 4;
    pub bh_type, set_bh_type: 3, 0;
}

pub struct Vl53l8cx<B: BusOperation, LPN: OutputPin, T: DelayNs> {
    pub temp_buffer: [u8;  VL53L8CX_TEMPORARY_BUFFER_SIZE],
    pub offset_data: [u8;  VL53L8CX_OFFSET_BUFFER_SIZE],
    pub xtalk_data: [u8; VL53L8CX_XTALK_BUFFER_SIZE],
    pub streamcount: u8,
    pub data_read_size: u32,
    pub is_auto_stop_enabled: bool,

    pub lpn_pin: LPN,
    
    pub chunk_size: usize,
    pub bus: B,
    pub tim: T
}

#[derive(Copy, Clone, Debug)]
pub enum Error<B> {
    Bus(B),
    Other,
    Timeout,
    Mcu,
    Go2,
    CorruptedFrame,
    InvalidParam,
    CheckSumFail
}

/**
 * @brief Structure ResultsData contains the ranging results of
 * VL53L8CX. If user wants more than 1 target per zone, the results can be split
 * into 2 sub-groups :
 * - Per zone results. These results are common to all targets (ambient_per_spad
 * , nb_target_detected and nb_spads_enabled).
 * - Per target results : These results are different relative to the detected
 * target (signal_per_spad, range_sigma_mm, distance_mm, reflectance,
 * target_status).
 */
#[repr(C)]
pub struct ResultsData {
  /* Internal sensor silicon temperature */
    pub silicon_temp_degc: i8, 
    #[cfg(not(feature="VL53L8CX_DISABLE_AMBIENT_PER_SPAD"))]
  /* Ambient noise in kcps/spads */
  pub ambient_per_spad: [u32; VL53L8CX_RESOLUTION_8X8 as usize],
    #[cfg(not(feature="VL53L8CX_DISABLE_NB_TARGET_DETECTED"))]
  /* Number of valid target detected for 1 zone */
  pub nb_target_detected: [u8; VL53L8CX_RESOLUTION_8X8 as usize],
    #[cfg(not(feature="VL53L8CX_DISABLE_NB_SPADS_ENABLED"))]
  /* Number of spads enabled for this ranging */
    pub nb_spads_enabled: [u32; VL53L8CX_RESOLUTION_8X8 as usize],
    #[cfg(not(feature="VL53L8CX_DISABLE_SIGNAL_PER_SPAD"))]
  /* Signal returned to the sensor in kcps/spads */
    pub signal_per_spad: [u32; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
    #[cfg(not(feature="VL53L8CX_DISABLE_RANGE_SIGMA_MM"))]
  /* Sigma of the current distance in mm */
    pub range_sigma_mm: [u16; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
    #[cfg(not(feature="VL53L8CX_DISABLE_DISTANCE_MM"))]
  /* Measured distance in mm */
    pub distance_mm: [i16; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
    #[cfg(not(feature="VL53L8CX_DISABLE_REFLECTANCE_PERCENT"))]
  /* Estimated reflectance in percent */
    pub reflectance: [u8; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
    #[cfg(not(feature="VL53L8CX_DISABLE_TARGET_STATUS"))]
  /* Status indicating the measurement validity (5 & 9 means ranging OK)*/
    pub target_status: [u8; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
    #[cfg(not(feature="VL53L8CX_DISABLE_MOTION_INDICATOR"))]
  /* Motion detector results */
    pub motion_indicator: MotionIndicator
} 

impl ResultsData {
    pub fn new() -> Self {
        ResultsData {
            silicon_temp_degc: 0, 
            #[cfg(not(feature="VL53L8CX_DISABLE_AMBIENT_PER_SPAD"))]
            ambient_per_spad: [0; VL53L8CX_RESOLUTION_8X8 as usize],
            #[cfg(not(feature="VL53L8CX_DISABLE_NB_TARGET_DETECTED"))]
            nb_target_detected: [0; VL53L8CX_RESOLUTION_8X8 as usize],
            #[cfg(not(feature="VL53L8CX_DISABLE_NB_SPADS_ENABLED"))]
            nb_spads_enabled: [0; VL53L8CX_RESOLUTION_8X8 as usize],
            #[cfg(not(feature="VL53L8CX_DISABLE_SIGNAL_PER_SPAD"))]
            signal_per_spad: [0; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
            #[cfg(not(feature="VL53L8CX_DISABLE_RANGE_SIGMA_MM"))]
            range_sigma_mm: [0; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
            #[cfg(not(feature="VL53L8CX_DISABLE_DISTANCE_MM"))]
            distance_mm: [0; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
            #[cfg(not(feature="VL53L8CX_DISABLE_REFLECTANCE_PERCENT"))]
            reflectance: [0; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
            #[cfg(not(feature="VL53L8CX_DISABLE_TARGET_STATUS"))]
            target_status: [0; (VL53L8CX_RESOLUTION_8X8 as usize) * (VL53L8CX_NB_TARGET_PER_ZONE as usize)],
            #[cfg(not(feature="VL53L8CX_DISABLE_MOTION_INDICATOR"))]
            motion_indicator: MotionIndicator::new()
        }
    }
}

impl<B: BusOperation, LPN: OutputPin, T: DelayNs> Vl53l8cx<B, LPN, T> {
    /**
     * @brief Inner function, not available outside this file. This function is used
     * to wait for an answer from VL53L8CX sensor.
     */
    pub fn poll_for_answer(&mut self, size: usize, pos: u8, reg: u16, mask: u8, expected_val: u8) -> Result<(), Error<B::Error>> {
        let mut timeout: u8 = 0;

        while timeout <= 200 {
            self.read_from_register(reg, size)?;
            self.delay(10);
            
            if size >= 4 && self.temp_buffer[2] >= 0x7F {
                return Err(Error::Mcu);
            }
            if self.temp_buffer[pos as usize] & mask == expected_val {
                return Ok(());
            }
            timeout+=1;
        }
        Err(Error::Timeout)
    }

/*
 * Inner function, not available outside this file. This function is used to
 * wait for the MCU to boot.
 */
    pub fn poll_for_mcu_boot(&mut self) -> Result<(), Error<B::Error>> {
        let mut timeout: u16 = 0;

        while timeout <= 500 {
            self.read_from_register(0x06, 2)?;
            if self.temp_buffer[0] & 0x80 != 0 {
                if self.temp_buffer[1] & 0x01 != 0 {
                    return Ok(());
                }
            }
            self.delay(1);
            if self.temp_buffer[0] & 0x01 != 0 {
                return Ok(());
            }
            timeout += 1;
        }
        Err(Error::Timeout)
    }
/**
 * @brief Inner function, not available outside this file. This function is used
 * to set the offset data gathered from NVM.
 */
    pub fn send_offset_data(&mut self, resolution: u8) -> Result<(), Error<B::Error>> {
        let mut signal_grid: [u32; 64] = [0; 64];
        let mut range_grid: [i16; 64] = [0; 64];
        let dss_4x4: [u8; 8] = [0x0F, 0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07];
        let footer: [u8; 8] = [0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x01, 0xE4];

        self.temp_buffer[..VL53L8CX_OFFSET_BUFFER_SIZE].copy_from_slice(&self.offset_data);

        /* Data extrapolation is required for 4X4 offset */
        if resolution == VL53L8CX_RESOLUTION_4X4 {
            self.temp_buffer[0x10..0x10+dss_4x4.len()].copy_from_slice(&dss_4x4);
            swap_buffer(&mut self.temp_buffer, VL53L8CX_OFFSET_BUFFER_SIZE);
            from_u8_to_u32(&mut self.temp_buffer[0x3c..0x3c+256], &mut signal_grid);
            from_u8_to_i16(&mut self.temp_buffer[0x140..0x140+128], &mut range_grid);
            
            for j in 0..4 {
                for i in 0..4 {
                    signal_grid[i + (4 * j)] = ((
                          signal_grid[(2 * i) + (16 * j) + 0] as u64 
                        + signal_grid[(2 * i) + (16 * j) + 1] as u64 
                        + signal_grid[(2 * i) + (16 * j) + 8] as u64 
                        + signal_grid[(2 * i) + (16 * j) + 9] as u64 
                    ) /4) as u32;
                    range_grid[i + (4 * j)] = ((
                          range_grid[(2 * i) + (16 * j) + 0] as i32 
                        + range_grid[(2 * i) + (16 * j) + 1] as i32 
                        + range_grid[(2 * i) + (16 * j) + 8] as i32 
                        + range_grid[(2 * i) + (16 * j) + 9] as i32
                    ) /4) as i16;
                }
            }
            signal_grid[16..].copy_from_slice(&[0;48]);
            range_grid[16..].copy_from_slice(&[0;48]);

            from_u32_to_u8(&mut signal_grid, &mut self.temp_buffer[0x3c..0x3c+256]);
            from_i16_to_u8(&mut range_grid, &mut self.temp_buffer[0x140..0x140+128]);

            swap_buffer(&mut self.temp_buffer, VL53L8CX_OFFSET_BUFFER_SIZE);
        }

        for i in 0..VL53L8CX_OFFSET_BUFFER_SIZE-4 {
            self.temp_buffer[i] = self.temp_buffer[i+8];
        }

        self.temp_buffer[0x1E0..0x1E0+footer.len()].copy_from_slice(&footer);
        self.write_multi_to_register_temp_buffer(0x2E18, VL53L8CX_OFFSET_BUFFER_SIZE)?;
        self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xFF, 0x03)?;

        Ok(())
    }   
/**
 * @brief Inner function, not available outside this file. This function is used
 * to set the Xtalk data from generic configuration, or user's calibration.
 */
    pub fn send_xtalk_data(&mut self, resolution: u8) -> Result<(), Error<B::Error>> {
        let res4x4: [u8; 8] = [0x0F, 0x04, 0x04, 0x17, 0x08, 0x10, 0x10, 0x07];
        let dss_4x4: [u8; 8] = [0x00, 0x78, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08];
        let profile_4x4: [u8; 4] = [0xA0, 0xFC, 0x01, 0x00];
        let mut signal_grid: [u32; 64] = [0; 64];

        self.temp_buffer[..VL53L8CX_XTALK_BUFFER_SIZE].copy_from_slice(&self.xtalk_data);

        /* Data extrapolation is required for 4X4 Xtalk */
        if resolution == VL53L8CX_RESOLUTION_4X4 {
            self.temp_buffer[0x8..0x8 + res4x4.len()].copy_from_slice(&res4x4);
            self.temp_buffer[0x020..0x020 + dss_4x4.len()].copy_from_slice(&dss_4x4);

            swap_buffer(&mut self.temp_buffer, VL53L8CX_XTALK_BUFFER_SIZE);
            from_u8_to_u32(&mut self.temp_buffer[0x34..0x34+256], &mut signal_grid);

            for j in 0..4 {
                for i in 0..4 {
                    signal_grid[i + (4 * j)] = ((
                        signal_grid[(2 * i) + (16 * j) + 0] as u64 
                      + signal_grid[(2 * i) + (16 * j) + 1] as u64 
                      + signal_grid[(2 * i) + (16 * j) + 8] as u64 
                      + signal_grid[(2 * i) + (16 * j) + 9] as u64 
                  ) /4) as u32;
                }
            }
            signal_grid[16..].copy_from_slice(&[0;48]);
            from_u32_to_u8(&mut signal_grid, &mut self.temp_buffer[0x34..0x34+256]);

            swap_buffer(&mut self.temp_buffer, VL53L8CX_XTALK_BUFFER_SIZE);
            self.temp_buffer[0x134..0x134+profile_4x4.len()].copy_from_slice(&profile_4x4);
            self.temp_buffer[0x078..0x078+4].copy_from_slice(&[0; 4]);
        }

        self.write_multi_to_register_temp_buffer(0x2CF8, VL53L8CX_XTALK_BUFFER_SIZE)?;
        self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xFF, 0x03)?;

        Ok(())
    }  

    pub fn read_from_register(&mut self, reg: u16, size: usize) -> Result<(), Error<B::Error>> {
            let mut read_size: usize;
            for i in (0..size).step_by(CHUNK_SIZE) {
                read_size = if size - i > CHUNK_SIZE { CHUNK_SIZE } else { size - i };
                let a: u8 = (reg + i as u16 >> 8) as u8;
                let b: u8 = (reg + i as u16 & 0xFF) as u8; 
                self.bus.write_read(&[a, b], &mut self.temp_buffer[i..i+read_size]).map_err(Error::Bus)?;
            }
        Ok(())
    }

    pub fn write_to_register(&mut self, reg: u16, val: u8) -> Result<(), Error<B::Error>> {
        let a: u8 = (reg >> 8) as u8;
        let b: u8 = (reg & 0xFF) as u8; 
        self.bus.write(&[a, b, val]).map_err(Error::Bus)?;
       
        Ok(())
    }
    
    pub fn write_multi_to_register(&mut self, reg: u16, wbuf: &[u8]) -> Result<(), Error<B::Error>> {
        let size = wbuf.len();
        let mut write_size: usize;
        let mut tmp: [u8; CHUNK_SIZE] = [0; CHUNK_SIZE];
        for i in (0..size).step_by(CHUNK_SIZE-2) {
            write_size = if size - i > CHUNK_SIZE-2 { CHUNK_SIZE-2 } else { size - i };
            tmp[0] = (reg + i as u16 >> 8) as u8;
            tmp[1] = (reg + i as u16 & 0xFF) as u8;
            tmp[2..2+write_size].copy_from_slice(&wbuf[i..i+write_size]);
            self.bus.write(&tmp[..2+write_size]).map_err(Error::Bus)?;    
        }   
        Ok(())
    }
   
    pub fn write_multi_to_register_temp_buffer(&mut self, reg: u16, size: usize) -> Result<(), Error<B::Error>> {       
        let mut write_size: usize;
        let mut tmp: [u8; CHUNK_SIZE] = [0; CHUNK_SIZE];
        
        for i in (0..size).step_by(CHUNK_SIZE-2) {
            write_size = if size - i > CHUNK_SIZE-2 { CHUNK_SIZE-2 } else { size - i };
            tmp[0] = (reg + i as u16 >> 8) as u8;
            tmp[1] = (reg + i as u16 & 0xFF) as u8;
            tmp[2..2+write_size].copy_from_slice(&self.temp_buffer[i..i+write_size]);
            self.bus.write(&tmp[..2+write_size]).map_err(Error::Bus)?;   
        }
        Ok(())
    }

    pub fn delay(&mut self, ms: u32) {
        self.tim.delay_ms(ms);
    }

    pub fn on(&mut self) -> Result<(), Error<B::Error>>{
        self.lpn_pin.set_high().unwrap();
        self.delay(10);
        Ok(())
    }

    pub fn off(&mut self) -> Result<(), Error<B::Error>>{
        self.lpn_pin.set_low().unwrap();
        self.delay(10);
        Ok(())
    }
    
    /**
     * @brief Check if the VL53L8CX sensor is alive(responding to I2C communication).
    */
    pub fn is_alive(&mut self) -> Result<(), Error<B::Error>> {
        self.write_to_register(0x7fff, 0x00)?;
        self.read_from_register(0, 2)?;
        self.write_to_register(0x7fff, 0x02)?;
        let device_id: u8 = self.temp_buffer[0];
        let revision_id: u8 = self.temp_buffer[1];
        if (device_id != 0xF0) || (revision_id != 0x0C) {
            return Err(Error::Other);
        }

        Ok(())
    }
    
/**
 * @brief This function can be used to read 'extra data' from DCI. Using a known
 * index, the function fills the casted structure passed in argument.
 * Please note that the FW only accept data of 32 bits. So data can
 * only have a size of 32, 64, 96, 128, bits ....
 * @param (u16) index : Index of required value.
 * @param (usize)data_size : This field must be the structure or array size
 * (using sizeof() function).
 */
    pub fn dci_read_data(&mut self, index: u16, data_size: usize) -> Result<(), Error<B::Error>> {
        let read_size: usize = data_size + 12; 
        let mut cmd: [u8; 12] = [
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x0f,
            0x00, 0x02, 0x00, 0x08
        ];
        if read_size > VL53L8CX_TEMPORARY_BUFFER_SIZE {
            return Err(Error::Other);
        } 
        cmd[0] = (index >> 8) as u8;
        cmd[1] = (index & 0xff) as u8;
        cmd[2] = ((data_size & 0xff0) >> 4) as u8;
        cmd[3] = ((data_size & 0xf) << 4) as u8;
        
        /* Request data reading from FW */
        self.write_multi_to_register(VL53L8CX_UI_CMD_END - 11, &cmd)?;
        self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xFF, 0x03)?;
        
        /* Read new data sent (4 bytes header + data_size + 8 bytes footer) */
        self.read_from_register(VL53L8CX_UI_CMD_START, read_size)?;
        swap_buffer(&mut self.temp_buffer, read_size);
        
        /* Copy data from FW into input structure (-4 bytes to remove header) */
        for i in 0..data_size {
            self.temp_buffer[i] = self.temp_buffer[i+4];
        }
        
        Ok(())
    }   
    
/**
 * @brief This function can be used to write 'extra data' to DCI. The data can
 * be simple data, or casted structure.
 * Please note that the FW only accept data of 32 bits. So data can
 * only have a size of 32, 64, 96, 128, bits ..
 * @param (u16) index : Index of required value.
 * @param (usize)data_size : This field must be the structure or array size
 * (using sizeof() function).
 */
    pub fn dci_write_data(&mut self, index: u16, data_size: usize) -> Result<(), Error<B::Error>> {
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
            swap_buffer(&mut self.temp_buffer, data_size);
            for i in 0..data_size {
                self.temp_buffer[data_size-1 - i+4] = self.temp_buffer[data_size-1 - i];
            }

            /* Add headers and footer */
            self.temp_buffer[..headers.len()].copy_from_slice(&headers);
            self.temp_buffer[data_size+4..data_size+4+footer.len()].copy_from_slice(&footer);

            /* Send data to FW */
            self.write_multi_to_register_temp_buffer(address, data_size + 12)?;
            self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xff, 0x03)?;

            swap_buffer(&mut self.temp_buffer, data_size);
        }

        Ok(())
    }   
    
/**
 * @brief This function can be used to replace 'extra data' in DCI. The data can
 * be simple data, or casted structure.
 * Please note that the FW only accept data of 32 bits. So data can
 * only have a size of 32, 64, 96, 128, bits ..
 * @param (u16) index : Index of required value.
 * @param (usize) data_size : This field must be the structure or array size
 * (using sizeof() function).
 * @param (&[u8]) new_data : Contains the new fields.
 * @param (usize) new_data_size : New data size.
 * @param (usize) new_data_pos : New data position into the buffer.
 */
    pub fn dci_replace_data(&mut self, index: u16, data_size: usize, new_data: &[u8], new_data_size: usize, new_data_pos: usize) -> Result<(), Error<B::Error>> {
        self.dci_read_data(index, data_size)?;
        self.temp_buffer[new_data_pos..new_data_pos+new_data_size].copy_from_slice(&new_data[..new_data_size]);
        self.dci_write_data(index, data_size)?;
        
        Ok(())
    }   

/**
 * @brief Mandatory function used to initialize the sensor. This function must
 * be called after a power on, to load the firmware into the VL53L8CX. It takes
 * a few hundred milliseconds.
 */
    pub fn init(&mut self) -> Result<(), Error<B::Error>> {
        let pipe_ctrl: [u8; 4] = [VL53L8CX_NB_TARGET_PER_ZONE as u8, 0x00, 0x01, 0x00];
        let single_range: [u32; 1] = [0x01];
        let mut crc_checksum: [u32; 1] = [0];

        self.write_to_register(0x7fff, 0x00)?;
        self.write_to_register(0x0009, 0x04)?;
        self.write_to_register(0x000F, 0x40)?;
        self.write_to_register(0x000A, 0x03)?;
        self.read_from_register(0x7FFF, 1)?;
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

	    /* Wait for sensor booted (several ms required to get sensor ready ) */
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
        self.read_from_register(0x7fff, 1)?;
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
        self.read_from_register(0x7fff, 1)?;

        self.write_to_register(0x7fff, 0x01)?;
        // self.write_to_register(0x20, 0x07)?;
        // self.write_to_register(0x20, 0x06)?;

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
        self.read_from_register(0x7fff, 1)?;
        self.write_to_register(0x0C, 0x01)?;

        /* Reset MCU and wait boot */
        self.write_to_register(0x7FFF, 0x00)?;
        self.write_to_register(0x114, 0x00)?;
        self.write_to_register(0x115, 0x00)?;
        self.write_to_register(0x116, 0x42)?;
        self.write_to_register(0x117, 0x00)?;
        self.write_to_register(0x0B, 0x00)?;
        self.read_from_register(0x7fff, 1)?;
        self.write_to_register(0x0C, 0x00)?;
        self.write_to_register(0x0B, 0x01)?;

        self.poll_for_mcu_boot()?;

        self.write_to_register(0x7fff, 0x02)?;

        /* Firmware checksum */
        self.read_from_register((0x812FFC & 0xFFFF) as u16, 4)?;
        swap_buffer(&mut self.temp_buffer, 4);
        from_u8_to_u32(&self.temp_buffer[..4], &mut crc_checksum);
        if crc_checksum [0] != 0xc0b6c9e {
            return Err(Error::CheckSumFail);
        }

        /* Get offset NVM data and store them into the offset buffer */
        self.write_multi_to_register(0x2fd8, &VL53L8CX_GET_NVM_CMD)?;
        self.poll_for_answer(4, 0, VL53L8CX_UI_CMD_STATUS, 0xff, 2)?;

        self.read_from_register(VL53L8CX_UI_CMD_START, VL53L8CX_NVM_DATA_SIZE)?;
        self.offset_data.copy_from_slice(&self.temp_buffer[..VL53L8CX_OFFSET_BUFFER_SIZE]);
        self.send_offset_data(VL53L8CX_RESOLUTION_4X4)?;

        /* Set default Xtalk shape. Send Xtalk to sensor */
        self.xtalk_data.copy_from_slice(&VL53L8CX_DEFAULT_XTALK);
        self.send_xtalk_data(VL53L8CX_RESOLUTION_4X4)?;
      
        /* Send default configuration to VL53L8CX firmware */
        self.write_multi_to_register(0x2c34,&VL53L8CX_DEFAULT_CONFIGURATION)?;
        self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xff, 0x03)?;
        self.temp_buffer[..pipe_ctrl.len()].copy_from_slice(&pipe_ctrl);
        self.dci_write_data(VL53L8CX_DCI_PIPE_CONTROL, 4)?;
      
        if VL53L8CX_NB_TARGET_PER_ZONE != 1 {
            let tmp: [u8; 1] = [VL53L8CX_NB_TARGET_PER_ZONE as u8];
            self.dci_replace_data(VL53L8CX_DCI_FW_NB_TARGET, 16, &tmp, 1, 0x0C)?;
        }
        from_u32_to_u8(&single_range, &mut self.temp_buffer[..4]);
        self.dci_write_data( VL53L8CX_DCI_SINGLE_RANGE, 4)?;
      
        Ok(())
    }

/**s
 * @brief This function starts a ranging session. When the sensor streams, host
 * cannot change settings 'on-the-fly'.
 */
    pub fn start_ranging(&mut self) -> Result<(), Error<B::Error>> {
        let resolution: u8 = self.get_resolution()?;
        let mut tmp: [u16; 1] = [0];
        let mut header_config: [u32; 2] = [0, 0];
        let cmd: [u8; 4] = [0x00, 0x03, 0x00, 0x00];

        self.data_read_size = 0;
        self.streamcount = 255;
        let mut bh: BlockHeader;

        let mut output_bh_enable: [u32; 4] = [0x00000007, 0x00000000, 0x00000000, 0xC0000000];

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

        if !cfg!(feature = "VL53L8CX_DISABLE_AMBIENT_PER_SPAD") { output_bh_enable[0] += 8; }
        if !cfg!(feature = "VL53L8CX_DISABLE_NB_SPADS_ENABLED") { output_bh_enable[0] += 16; }
        if !cfg!(feature = "VL53L8CX_DISABLE_NB_TARGET_DETECTED") { output_bh_enable[0] += 32; }
        if !cfg!(feature = "VL53L8CX_DISABLE_SIGNAL_PER_SPAD") { output_bh_enable[0] += 64; }
        if !cfg!(feature = "VL53L8CX_DISABLE_RANGE_SIGMA_MM") { output_bh_enable[0] += 128; }
        if !cfg!(feature = "VL53L8CX_DISABLE_DISTANCE_MM") { output_bh_enable[0] += 256; }
        if !cfg!(feature = "VL53L8CX_DISABLE_REFLECTANCE_PERCENT") { output_bh_enable[0] += 512; }
        if !cfg!(feature = "VL53L8CX_DISABLE_TARGET_STATUS") { output_bh_enable[0] += 1024; }
        if !cfg!(feature = "VL53L8CX_DISABLE_MOTION_INDICATOR") { output_bh_enable[0] += 2048; }
        
        /* Update data size */
        for i in 0..12 {
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
            output[i] = bh.bh_bytes();
        }
        self.data_read_size += 24;

        from_u32_to_u8(&output, &mut self.temp_buffer[..48]);
        self.dci_write_data(VL53L8CX_DCI_OUTPUT_LIST, 48)?;
        
        header_config[0] = self.data_read_size;
        header_config[1] = 12+1 as u32;

        from_u32_to_u8(&header_config, &mut self.temp_buffer[..8]);
        self.dci_write_data(VL53L8CX_DCI_OUTPUT_CONFIG, 8)?;

        from_u32_to_u8(&output_bh_enable, &mut self.temp_buffer[..16]);
        self.dci_write_data(VL53L8CX_DCI_OUTPUT_ENABLES, 16)?;
        
        /* Start xshut bypass (interrupt mode) */
        self.write_to_register(0x7fff, 0x00)?;
        self.write_to_register(0x09, 0x05)?;
        self.write_to_register(0x7fff, 0x02)?;

        /* Start ranging session */
        self.write_multi_to_register(VL53L8CX_UI_CMD_END - (4-1), &cmd)?;
        self.poll_for_answer(4, 1, VL53L8CX_UI_CMD_STATUS, 0xff, 0x03)?;

        /* Read ui range data content and compare if data size is the correct one */
        self.dci_read_data(0x5440, 12)?;
        from_u8_to_u16(&self.temp_buffer[0x8..0x8+2], &mut tmp);
        if tmp[0] != self.data_read_size as u16 {   
            return Err(Error::Other);
        }

        /* Ensure that there is no laser safety fault */
        self.dci_read_data(0xe0c4, 8)?;
        if self.temp_buffer[0x6] != 0 {
            return Err(Error::Other);
        }

        Ok(())
    }

/**
 * @brief This function stops the ranging session. It must be used when the
 * sensor streams, after calling vl53l8cx_start_ranging().
 */
    #[allow(dead_code)]
    pub fn stop_ranging(&mut self) -> Result<(), Error<B::Error>> {
        let mut timeout: u16 = 0;
        let mut auto_flag_stop: [u32; 1] = [0];

        self.read_from_register(0x2ffc, 4)?;
        from_u8_to_u32(&self.temp_buffer[..4], &mut auto_flag_stop);

        if auto_flag_stop[0] != 0x4ff {
            self.write_to_register(0x7fff, 0x00)?;

            /* Provoke MCU stop */
            self.write_to_register(0x15, 0x16)?;
            self.write_to_register(0x14, 0x01)?;

            /* Poll for G02 status 0 MCU stop */
            while self.temp_buffer[0] & (0x80 as u8) >> 7 == 0x00 && timeout <= 500 {
                self.read_from_register(0x6, 1)?;
                self.delay(10);

                timeout += 1;
            }
        }

        /* Check GO2 status 1 if status is still OK */
        self.read_from_register(0x6, 1)?;
        if self.temp_buffer[0] & 0x80 != 0 {
            self.read_from_register(0x7, 1)?;
            if self.temp_buffer[0] != 0x84 && self.temp_buffer[0] != 0x85 {
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
    /**
 * @brief This function checks if a new data is ready by polling I2C. If a new
 * data is ready, a flag will be raised.
 * @return (bool) isReady : Value is 0 if data
 * is not ready, or 1 if a new data is ready.
 */
    pub fn check_data_ready(&mut self) -> Result<bool, Error<B::Error>> {
        let is_ready: bool;
        self.read_from_register(0, 4)?;
        if (self.temp_buffer[0] != self.streamcount) 
            && (self.temp_buffer[0] != 0xff) 
            && (self.temp_buffer[1] == 0x05) 
            && (self.temp_buffer[2] & 0x05 == 0x05) 
            && (self.temp_buffer[3] & 0x10 == 0x10) 
        {
            is_ready = true;
            self.streamcount = self.temp_buffer[0];
        } else {
            if self.temp_buffer[3] & 0x80 != 0 {
                return Err(Error::Go2);
            }
            is_ready = false;
        }

        Ok(is_ready)
    }

/**
 * @brief This function gets the ranging data, using the selected output and the
 * resolution.
 * @return (ResultsData) results : VL53L5 results structure.
 */
    pub fn get_ranging_data(&mut self) -> Result<ResultsData, Error<B::Error>> {
        let mut result: ResultsData = ResultsData::new();
        let mut msize: usize;
        let mut header_id: u16;
        let mut footer_id: u16;
        let mut bh: BlockHeader;

        self.read_from_register(0, self.data_read_size as usize)?;
        self.streamcount = self.temp_buffer[0];
        swap_buffer(&mut self.temp_buffer, self.data_read_size as usize);

        /* Start conversion at position 16 to avoid headers */
        let mut i: usize = 16;
        while i < self.data_read_size as usize {

            let mut buf: [u32; 1] = [0;1];
            from_u8_to_u32(&self.temp_buffer[i..i+4], &mut buf);
            bh = BlockHeader(buf[0]);

            if bh.bh_type() > 0x1 && bh.bh_type() < 0xd {
                msize = (bh.bh_type() * bh.bh_size()) as usize;
            } else  {
                msize = bh.bh_size() as usize;
            }

            i += 4;

            if bh.bh_idx() == VL53L8CX_METADATA_IDX as u32 {
                result.silicon_temp_degc = self.temp_buffer[i+8] as i8;
                i += msize; 
                continue;
            } 
            
            let mut src: &[u8] = &[0];
            if i+msize <= VL53L8CX_TEMPORARY_BUFFER_SIZE {
                src = &self.temp_buffer[i..i+msize];
            }

            i += msize;
            
            #[cfg(not(feature = "VL53L8CX_DISABLE_AMBIENT_PER_SPAD"))] 
            if bh.bh_idx() == VL53L8CX_AMBIENT_RATE_IDX as u32 {
                from_u8_to_u32(src, &mut result.ambient_per_spad);
                continue;
            }

            #[cfg(not(feature = "VL53L8CX_DISABLE_NB_SPADS_ENABLED"))] 
            if bh.bh_idx() == VL53L8CX_SPAD_COUNT_IDX as u32 {
                from_u8_to_u32(src, &mut result.nb_spads_enabled);
                continue;
            }

            #[cfg(not(feature = "VL53L8CX_DISABLE_NB_TARGET_DETECTED"))] 
            if bh.bh_idx() == VL53L8CX_NB_TARGET_DETECTED_IDX as u32 {
                result.nb_target_detected[..msize].copy_from_slice(src); 
                continue;
            }

            #[cfg(not(feature = "VL53L8CX_DISABLE_SIGNAL_PER_SPAD"))]
            if bh.bh_idx() == VL53L8CX_SIGNAL_RATE_IDX as u32 {
                from_u8_to_u32(src, &mut result.signal_per_spad);
                continue;
            }

            #[cfg(not(feature = "VL53L8CX_DISABLE_RANGE_SIGMA_MM"))]
            if bh.bh_idx() == VL53L8CX_RANGE_SIGMA_MM_IDX as u32 {
                from_u8_to_u16(src, &mut result.range_sigma_mm);   
                continue;
            } 

            #[cfg(not(feature = "VL53L8CX_DISABLE_DISTANCE_MM"))] 
            if bh.bh_idx() == VL53L8CX_DISTANCE_IDX as u32 {
                from_u8_to_i16(src, &mut result.distance_mm);
                continue;
            }

            #[cfg(not(feature= "VL53L8CX_DISABLE_REFLECTANCE_PERCENT"))]
            if bh.bh_idx() == VL53L8CX_REFLECTANCE_EST_PC_IDX as u32 {
                result.reflectance[..msize].copy_from_slice(src); 
                continue;
            }

            #[cfg(not(feature = "VL53L8CX_DISABLE_TARGET_STATUS"))]
            if bh.bh_idx() == VL53L8CX_TARGET_STATUS_IDX as u32 {
                result.target_status[..msize].copy_from_slice(src); 
                continue;
            }

            #[cfg(not(feature = "VL53L8CX_DISABLE_MOTION_INDICATOR"))]
            if bh.bh_idx() == VL53L8CX_MOTION_DETEC_IDX as u32 {
                from_u8_to_motion_indicator(src, &mut result.motion_indicator);
                continue;
            }
        }
        if VL53L8CX_USE_RAW_FORMAT == 0 {
            /* Convert data into their real format */
            #[cfg(not(feature = "VL53L8CX_DISABLE_AMBIENT_PER_SPAD"))] {
                for i in 0..VL53L8CX_RESOLUTION_8X8 as usize {
                    result.ambient_per_spad[i] /= 2048;
                }
            }
            for i in 0..(VL53L8CX_RESOLUTION_8X8 as usize)*(VL53L8CX_NB_TARGET_PER_ZONE as usize) {
                #[cfg(not(feature = "VL53L8CX_DISABLE_DISTANCE_MM"))] {
                    result.distance_mm[i] /= 4;
                    if result.distance_mm[i] < 0 {
                        result.distance_mm[i] = 0;
                    }
                }
                #[cfg(not(feature = "VL53L8CX_DISABLE_REFLECTANCE_PERCENT"))] {
                    result.reflectance[i] /= 2;
                }
                #[cfg(not(feature = "VL53L8CX_RANGE_SIGMA_MM"))]{
                    result.range_sigma_mm[i] /= 128;
                }
                #[cfg(not(feature = "VL53L8CX_DISABLE_SIGNAL_PER_SPAD"))] {
                    result.signal_per_spad[i] /= 2048;   
                }
                /* Set target status to 255 if no target is detected for this zone */
                #[cfg(not(any(feature="VL53L8CX_DISABLE_DISTANCE_MM", feature="VL53L8CX_DISABLE_TARGET_STATUS")))] {
                    for i in 0..VL53L8CX_RESOLUTION_8X8 as usize {
                        if result.nb_target_detected[i] == 0 {
                            for j in 0..VL53L8CX_NB_TARGET_PER_ZONE as usize {
                                result.target_status[VL53L8CX_NB_TARGET_PER_ZONE as usize*i + j] = 255;
                            }
                        }
                    }
                }
            }

            #[cfg(not(feature = "VL53L8CX_DISABLE_MOTION_INDICATOR"))] {
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
            return Err(Error::CorruptedFrame);
        }

        Ok(result)
    }    

}