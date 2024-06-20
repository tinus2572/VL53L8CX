
use consts::*;
use buffers::*;

use crate::{consts, buffers, BusOperation, Vl53l8cx, Error, bitfield};

bitfield! {
    struct BlockHeader(u32);
    bh_idx, set_bh_idx: 16, 12;
    bh_size, set_bh_size: 12, 4;
    bh_type, set_bh_type: 4, 0;
}

impl<B: BusOperation> Vl53l8cx<B> {
    #[allow(dead_code)]
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

    #[allow(dead_code)]
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

    #[allow(dead_code)]
    fn get_xtalk_margin(&mut self) -> Result<u32, Error<B::Error>> {
        self.dci_read_data_temp_buffer(VL53L8CX_DCI_XTALK_CFG, 16)?;
        let mut xtalk_margin: u32 = (self.temp_buffer[0] as u32) << 24 | (self.temp_buffer[1] as u32) << 16 | (self.temp_buffer[2] as u32) << 8 | self.temp_buffer[3] as u32;
        xtalk_margin /= 2048;
        Ok(xtalk_margin)
    }
    
    #[allow(dead_code)]
    fn set_xtalk_margin(&mut self, xtalk_margin: u32) -> Result<(), Error<B::Error>> {
        let mut margin_kcps: [u8; 4] = [0; 4];
        margin_kcps.copy_from_slice(&(xtalk_margin<<11).to_ne_bytes());
        if xtalk_margin > 10000 {
            return Err(Error::Other);
        }
        self.dci_replace_data_temp_buffer(VL53L8CX_DCI_XTALK_CFG, 16, &margin_kcps, 4, 0)?;

        Ok(())
    }

    #[allow(dead_code)]
    fn calibrate_xtalk(&mut self, reflectance_percent: u16, nb_samples: u8, distance_mm: u16) -> Result<(), Error<B::Error>> {
        let mut timeout: u16 = 0;
        let mut continue_loop: u8 = 1;
        let cmd: [u8; 4] = [0x00, 0x03, 0x00, 0x00];
        let footer: [u8; 8] = [0x00, 0x00, 0x00, 0x0F, 0x00, 0x01, 0x03, 0x04];
        let mut reflectance = reflectance_percent;
        let mut distance = distance_mm;
        let samples = nb_samples;
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
                // return Err(Error::Other);
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

    #[allow(dead_code)]
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

    #[allow(dead_code)]
    fn set_caldata_xtalk(&mut self, xtalk_data: [u8; VL53L8CX_XTALK_BUFFER_SIZE]) -> Result<(), Error<B::Error>> {
        let resolution = self.get_resolution()?;
        self.xtalk_data.copy_from_slice(&xtalk_data);
        self.set_resolution(resolution)?;

        Ok(())
    }
}