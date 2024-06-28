use consts::*;
use utils::*;

use crate::{consts, utils, BusOperation, Vl53l8cx, Error};


impl<B: BusOperation> Vl53l8cx<B> {
    
    pub fn get_resolution(&mut self) -> Result<u8, Error<B::Error>> {
        self.dci_read_data(VL53L8CX_DCI_ZONE_CONFIG, 8)?;
        let resolution: u8 = self.temp_buffer[0x00] * self.temp_buffer[0x01];

        Ok(resolution)
    }

    pub fn set_resolution(&mut self, resolution: u8) -> Result<(), Error<B::Error>> {

        if resolution == VL53L8CX_RESOLUTION_4X4 {
            self.dci_read_data(VL53L8CX_DCI_DSS_CONFIG, 16)?;
            self.temp_buffer[0x04] = 64;
            self.temp_buffer[0x06] = 64;
            self.temp_buffer[0x09] = 4;
            self.dci_write_data(VL53L8CX_DCI_DSS_CONFIG, 16)?;
            self.dci_read_data(VL53L8CX_DCI_ZONE_CONFIG, 8)?;
            self.temp_buffer[0x00] = 4;
            self.temp_buffer[0x01] = 4;
            self.temp_buffer[0x04] = 8;
            self.temp_buffer[0x05] = 8;
            self.dci_write_data(VL53L8CX_DCI_ZONE_CONFIG, 8)?;
        } else if resolution == VL53L8CX_RESOLUTION_8X8 {
            self.dci_read_data(VL53L8CX_DCI_DSS_CONFIG, 16)?;
            self.temp_buffer[0x04] = 16;
            self.temp_buffer[0x06] = 16;
            self.temp_buffer[0x09] = 1;
            self.dci_write_data(VL53L8CX_DCI_DSS_CONFIG, 16)?;
            self.dci_read_data(VL53L8CX_DCI_ZONE_CONFIG, 8)?;
            self.temp_buffer[0x00] = 8;
            self.temp_buffer[0x01] = 8;
            self.temp_buffer[0x04] = 4;
            self.temp_buffer[0x05] = 4;
            self.dci_write_data(VL53L8CX_DCI_ZONE_CONFIG, 8)?;
        } else {
            return Err(Error::InvalidParam);
        }
        self.send_offset_data(resolution)?;
        self.send_xtalk_data(resolution)?;

        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_power_mode(&mut self) -> Result<u8, Error<B::Error>> {
        let power_mode: u8;
        self.write_to_register(0x7fff, 0x00)?;
        self.read_from_register(0x009, 1)?;    
        if self.temp_buffer[0] == 0x4 {
            power_mode = VL53L8CX_POWER_MODE_WAKEUP;
        } else if self.temp_buffer[0] == 0x2 {
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
    pub fn get_external_sync_pin_enable(&mut self) -> Result<u8, Error<B::Error>> {
        let is_sync_pin_enabled: u8;
        self.dci_read_data(VL53L8CX_DCI_SYNC_PIN, 4)?;

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
        self.read_from_register(VL53L8CX_DCI_SYNC_PIN, 4)?;
        from_u8_to_u32(&self.temp_buffer[3..3+4], &mut tmp);
        
        /* Update bit 1 with mask (set sync pause bit) */
        if enable_sync_pin == 0 {
            tmp[0] &= !(1 << 1);
        } else {
            tmp[0] |= 1 << 1;
        }

        self.temp_buffer[3] = tmp[0] as u8;
        self.dci_write_data(VL53L8CX_DCI_SYNC_PIN, 4)?;

        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_target_order(&mut self) -> Result<u8, Error<B::Error>> {
        let target_order: u8;
        self.dci_read_data(VL53L8CX_DCI_TARGET_ORDER, 4)?;
        target_order = self.temp_buffer[0];

        Ok(target_order)
    }

    #[allow(dead_code)]
    pub fn set_target_order(&mut self, target_order: u8) -> Result<(), Error<B::Error>> {
        if target_order == VL53L8CX_TARGET_ORDER_CLOSEST || target_order == VL53L8CX_TARGET_ORDER_STRONGEST {
            self.dci_replace_data(VL53L8CX_DCI_TARGET_ORDER, 4, &[target_order], 1, 0x0)?;
        } else {
            return Err(Error::InvalidParam);
        }
        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_sharpener_percent(&mut self) -> Result<u8, Error<B::Error>> {
        let sharpener_percent: u8;
        self.dci_read_data(VL53L8CX_DCI_SHARPENER, 16)?;
        sharpener_percent = self.temp_buffer[0xD] * 100 / 255;

        Ok(sharpener_percent)
    }

    #[allow(dead_code)]
    pub fn set_sharpener_percent(&mut self, sharpener_percent: u8) -> Result<(), Error<B::Error>> {
        let sharpener: u8;
        if sharpener_percent >= 100 {
            return Err(Error::InvalidParam);
        }
        sharpener = sharpener_percent * 255 / 100;
        self.dci_replace_data(VL53L8CX_DCI_SHARPENER, 16, &[sharpener], 1, 0xd)?;

        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_integration_time(&mut self) -> Result<u32, Error<B::Error>> {
        let mut time_ms: [u32; 1] = [0];
        self.dci_read_data(VL53L8CX_DCI_INT_TIME, 20)?;
        from_u8_to_u32(&self.temp_buffer[..4], &mut time_ms);
        
        time_ms[0] /= 1000;
        
        Ok(time_ms[0])
    }

    #[allow(dead_code)]
    pub fn set_integration_time(&mut self, integration_time_ms: u32) -> Result<(), Error<B::Error>> {
        let mut integration: u32 = integration_time_ms;

        /* Integration time must be between 2ms and 1000ms */
        if integration < 2 || integration > 1000 {
            return Err(Error::InvalidParam);
        }
        integration *= 1000;
        
        let mut buf: [u8; 4] = [0; 4];
        let tmp: [u32; 1] = [integration];
        from_u32_to_u8(&tmp, &mut buf);
        
        self.dci_replace_data(VL53L8CX_DCI_INT_TIME, 20, &buf, 4, 0x00)?;

        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_ranging_mode(&mut self) -> Result<u8, Error<B::Error>> {
        let ranging_mode: u8;
        self.dci_read_data(VL53L8CX_DCI_RANGING_MODE, 8)?;
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
        self.dci_read_data(VL53L8CX_DCI_RANGING_MODE, 8)?;

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

        self.dci_write_data(VL53L8CX_DCI_RANGING_MODE, 8)?;

        from_u32_to_u8(&single_range, &mut self.temp_buffer[..4]);
        self.dci_write_data(VL53L8CX_DCI_SINGLE_RANGE, 4)?;
        
        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_frequency_hz(&mut self) -> Result<u8, Error<B::Error>> {
        self.dci_read_data(VL53L8CX_DCI_FREQ_HZ, 4)?;
        let frequency_hz: u8 = self.temp_buffer[0x01];

        Ok(frequency_hz)
    }

    #[allow(dead_code)]
    pub fn set_frequency_hz(&mut self, frequency_hz: u8) -> Result<(), Error<B::Error>> {
        let tmp: [u8; 1] = [frequency_hz];
        self.dci_replace_data(VL53L8CX_DCI_FREQ_HZ, 4, &tmp, 1, 0x01)?;

        Ok(())
    }
}