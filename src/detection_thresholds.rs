
use consts::*;
use utils::*;

use crate::{consts, utils, BusOperation, Vl53l8cx, Error};

#[repr(C)]
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub struct DetectionThresholds {
    pub param_low_thresh: i32,
    pub param_high_thresh: i32,
    pub measurement: u8,
    pub th_type: u8,
    pub zone_num: u8,
    pub math_op: u8
}

impl DetectionThresholds {
    pub fn new() -> Self {
        let param_low_thresh: i32 = 0;
        let param_high_thresh: i32 = 0;
        let measurement: u8 = VL53L8CX_DISTANCE_MM;
        let th_type: u8 = VL53L8CX_IN_WINDOW;
        let zone_num: u8 = 0;
        let math_op: u8 = VL53L8CX_OPERATION_NONE;
        Self { param_low_thresh,
            param_high_thresh,
            measurement,
            th_type,
            zone_num,
            math_op }
    }
}

fn from_u8_to_thresholds(src: &[u8], dst: &mut [DetectionThresholds]) {
    for i in 0..dst.len() {
        let j: usize = 12 * i;
        from_u8_to_i32(&src[j..j+4], &mut [dst[i].param_low_thresh]);
        from_u8_to_i32(&src[j+4..j+8], &mut [dst[i].param_high_thresh]);
        dst[i].measurement = src[j+8];
        dst[i].th_type = src[j+9];
        dst[i].zone_num = src[j+10];
        dst[i].math_op = src[j+11];
    }
}

fn from_thresholds_to_u8(src: &[DetectionThresholds], dst: &mut [u8]) {
    for i in 0..src.len() {
        let j: usize = 12 * i;
        from_i32_to_u8(&[src[i].param_low_thresh], &mut dst[j..j+4]);
        from_i32_to_u8(&[src[i].param_high_thresh], &mut dst[j+4..j+8]);
        dst[j+8] = src[i].measurement;
        dst[j+9] = src[i].th_type;
        dst[j+10] = src[i].zone_num;
        dst[j+11] = src[i].math_op;
    }
}

impl<B: BusOperation> Vl53l8cx<B> {
#[allow(dead_code)]
    pub fn get_detection_threshholds_enable(&mut self) -> Result<u8, Error<B::Error>> {
        let enabled: u8;
        self.dci_read_data(VL53L8CX_DCI_DET_THRESH_GLOBAL_CONFIG, 8)?;
        enabled = self.temp_buffer[0x1];
        Ok(enabled)
    }

#[allow(dead_code)]
    pub fn set_detection_threshholds_enable(&mut self, enabled: u8) -> Result<(), Error<B::Error>> {
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
        self.dci_replace_data(VL53L8CX_DCI_DET_THRESH_GLOBAL_CONFIG, 8, &grp_global_config, 4, 0x00)?;
        
        /* Update interrupt config */
        self.dci_replace_data(VL53L8CX_DCI_DET_THRESH_CONFIG, 20, &tmp, 1, 0x11)?;
        
        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_detection_threshholds(&mut self, thresholds: &mut [DetectionThresholds; VL53L8CX_NB_THRESHOLDS] ) -> Result<(), Error<B::Error>> {
  
        /* Get thresholds configuration */
        from_thresholds_to_u8(thresholds, &mut self.temp_buffer[..VL53L8CX_NB_THRESHOLDS * 12]);
        self.dci_read_data(VL53L8CX_DCI_DET_THRESH_START, VL53L8CX_NB_THRESHOLDS * 12)?;
        from_u8_to_thresholds(&self.temp_buffer[..VL53L8CX_NB_THRESHOLDS * 12], thresholds);
        
        for i in 0..VL53L8CX_NB_THRESHOLDS {
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

    #[allow(dead_code)]
    pub fn set_detection_threshholds(&mut self, thresholds: &mut [DetectionThresholds; VL53L8CX_NB_THRESHOLDS] ) -> Result<(), Error<B::Error>> {
        for i in 0..VL53L8CX_NB_THRESHOLDS {
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

        /* Set valid target list */
        self.temp_buffer[..8].copy_from_slice(&[0x05; 8]);
        self.dci_write_data(VL53L8CX_DCI_DET_THRESH_VALID_STATUS, 8)?;
        
        /* Set thresholds configuration */
        from_thresholds_to_u8(thresholds, &mut self.temp_buffer[..VL53L8CX_NB_THRESHOLDS * 12]);
        self.dci_write_data(VL53L8CX_DCI_DET_THRESH_START, VL53L8CX_NB_THRESHOLDS * 12)?;

        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_detection_threshholds_auto_stop(&mut self) -> Result<u8, Error<B::Error>> {
        self.dci_read_data(VL53L8CX_DCI_PIPE_CONTROL, 4)?;
        let auto_stop: u8 = self.temp_buffer[0x03];
        Ok(auto_stop)
    }

    #[allow(dead_code)]
    pub fn set_detection_threshholds_auto_stop(&mut self, auto_stop: u8) -> Result<u8, Error<B::Error>> {
        let tmp: [u8; 1] = [auto_stop];
        self.dci_replace_data(VL53L8CX_DCI_PIPE_CONTROL, 4, &tmp, 1, 0x03)?;
        Ok(auto_stop)
    }
}