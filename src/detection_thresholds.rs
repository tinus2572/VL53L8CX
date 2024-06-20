
use consts::*;

use crate::{consts, BusOperation, Vl53l8cx, Error};

#[repr(C)]
pub struct DetectionThresholds {
    param_low_thresh: i32,
    param_high_thresh: i32,
    measurement: u8,
    th_type: u8,
    zone_num: u8,
    math_op: u8
}

impl<B: BusOperation> Vl53l8cx<B> {
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
}