
use consts::*;

use crate::{consts, BusOperation, Vl53l8cx, Error};


#[repr(C)]
#[allow(dead_code)]
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
    #[allow(dead_code)]
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
    pub global_indicator_1: u32,
    pub global_indicator_2: u32,
    pub status: u8,
    pub nb_of_detected_aggregates: u8,
    pub nb_of_aggregates: u8,
    pub spare: u8,
    pub motion: [u32; 32],
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

impl<B: BusOperation> Vl53l8cx<B> {

    #[allow(dead_code)]
    fn motion_indicator_init(&mut self, resolution: u8) -> Result<(), Error<B::Error>> {
        let mut motion_config = MotionConfiguration::new();
        self.motion_indicator_set_resolution(&mut motion_config, resolution)?;
        Ok(())
    }

    #[allow(dead_code)]
    fn dci_write_data_motion_config(&mut self, motion_config: &mut MotionConfiguration) -> Result<(), Error<B::Error>> {
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

    #[allow(dead_code)]
    fn motion_indicator_set_distance_motion(&mut self, motion_config: &mut MotionConfiguration, distance_min_mm: u16, distance_max_mm: u16) -> Result<(), Error<B::Error>> {
        let mut tmp: f64;
        if distance_max_mm - distance_min_mm > 1500 || distance_max_mm > 4000 || distance_min_mm < 400 {
            return Err(Error::Other);
        }

        tmp = ((distance_min_mm as f64 / 37.5348) - 4.0) * 2048.5;
        motion_config.ref_bin_offset = tmp as i32;

        tmp = ((((distance_max_mm - distance_min_mm) as f64 / 10.0) + 30.02784) / 15.01392) + 0.5;
        motion_config.feature_length = tmp as u8;

        self.dci_write_data_motion_config(motion_config)?;
        Ok(())
    }

    #[allow(dead_code)]
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

        self.dci_write_data_motion_config(motion_config)?;
        Ok(())
    }
}