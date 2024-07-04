
use consts::*;
use utils::*;

use crate::{consts, utils, BusOperation, Error, Vl53l8cx};


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
        MotionConfiguration { 
            ref_bin_offset: 13633, 
            detection_threshold: 2883584, 
            extra_noise_sigma: 0, 
            null_den_clip_value: 0, 
            mem_update_mode: 6, 
            mem_update_choice: 2, 
            sum_span: 4, 
            feature_length: 9, 
            nb_of_aggregates: 16, 
            nb_of_temporal_accumulations: 16, 
            min_nb_for_global_detection: 1, 
            global_indicator_format_1: 8, 
            global_indicator_format_2: 0, 
            spare_1: 0, 
            spare_2: 0, 
            spare_3: 0, 
            map_id: [0; 64], 
            indicator_format_1: [0; 32], 
            indicator_format_2: [0; 32] 
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

pub fn from_u8_to_motion_indicator(src: &[u8], dst: &mut MotionIndicator) {
    from_u8_to_u32(&src[..4], &mut [dst.global_indicator_1]);
    from_u8_to_u32(&src[4..8], &mut [dst.global_indicator_2]);
    dst.status = src[8];
    dst.nb_of_detected_aggregates = src[9];
    dst.nb_of_aggregates = src[10];
    dst.spare = src[11];
    from_u8_to_u32(&src[12..140], &mut dst.motion);
}

#[allow(dead_code)]
pub fn from_u8_to_motion_configuration(src: &[u8], dst: &mut MotionConfiguration) {
    from_u8_to_i32(&src[..4], &mut [dst.ref_bin_offset]);
    from_u8_to_u32(&src[4..8], &mut [dst.detection_threshold]);
    from_u8_to_u32(&src[8..12], &mut [dst.extra_noise_sigma]);
    from_u8_to_u32(&src[12..16], &mut [dst.null_den_clip_value]);
    dst.mem_update_mode = src[16];
    dst.mem_update_choice = src[17];
    dst.sum_span = src[18];
    dst.feature_length = src[19];
    dst.nb_of_aggregates = src[20];
    dst.nb_of_temporal_accumulations = src[21];
    dst.min_nb_for_global_detection = src[22];
    dst.global_indicator_format_1 = src[23];
    dst.global_indicator_format_2 = src[24];
    dst.spare_1 = src[25];
    dst.spare_2 = src[26];
    dst.spare_3 = src[27];
    for i in 0..64 {
        dst.map_id[i] = src[28+i] as i8;
    }
    dst.indicator_format_1.copy_from_slice(&src[92..124]);
    dst.indicator_format_2.copy_from_slice(&src[124..156]);
}

pub fn from_motion_configuration_to_u8(src: &MotionConfiguration, dst: &mut [u8]) {
    from_i32_to_u8(&[src.ref_bin_offset], &mut dst[..4]);
    from_u32_to_u8(&[src.detection_threshold], &mut dst[4..8]);
    from_u32_to_u8(&[src.extra_noise_sigma], &mut dst[8..12]);
    from_u32_to_u8(&[src.null_den_clip_value], &mut dst[12..16]);
    dst[16] = src.mem_update_mode;
    dst[17] = src.mem_update_choice;
    dst[18] = src.sum_span;
    dst[19] = src.feature_length;
    dst[20] = src.nb_of_aggregates;
    dst[21] = src.nb_of_temporal_accumulations;
    dst[22] = src.min_nb_for_global_detection;
    dst[23] = src.global_indicator_format_1;
    dst[24] = src.global_indicator_format_2;
    dst[25] = src.spare_1;
    dst[26] = src.spare_2;
    dst[27] = src.spare_3;
    for i in 0..64 {
        dst[28+i] = src.map_id[i] as u8;
    }
    dst[92..124].copy_from_slice(&src.indicator_format_1);
    dst[124..156].copy_from_slice(&src.indicator_format_2);
}

impl<B: BusOperation> Vl53l8cx<B> {

    #[allow(dead_code)]
    fn motion_indicator_init(&mut self, resolution: u8) -> Result<(), Error<B::Error>> {
        let mut motion_config = MotionConfiguration::new();
        self.motion_indicator_set_resolution(&mut motion_config, resolution)?;
        Ok(())
    }

    #[allow(dead_code)]
    fn motion_indicator_set_distance_motion(&mut self, motion_config: &mut MotionConfiguration, distance_min_mm: u16, distance_max_mm: u16) -> Result<(), Error<B::Error>> {
        let mut tmp: f64;
        if distance_max_mm - distance_min_mm > 1500 || distance_max_mm > 4000 || distance_min_mm < 400 {
            return Err(Error::InvalidParam);
        }

        tmp = ((distance_min_mm as f64 / 37.5348) - 4.0) * 2048.5;
        motion_config.ref_bin_offset = tmp as i32;

        tmp = ((((distance_max_mm - distance_min_mm) as f64 / 10.0) + 30.02784) / 15.01392) + 0.5;
        motion_config.feature_length = tmp as u8;

        from_motion_configuration_to_u8(&motion_config, &mut self.temp_buffer[..156]);
        self.dci_write_data(VL53L8CX_DCI_MOTION_DETECTOR_CFG, 156)?;

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
            return Err(Error::InvalidParam);
        }

        from_motion_configuration_to_u8(&motion_config, &mut self.temp_buffer[..156]);
        self.dci_write_data(VL53L8CX_DCI_MOTION_DETECTOR_CFG, 156)?;
        Ok(())
    }
}