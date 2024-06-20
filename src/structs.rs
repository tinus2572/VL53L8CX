use consts::*;
use crate::{consts, BusOperation, Output, PushPull, Pin, SysDelay, bitfield, MotionIndicator};

pub struct Vl53l8cx<B: BusOperation> {
    pub temp_buffer: [u8;  VL53L8CX_TEMPORARY_BUFFER_SIZE],
    pub offset_data: [u8;  VL53L8CX_OFFSET_BUFFER_SIZE],
    pub xtalk_data: [u8; VL53L8CX_XTALK_BUFFER_SIZE],
    pub streamcount: u8,
    pub data_read_size: u32,

    pub lpn_pin: Pin<'B', 0, Output<PushPull>>,
    pub i2c_rst_pin: i8,
    
    pub bus: B,
    pub delay: SysDelay
}


#[derive(Copy, Clone, Debug)]
pub enum Error<B> {
    Bus(B),
    Other,
}

bitfield! {
    struct BlockHeader(u32);
    bh_idx, set_bh_idx: 16, 12;
    bh_size, set_bh_size: 12, 4;
    bh_type, set_bh_type: 4, 0;
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
