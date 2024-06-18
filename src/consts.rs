
pub const VL53L8CX_DEFAULT_I2C_ADDRESS: u8 = 0x52;

pub const VL53L8CX_RESOLUTION_4X4: u8 = 16;
pub const VL53L8CX_RESOLUTION_8X8: u8 = 64;


pub const VL53L8CX_STATUS_OK: u8 = 0;
pub const VL53L8CX_STATUS_TIMEOUT_ERROR: u8 = 1;
pub const VL53L8CX_STATUS_CORRUPTED_FRAME: u8 = 2;
pub const VL53L8CX_STATUS_LASER_SAFETY: u8 = 3;
pub const VL53L8CX_MCU_ERROR: u8 = 66;
pub const VL53L8CX_STATUS_INVALID_PARAM: u8 = 127;
pub const VL53L8CX_STATUS_ERROR: u8 = 255;


pub const VL53L8CX_START_BH: u32 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0x0000000D } else { 0x0000000D };
pub const VL53L8CX_METADATA_BH: u32 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0x54B400C0 } else { 0x54B400C0 };
pub const VL53L8CX_COMMONDATA_BH: u32 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0x54C00040 } else { 0x54C00040 };
pub const VL53L8CX_AMBIENT_RATE_BH: u32 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0x54D00104 } else { 0x54D00104 };
pub const VL53L8CX_SPAD_COUNT_BH: u32 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0x55D00404 } else { 0x57D00401 };
pub const VL53L8CX_NB_TARGET_DETECTED_BH: u32 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0xDB840401 } else { 0x55D00404 };
pub const VL53L8CX_SIGNAL_RATE_BH: u32 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0xDBC40404 } else { 0x58900404 };
pub const VL53L8CX_RANGE_SIGMA_MM_BH: u32 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0xDEC40402 } else { 0x64900402 };
pub const VL53L8CX_DISTANCE_BH: u32 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0xDF440402 } else { 0x66900402 };
pub const VL53L8CX_REFLECTANCE_BH: u32 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0xE0440401 } else { 0x6A900401 };
pub const VL53L8CX_TARGET_STATUS_BH: u32 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0xE0840401 } else { 0x6B900401 };
pub const VL53L8CX_MOTION_DETECT_BH: u32 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0xD85808C0 } else { 0xCC5008C0 };

pub const VL53L8CX_METADATA_IDX: u16 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0x54B4 } else { 0x54B4 };
pub const VL53L8CX_SPAD_COUNT_IDX: u16 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0x55D0 } else { 0x55D0 };
pub const VL53L8CX_AMBIENT_RATE_IDX: u16 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0x54D0 } else { 0x54D0 };
pub const VL53L8CX_NB_TARGET_DETECTED_IDX: u16 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0xDB84 } else { 0x57D0 };
pub const VL53L8CX_SIGNAL_RATE_IDX: u16 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0xDBC4 } else { 0x5890 };
pub const VL53L8CX_RANGE_SIGMA_MM_IDX: u16 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0xDEC4 } else { 0x6490 };
pub const VL53L8CX_DISTANCE_IDX: u16 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0xDF44 } else { 0x6690 };
pub const VL53L8CX_REFLECTANCE_EST_PC_IDX: u16 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0xE044 } else { 0x6A90 };
pub const VL53L8CX_TARGET_STATUS_IDX: u16 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0xE084 } else { 0x6B90 };
pub const VL53L8CX_MOTION_DETEC_IDX: u16 = if VL53L8CX_NB_TARGET_PER_ZONE == 1 { 0xD858 } else { 0xCC50 };

pub const VL53L8CX_NVM_DATA_SIZE: usize = 492;
pub const VL53L8CX_CONFIGURATION_SIZE: usize = 972;
pub const VL53L8CX_OFFSET_BUFFER_SIZE: usize = 488;
pub const VL53L8CX_XTALK_BUFFER_SIZE: usize = 776;

// pub const VL53L8CX_DCI_FREQ_HZ: u16 = 0x5458;
// pub const VL53L8CX_DCI_INT_TIME: u16 = 0x545C;
// pub const VL53L8CX_DCI_RANGING_MODE: u16 = 0xAD30;
// pub const VL53L8CX_DCI_DSS_CONFIG: u16 = 0xAD38;
// pub const VL53L8CX_DCI_TARGET_ORDER: u16 = 0xAE64;
// pub const VL53L8CX_DCI_SHARPENER: u16 = 0xAED8;
// pub const VL53L8CX_DCI_INTERNAL_CP: u16 = 0xB39C;
// pub const VL53L8CX_DCI_SYNC_PIN: u16 = 0xB5F0;
// pub const VL53L8CX_DCI_MOTION_DETECTOR_CFG: u16 = 0xBFAC;
pub const VL53L8CX_DCI_ZONE_CONFIG: u16 = 0x5450;
pub const VL53L8CX_DCI_FW_NB_TARGET: u16 = 0x5478;
pub const VL53L8CX_DCI_SINGLE_RANGE: u16 = 0xD964;
pub const VL53L8CX_DCI_OUTPUT_CONFIG: u16 = 0xD968;
pub const VL53L8CX_DCI_OUTPUT_ENABLES: u16 = 0xD970;
pub const VL53L8CX_DCI_OUTPUT_LIST: u16 = 0xD980;
pub const VL53L8CX_DCI_PIPE_CONTROL: u16 = 0xDB80;

pub const VL53L8CX_UI_CMD_STATUS: u16 = 0x2C00;
pub const VL53L8CX_UI_CMD_START: u16 = 0x2C04;
pub const VL53L8CX_UI_CMD_END: u16 = 0x2FFF;


const L5CX_AMB_SIZE: usize = ((1-VL53L8CX_DISABLE_AMBIENT_PER_SPAD) * 260) as usize;
const L5CX_SPAD_SIZE: usize = ((1-VL53L8CX_DISABLE_NB_SPADS_ENABLED) * 260) as usize;
const L5CX_NTAR_SIZE: usize = ((1-VL53L8CX_DISABLE_NB_TARGET_DETECTED) * 68) as usize;
const L5CX_SPS_SIZE: usize = ((1-VL53L8CX_DISABLE_SIGNAL_PER_SPAD) * 
((256 * VL53L8CX_NB_TARGET_PER_ZONE) + 4)) as usize;
const L5CX_SIGR_SIZE: usize = ((1-VL53L8CX_DISABLE_RANGE_SIGMA_MM) * 
((128 * VL53L8CX_NB_TARGET_PER_ZONE) + 4)) as usize;
const L5CX_DIST_SIZE: usize = ((1-VL53L8CX_DISABLE_DISTANCE_MM) *
((128 * VL53L8CX_NB_TARGET_PER_ZONE) + 4)) as usize;
const L5CX_RFLEST_SIZE: usize = ((1-VL53L8CX_DISABLE_REFLECTANCE_PERCENT) *
((64 * VL53L8CX_NB_TARGET_PER_ZONE) + 4)) as usize;
const L5CX_STA_SIZE: usize = ((1-VL53L8CX_DISABLE_TARGET_STATUS) * 
((64 * VL53L8CX_NB_TARGET_PER_ZONE) + 4)) as usize;
const L5CX_MOT_SIZE: usize = ((1-VL53L8CX_DISABLE_MOTION_INDICATOR) * 144) as usize;

const VL53L8CX_MAX_RESULTS_SIZE: usize = 40 
+ L5CX_AMB_SIZE + L5CX_SPAD_SIZE + L5CX_NTAR_SIZE + L5CX_SPS_SIZE 
+ L5CX_SIGR_SIZE + L5CX_DIST_SIZE + L5CX_RFLEST_SIZE + L5CX_STA_SIZE 
+ L5CX_MOT_SIZE + 20;


pub const VL53L8CX_TEMPORARY_BUFFER_SIZE: usize = 
if 1024 < VL53L8CX_MAX_RESULTS_SIZE { 
    VL53L8CX_MAX_RESULTS_SIZE 
} else {
    1024
};

pub const VL53L8CX_USE_RAW_FORMAT: u8 = 0;

pub const VL53L8CX_NB_TARGET_PER_ZONE: u32 = 1;

// Change the value to 1 to DISABLE the given parameter 
// (All enable by default)
pub const VL53L8CX_DISABLE_AMBIENT_PER_SPAD: u32 = 0;
pub const VL53L8CX_DISABLE_NB_SPADS_ENABLED: u32 = 0;
pub const VL53L8CX_DISABLE_NB_TARGET_DETECTED: u32 = 0;
pub const VL53L8CX_DISABLE_SIGNAL_PER_SPAD: u32 = 0;
pub const VL53L8CX_DISABLE_RANGE_SIGMA_MM: u32 = 0;
pub const VL53L8CX_DISABLE_DISTANCE_MM: u32 = 0;
pub const VL53L8CX_DISABLE_REFLECTANCE_PERCENT: u32 = 0;
pub const VL53L8CX_DISABLE_TARGET_STATUS: u32 = 0;
pub const VL53L8CX_DISABLE_MOTION_INDICATOR: u32 = 0;

// const DEFAULT_I2C_BUFFER_LEN: u16 = 32;


// pub const VL53L8CX_TARGET_ORDER_CLOSEST: u8 = 1;
// pub const VL53L8CX_TARGET_ORDER_STRONGEST: u8 = 2;

// pub const VL53L8CX_RANGING_MODE_CONTINUOUS: u8 = 1;
// pub const VL53L8CX_RANGING_MODE_AUTONOMOUS: u8 = 3;

// pub const VL53L8CX_POWER_MODE_SLEEP: u8 = 0;
// pub const VL53L8CX_POWER_MODE_WAKEUP: u8 = 1;

pub const VL53L8CX_NB_THRESHOLDS: u8 = 64;

pub const VL53L8CX_DCI_DET_THRESH_CONFIG: u16 = 0x5488;
pub const VL53L8CX_DCI_DET_THRESH_GLOBAL_CONFIG: u16 = 0xB6E0;
pub const VL53L8CX_DCI_DET_THRESH_START: u16 = 0xB6E8;
pub const VL53L8CX_DCI_DET_THRESH_VALID_STATUS: u16 = 0xB9F0;

pub const VL53L8CX_DISTANCE_MM: u8 = 1;
pub const VL53L8CX_SIGNAL_PER_SPAD_KCPS: u8 = 2;
pub const VL53L8CX_RANGE_SIGMA_MM: u8 = 4;
pub const VL53L8CX_AMBIENT_PER_SPAD_KCPS: u8 = 8;
pub const VL53L8CX_NB_TARGET_DETECTED: u8 = 9;
pub const VL53L8CX_TARGET_STATUS: u8 = 12;
pub const VL53L8CX_NB_SPADS_ENABLED: u8 = 13;
pub const VL53L8CX_MOTION_INDICATOR: u8 = 19;
