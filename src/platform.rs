const VL53L8CX_NB_TARGET_PER_ZONE: u16 = 1;


// Change the value to 1 to DISABLE the given parameter 
// (All enable by default)
const VL53L8CX_DISABLE_AMBIENT_PER_SPAD: u16 = 0;
const VL53L8CX_DISABLE_NB_SPADS_ENABLED: u16 = 0;
const VL53L8CX_DISABLE_NB_TARGET_DETECTED: u16 = 0;
const VL53L8CX_DISABLE_SIGNAL_PER_SPAD: u16 = 0;
const VL53L8CX_DISABLE_RANGE_SIGMA_MM: u16 = 0;
const VL53L8CX_DISABLE_DISTANCE_MM: u16 = 0;
const VL53L8CX_DISABLE_REFLECTANCE_PERCENT: u16 = 0;
const VL53L8CX_DISABLE_TARGET_STATUS: u16 = 0;
const VL53L8CX_DISABLE_MOTION_INDICATOR: u16 = 0;

const DEFAULT_I2C_BUFFER_LEN: u16 = 32;

struct VL53L8CX_Platform {
    address: u16,
    dev_i2c: I2C,
    dev_spi: SPI,
    cs_pin: u32,
    spi_speed: u32,
    lpn_pin: u32,
    i2c_rst_pin: u32,
}