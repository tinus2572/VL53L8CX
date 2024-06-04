use bitfield::bitfield;
use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::{Operation, SpiDevice};

 
 /**
  * @brief Current driver version.
  */
 
  const VL53L8CX_API_REVISION: &str = "VL53L8CX_1.0.4";
 
  /**
   * @brief Default I2C address of VL53L8CX sensor. Can be changed using function
   * set_i2c_address() function is called.
   */
  
  const VL53L8CX_DEFAULT_I2C_ADDRESS: u16 = 0x52;
  
  /**
   * @brief Macro VL53L8CX_RESOLUTION_4X4 or VL53L8CX_RESOLUTION_8X8 allows
   * setting sensor in 4x4 mode or 8x8 mode, using function
   * vl53l8cx_set_resolution().
   */
  
  const VL53L8CX_RESOLUTION_4X4: u8 = 16;
  const VL53L8CX_RESOLUTION_8X8: u8 = 64;
  
  
  /**
   * @brief Macro VL53L8CX_TARGET_ORDER_STRONGEST or VL53L8CX_TARGET_ORDER_CLOSEST
   *  are used to select the target order for data output.
   */
  
  const VL53L8CX_TARGET_ORDER_CLOSEST: u8 = 1;
  const VL53L8CX_TARGET_ORDER_STRONGEST: u8 = 2;
  
  /**
   * @brief Macro VL53L8CX_RANGING_MODE_CONTINUOUS and
   * VL53L8CX_RANGING_MODE_AUTONOMOUS are used to change the ranging mode.
   * Autonomous mode can be used to set a precise integration time, whereas
   * continuous is always maximum.
   */
  
  const VL53L8CX_RANGING_MODE_CONTINUOUS: u8 = 1;
  const VL53L8CX_RANGING_MODE_AUTONOMOUS: u8 = 3;
  
  /**
   * @brief The default power mode is VL53L8CX_POWER_MODE_WAKEUP. User can choose
   * the mode VL53L8CX_POWER_MODE_SLEEP to save power consumption is the device
   * is not used. The low power mode retains the firmware and the configuration.
   * Both modes can be changed using function vl53l8cx_set_power_mode().
   */
  
  const VL53L8CX_POWER_MODE_SLEEP: u8 = 0;
  const VL53L8CX_POWER_MODE_WAKEUP: u8 = 1;
  
  /**
   * @brief Macro VL53L8CX_STATUS_OK indicates that VL53L5 sensor has no error.
   * Macro VL53L8CX_STATUS_ERROR indicates that something is wrong (value,
   * I2C access, ...). Macro VL53L8CX_MCU_ERROR is used to indicate a MCU issue.
   */
  
  const VL53L8CX_STATUS_OK: u8 = 0;
  const VL53L8CX_STATUS_TIMEOUT_ERROR: u8 = 1;
  const VL53L8CX_STATUS_CORRUPTED_FRAME: u8 = 2;
  const VL53L8CX_STATUS_LASER_SAFETY: u8 = 3;
  const VL53L8CX_MCU_ERROR: u8 = 66;
  const VL53L8CX_STATUS_INVALID_PARAM: u8 = 127;
  const VL53L8CX_STATUS_ERROR: u8 = 255;
  
  /**
   * @brief Definitions for Range results block headers
   */
  
  if VL53L8CX_NB_TARGET_PER_ZONE == 1 {
  
    const VL53L8CX_START_BH: u32 = 0x0000000D;
    const VL53L8CX_METADATA_BH: u32 = 0x54B400C0;
    const VL53L8CX_COMMONDATA_BH: u32 = 0x54C00040;
    const VL53L8CX_AMBIENT_RATE_BH: u32 = 0x54D00104;
    const VL53L8CX_SPAD_COUNT_BH: u32 = 0x55D00404;
    const VL53L8CX_NB_TARGET_DETECTED_BH: u32 = 0xDB840401;
    const VL53L8CX_SIGNAL_RATE_BH: u32 = 0xDBC40404;
    const VL53L8CX_RANGE_SIGMA_MM_BH: u32 = 0xDEC40402;
    const VL53L8CX_DISTANCE_BH: u32 = 0xDF440402;
    const VL53L8CX_REFLECTANCE_BH: u32 = 0xE0440401;
    const VL53L8CX_TARGET_STATUS_BH: u32 = 0xE0840401;
    const VL53L8CX_MOTION_DETECT_BH: u32 = 0xD85808C0;
  
    const VL53L8CX_METADATA_IDX: u16 = 0x54B4;
    const VL53L8CX_SPAD_COUNT_IDX: u16 = 0x55D0;
    const VL53L8CX_AMBIENT_RATE_IDX: u16 = 0x54D0;
    const VL53L8CX_NB_TARGET_DETECTED_IDX u16 = 0xDB84;
    const VL53L8CX_SIGNAL_RATE_IDX: u16 = 0xDBC4;
    const VL53L8CX_RANGE_SIGMA_MM_IDX: u16 = 0xDEC4;
    const VL53L8CX_DISTANCE_IDX: u16 = 0xDF44;
    const VL53L8CX_REFLECTANCE_EST_PC_IDX: u16 = 0xE044;
    const VL53L8CX_TARGET_STATUS_IDX: u16 = 0xE084;
    const VL53L8CX_MOTION_DETEC_IDX: u16 = 0xD858;
  
  } else {
    const VL53L8CX_START_BH: u32 = 0x0000000D;
    const VL53L8CX_METADATA_BH: u32 = 0x54B400C0;
    const VL53L8CX_COMMONDATA_BH: u32 = 0x54C00040;
    const VL53L8CX_AMBIENT_RATE_BH: u32 = 0x54D00104;
    const VL53L8CX_NB_TARGET_DETECTED_BH: u32 = 0x57D00401;
    const VL53L8CX_SPAD_COUNT_BH: u32 = 0x55D00404;
    const VL53L8CX_SIGNAL_RATE_BH: u32 = 0x58900404;
    const VL53L8CX_RANGE_SIGMA_MM_BH: u32 = 0x64900402;
    const VL53L8CX_DISTANCE_BH: u32 = 0x66900402;
    const VL53L8CX_REFLECTANCE_BH: u32 = 0x6A900401;
    const VL53L8CX_TARGET_STATUS_BH: u32 = 0x6B900401;
    const VL53L8CX_MOTION_DETECT_BH: u32 = 0xCC5008C0;
  
    const VL53L8CX_METADATA_IDX: u16 = 0x54B4;
    const VL53L8CX_SPAD_COUNT_IDX: u16 = 0x55D0;
    const VL53L8CX_AMBIENT_RATE_IDX: u16 = 0x54D0;
    const VL53L8CX_NB_TARGET_DETECTED_IDX: u16 = 0x57D0;
    const VL53L8CX_SIGNAL_RATE_IDX: u16 = 0x5890;
    const VL53L8CX_RANGE_SIGMA_MM_IDX: u16 = 0x6490;
    const VL53L8CX_DISTANCE_IDX: u16 = 0x6690;
    const VL53L8CX_REFLECTANCE_EST_PC_IDX: u16 = 0x6A90;
    const VL53L8CX_TARGET_STATUS_IDX: u16 = 0x6B90;
    const VL53L8CX_MOTION_DETEC_IDX: u16 = 0xCC50;
   } 
  
  /**
   * @brief Inner Macro for API. Not for user, only for development.
   */
  
  const VL53L8CX_NVM_DATA_SIZE: u16 = 492;
  const VL53L8CX_CONFIGURATION_SIZE: u16 = 972;
  const VL53L8CX_OFFSET_BUFFER_SIZE: u16 = 488;
  const VL53L8CX_XTALK_BUFFER_SIZE: u16 = 776;
  
  const VL53L8CX_DCI_ZONE_CONFIG: u16 = 0x5450;
  const VL53L8CX_DCI_FREQ_HZ: u16 = 0x5458;
  const VL53L8CX_DCI_INT_TIME: u16 = 0x545C;
  const VL53L8CX_DCI_FW_NB_TARGET: u16 = 0x5478;
  const VL53L8CX_DCI_RANGING_MODE: u16 = 0xAD30;
  const VL53L8CX_DCI_DSS_CONFIG: u16 = 0xAD38;
  const VL53L8CX_DCI_TARGET_ORDER: u16 = 0xAE64;
  const VL53L8CX_DCI_SHARPENER: u16 = 0xAED8;
  const VL53L8CX_DCI_INTERNAL_CP: u16 = 0xB39C;
  const VL53L8CX_DCI_SYNC_PIN: u16 = 0xB5F0;
  const VL53L8CX_DCI_MOTION_DETECTOR_CFG: u16 = 0xBFAC;
  const VL53L8CX_DCI_SINGLE_RANGE: u16 = 0xD964;
  const VL53L8CX_DCI_OUTPUT_CONFIG: u16 = 0xD968;
  const VL53L8CX_DCI_OUTPUT_ENABLES: u16 = 0xD970;
  const VL53L8CX_DCI_OUTPUT_LIST: u16 = 0xD980;
  const VL53L8CX_DCI_PIPE_CONTROL: u16 = 0xDB80;
  
  const VL53L8CX_UI_CMD_STATUS: u16 = 0x2C00;
  const VL53L8CX_UI_CMD_START: u16 = 0x2C04;
  const VL53L8CX_UI_CMD_END: u16 = 0x2FFF;
  
  /**
   * @brief Inner values for API. Max buffer size depends of the selected output.
   */
  
   const L5CX_AMB_SIZE: u16 = VL53L8CX_DISABLE_AMBIENT_PER_SPAD * 260;
   const L5CX_SPAD_SIZE: u16 = VL53L8CX_DISABLE_NB_SPADS_ENABLED * 260;
   const L5CX_NTAR_SIZE: u16 = VL53L8CX_DISABLE_NB_TARGET_DETECTED * 68;
   const L5CX_SPS_SIZE: u16 = VL53L8CX_DISABLE_SIGNAL_PER_SPAD * 
       ((256 * VL53L8CX_NB_TARGET_PER_ZONE) + 4);
   const L5CX_SIGR_SIZE: u16 = VL53L8CX_DISABLE_RANGE_SIGMA_MM * 
       ((128 * VL53L8CX_NB_TARGET_PER_ZONE) + 4);
   const L5CX_DIST_SIZE: u16 = VL53L8CX_DISABLE_DISTANCE_MM *
       ((128 * VL53L8CX_NB_TARGET_PER_ZONE) + 4);
   const L5CX_RFLEST_SIZE: u16 = VL53L8CX_DISABLE_REFLECTANCE_PERCENT *
       ((64 * VL53L8CX_NB_TARGET_PER_ZONE) + 4);
   const L5CX_STA_SIZE: u16 = VL53L8CX_DISABLE_TARGET_STATUS * 
       ((64 * VL53L8CX_NB_TARGET_PER_ZONE) + 4);
   const L5CX_MOT_SIZE: u16 = VL53L8CX_DISABLE_MOTION_INDICATOR * 144;
 
  /**
   * @brief Macro VL53L8CX_MAX_RESULTS_SIZE indicates the maximum size used by
   * output through I2C. Value 40 corresponds to headers + meta-data + common-data
   * and 20 corresponds to the footer.
   */
  
  const VL53L8CX_MAX_RESULTS_SIZE: u16 = ( 40 \
    + L5CX_AMB_SIZE + L5CX_SPAD_SIZE + L5CX_NTAR_SIZE + L5CX_SPS_SIZE \
    + L5CX_SIGR_SIZE + L5CX_DIST_SIZE + L5CX_RFLEST_SIZE + L5CX_STA_SIZE \
    + L5CX_MOT_SIZE + 20);
  
  /**
   * @brief Macro VL53L8CX_TEMPORARY_BUFFER_SIZE can be used to know the size of
   * the temporary buffer. The minimum size is 1024, and the maximum depends of
   * the output configuration.
   */
     
  if VL53L8CX_MAX_RESULTS_SIZE < 1024 {
    const VL53L8CX_TEMPORARY_BUFFER_SIZE: u32 = 1024;
  } else {
    const VL53L8CX_TEMPORARY_BUFFER_SIZE: u32 = VL53L8CX_MAX_RESULTS_SIZE;
  }
  
  
  /**
   * @brief Structure VL53L8CX_Configuration contains the sensor configuration.
   * User MUST not manually change these field, except for the sensor address.
   */
  
 struct VL53L8CX_Configuration {
    /* Platform, filled by customer into the 'platform.h' file */
    platform: VL53L8CX_Platform,
    /* Results streamcount, value auto-incremented at each range */
    streamcount: u8,
    /* Size of data read though I2C */
    data_read_size: u32,
    /* Address of default configuration buffer */
    default_configuration: u8,
    /* Address of default Xtalk buffer */
    default_xtalk: u8,
    /* Offset buffer */
    offset_data: [u8, VL53L8CX_OFFSET_BUFFER_SIZE],
    /* Xtalk buffer */
    xtalk_data: [u8, VL53L8CX_XTALK_BUFFER_SIZE],
    /* Temporary buffer used for internal driver processing */
    temp_buffer: [u8, VL53L8CX_TEMPORARY_BUFFER_SIZE],
    /* Auto-stop flag for stopping the sensor */
    is_auto_stop_enabled: u8,  
  }
  
  
  /**
   * @brief Structure VL53L8CX_ResultsData contains the ranging results of
   * VL53L8CX. If user wants more than 1 target per zone, the results can be split
   * into 2 sub-groups :
   * - Per zone results. These results are common to all targets (ambient_per_spad
   * , nb_target_detected and nb_spads_enabled).
   * - Per target results : These results are different relative to the detected
   * target (signal_per_spad, range_sigma_mm, distance_mm, reflectance,
   * target_status).
   */
  
  struct VL53L8CX_ResultsData {
    /* Internal sensor silicon temperature */
    silicon_temp_degc: i8,
  
    /* Ambient noise in kcps/spads */
    ambient_per_spad: [u32, VL53L8CX_RESOLUTION_8X8],
  
    /* Number of valid target detected for 1 zone */
    nb_target_detected[u8, VL53L8CX_RESOLUTION_8X8],
  
    /* Number of spads enabled for this ranging */
    nb_spads_enabled[u32, VL53L8CX_RESOLUTION_8X8],
  
    /* Signal returned to the sensor in kcps/spads */
    signal_per_spad[u32, (VL53L8CX_RESOLUTION_8X8 * VL53L8CX_NB_TARGET_PER_ZONE)],
  
    /* Sigma of the current distance in mm */
    range_sigma_mm[u16, (VL53L8CX_RESOLUTION_8X8 * VL53L8CX_NB_TARGET_PER_ZONE)],
  
    /* Measured distance in mm */
    distance_mm[u16, (VL53L8CX_RESOLUTION_8X8 * VL53L8CX_NB_TARGET_PER_ZONE)],
  
    /* Estimated reflectance in percent */
    reflectance[u8, (VL53L8CX_RESOLUTION_8X8 * VL53L8CX_NB_TARGET_PER_ZONE)],
  
    /* Status indicating the measurement validity (5 & 9 means ranging OK)*/
    target_status[u8, (VL53L8CX_RESOLUTION_8X8 * VL53L8CX_NB_TARGET_PER_ZONE)],
  
    /* Motion detector results */
    struct motion_indicator {
      global_indicator_1: u32,
      global_indicator_2: u32,
      status: u8,
      nb_of_detected_aggregates: u8,
      nb_of_aggregates: u8,
      spare: u8,
      motion: [u8, 32],
    } 
  
  } 
  
  
  union Block_header {
    bytes: u32,
    struct {
      type: u32,
      size: u32,
      idx: u32,
    }
  }
 
  const VL53L8CX_NB_TARGET_PER_ZONE: u16 = 1;


  // Change the value to 1 to DISABLE the given parameter 
  // (All enable by default)
  const VL53L8CX_DISABLE_AMBIENT_PER_SPAD: u32 = 0;
  const VL53L8CX_DISABLE_NB_SPADS_ENABLED: u32 = 0;
  const VL53L8CX_DISABLE_NB_TARGET_DETECTED: u32 = 0;
  const VL53L8CX_DISABLE_SIGNAL_PER_SPAD: u32 = 0;
  const VL53L8CX_DISABLE_RANGE_SIGMA_MM: u32 = 0;
  const VL53L8CX_DISABLE_DISTANCE_MM: u32 = 0;
  const VL53L8CX_DISABLE_REFLECTANCE_PERCENT: u32 = 0;
  const VL53L8CX_DISABLE_TARGET_STATUS: u32 = 0;
  const VL53L8CX_DISABLE_MOTION_INDICATOR: u32 = 0;
  
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

  
pub struct Vl53l8cxI2C<P> {
    i2c: P,
    address: SevenBitAddress,
}

impl<P: I2c> Vl53l8cxI2C<P> {
    pub(super) fn new(i2c: P, address: SevenBitAddress) -> Self {
        Self { i2c, address }
    }
}

pub trait BusOperation {
    type Error;

    fn read_bytes(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error>;
    fn write_bytes(&mut self, wbuf: &[u8]) -> Result<(), Self::Error>;
    fn write_byte_read_bytes(&mut self, wbuf: &[u8; 1], rbuf: &mut [u8])
        -> Result<(), Self::Error>;
}

impl<P: I2c> BusOperation for Vl53l8cxI2C<P> {
    type Error = P::Error;

    #[inline]
    fn read_bytes(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.read(self.address, rbuf)?;

        Ok(())
    }

    #[inline]
    fn write_bytes(&mut self, wbuf: &[u8]) -> Result<(), Self::Error> {
        self.i2c.write(self.address, wbuf)?;

        Ok(())
    }

    #[inline]
    fn write_byte_read_bytes(
        &mut self,
        wbuf: &[u8; 1],
        rbuf: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, wbuf, rbuf)?;

        Ok(())
    }
}

#![no_std]

use embedded_hal::i2c::{I2c, SevenBitAddress};

pub struct Vl53l8cx<B: BusOperation> {
    bus: B,
}

impl<P> Vl53l8cx<Vl53l8cxI2C<P>> {
    where
    P: I2c,
{
    pub fn new_i2c(i2c: P, address: I2CAddress) -> Result<Self, Error<P::Error>> {
        let bus = lps22df_reg::Lps22dfI2C::new(i2c, address as SevenBitAddress);
        let mut instance = Self { bus };
        let who = instance.who_am_i_get()?;
        if who != 0xB4 {
            return Err(Error::WhoAmIError(who));
        }
        instance.ctrl_reg2_set_boot()?;
        while instance.int_source_get_boot_on()? != 0 {}
        instance.ctrl_reg2_set_swreset()?;
        while instance.ctrl_reg2_get_swreset()? != 0 {}

        Ok(instance)
    }
}

impl<B: BusOperation> Vl53l8cx<B> {
    #[inline]
    fn read_from_register(&mut self, reg: u16, buf: &mut [u8]) -> Result<(), Error<B::Error>> {
        let a: u8 = (reg >> 8).try_into().unwrap();
        let b: u8 = (reg & 0xFF).try_into().unwrap(); 
        self.bus
            .write_byte_read_bytes(&[a, b], buf)
            .map_err(Error::Bus)?;

        Ok(())
    }

    #[inline]
    fn write_to_register(&mut self, reg: u16, val: u8) -> Result<(), Error<B::Error>> {
        let a: u8 = (reg >> 8).try_into().unwrap();
        let b: u8 = (reg & 0xFF).try_into().unwrap(); 
        self.bus
            .write_bytes(&[a, b, val])
            .map_err(Error::Bus)?;
        let mut arr: [u8; 1] = [0];
        self.read_from_register(reg, &mut arr)?;
        if arr[0] != val {
            return Err(Error::WriteFailure);
        }

        Ok(())
    }
    
    pub fn set_i2c_address(&mut self, i2c_address: u16) -> Result<(), Error<B::Error>> {
        self.write_to_register(0x7fff, 0x00)?;
        self.write_to_register(0x4, (uint8_t)(i2c_address >> 1))?;
        address = i2c_address;
        self.write_to_register(0x7fff, 0x02)?;
        
        Ok(())
    }

    pub fn is_alive(&mut self) -> Result<(), Error<B::Error>> {
        let device_id: [u8, 1] = [0];
        let revision_id: [u8, 1] = [0];
        self.write_to_register(0x7fff, 0x00)?;
        self.read_from_register(0, &mut device_id)?;
        self.read_from_register(1, &mut revision_id)?;
        self.write_to_register(0x7fff, 0x02)?;
        if (device_id == 0xF0 as u8) && (revision_id == 0x0C as u8) {
            return Ok(());
          } else {
            return Error();
          }
    }

}

uint8_t status = VL53L8CX_STATUS_OK;
uint8_t device_id, revision_id;
status |= WrByte(&(p_dev->platform), 0x7fff, 0x00);
status |= RdByte(&(p_dev->platform), 0, &device_id);
status |= RdByte(&(p_dev->platform), 1, &revision_id);
status |= WrByte(&(p_dev->platform), 0x7fff, 0x02);
if ((device_id == (uint8_t)0xF0) && (revision_id == (uint8_t)0x0C)) {
  *p_is_alive = 1;
} else {
  *p_is_alive = 0;
}