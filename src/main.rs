#![no_std]
#![no_main]

use panic_halt as _; 
use cortex_m_rt::entry;
use bitfield::bitfield;

use core::{fmt::Write, cell::RefCell};

use embedded_hal::{
    i2c::{I2c, SevenBitAddress},
    spi::{SpiDevice, Operation}
};

use stm32f4xx_hal::{
    gpio::{
        Output, 
        Pin, 
        PushPull, 
        PinState::High,
        gpioa, 
        gpiob,
        Alternate}, 
    pac::{USART2, Peripherals, CorePeripherals}, 
    prelude::*, 
    serial::{Config, Tx}, 
    timer::SysDelay,
    rcc::{Rcc, Clocks}
};

#[allow(unused_imports)]
use buffers::*;
use consts::*;
use bus_operation::*;
use motion_indicator::*;
#[allow(unused_imports)]
use detection_thresholds::*;
#[allow(unused_imports)]
use xtalk::*;
#[allow(unused_imports)]
use driver::*;
#[allow(unused_imports)]
use utils::*;
#[allow(unused_imports)]
use accessors::*;

mod buffers;
mod consts;
mod bus_operation;
mod accessors;
mod detection_thresholds;
mod motion_indicator;
mod xtalk;
mod driver;
mod utils;

fn get_gradient_color(number: u32) -> u32 {
    let max_number = 3999;
    // 256-color mode has color codes from 16 to 231 for gradient purposes
    let start_color = 16; // Black in 256-color mode
    let end_color = 231; // White in 256-color mode
    let color_range = end_color - start_color;
    let color_code = start_color + (number * color_range / max_number);
    return color_code;
}



fn write_results(tx: &mut Tx<USART2>, results: &ResultsData, width: usize) {

    // ┼ ├ ┤ ┬ ┴ ┌ └ ┐ ┘ │ ─

    writeln!(tx, "\x1B[2J").unwrap();

    writeln!(tx, "VL53L8A1 Simple Ranging demo application\n").unwrap();
    writeln!(tx, "Cell Format :\n").unwrap();
    writeln!(
        tx, 
        "\x1b[96m{dis:>20}\x1b[0m \x1b[92m{sta:<20}\x1b[0m", 
        dis="Distance [mm]", 
        sta="Status"
    ).unwrap();
    writeln!(
        tx, 
        "\x1b[93m{sig:>20}\x1b[0m \x1b[91m{amb:<20}\x1b[0m", 
        sig="Signal [kcps/spad]", 
        amb="Ambient [kcps/spad]"
    ).unwrap();

    for j in 0..width {
        for i in 0..width {
            write!(
                tx, 
                "\x1b[48;5;{bg}m  \x1b[0m", 
                bg=get_gradient_color(results.distance_mm[width*j+i] as u32),
            ).unwrap();
        } writeln!(tx, "").unwrap();
    } writeln!(tx, "").unwrap();

    for j in 0..width {
        for _ in 0..width { write!(tx, "+--------").unwrap(); } writeln!(tx, "+").unwrap();
        
        #[cfg(not(any(feature="VL53L8CX_DISABLE_DISTANCE_MM", feature="VL53L8CX_DISABLE_TARGET_STATUS")))]
        {
            for i in 0..width {
                write!(
                    tx, 
                    "|\x1b[96m{dis:>5}\x1b[0m \x1b[92m{sta:<2}\x1b[0m", 
                dis=results.distance_mm[width*j+i], 
                sta=results.target_status[width*j+i]
                ).unwrap();
            } write!(tx, "|\n").unwrap();
        }

        #[cfg(not(any(feature="VL53L8CX_DISABLE_SIGNAL_PER_SPAD", feature="VL53L8CX_DISABLE_AMBIENT_PER_SPAD")))]
        {
            for i in 0..width {
                let mut sig: u32 = results.signal_per_spad[width*j+i];
                if sig > 9999 { sig = 9999; }
                write!(
                    tx, 
                    "|\x1b[93m{sig:>5}\x1b[0m \x1b[91m{amb:<2}\x1b[0m", 
                    sig=sig, 
                    amb=results.ambient_per_spad[width*j+i]
                ).unwrap();
            } write!(tx, "|\n").unwrap();
        }
    }
    for _ in 0..width { write!(tx, "+--------").unwrap(); } writeln!(tx, "+").unwrap();

}

const I2C: bool = true;
const SPI: bool = false;
const BUS: bool = SPI;
const ENABLE_THRESHOLD: bool = false;

#[entry]
fn main() -> ! {
    let mut results: ResultsData = ResultsData::new();
    
    let dp: Peripherals = Peripherals::take().unwrap();
    let cp: CorePeripherals = CorePeripherals::take().unwrap();
    let rcc: Rcc = dp.RCC.constrain();
    let frequency = 8.MHz();
    let clocks: Clocks = rcc.cfgr.use_hse(frequency).sysclk(84.MHz()).freeze();
    let delay: SysDelay = cp.SYST.delay(&clocks);

    let gpioa: gpioa::Parts = dp.GPIOA.split();
    let gpiob: gpiob::Parts = dp.GPIOB.split();

    let _pwr_pin: Pin<'A', 7, Output> = gpioa.pa7.into_push_pull_output_in_state(High);
    let lpn_pin: Pin<'B', 0, Output> = gpiob.pb0.into_push_pull_output_in_state(High);
    let tx_pin: Pin<'A', 2, Alternate<7>> = gpioa.pa2.into_alternate();
    
    let mut tx: Tx<USART2> = dp.USART2.tx(
        tx_pin,
        Config::default()
        .baudrate(460800.bps())
        .wordlength_8()
        .parity_none(),
        &clocks).unwrap();
    
    let width: usize = 8;
    let resolution: u8 = (width * width) as u8; 

if BUS == SPI {
        
    use stm32f4xx_hal::spi::Spi;
    use stm32f4xx_hal::pac::SPI1;
    use embedded_hal::spi::MODE_3;
    use embedded_hal_bus::spi::{NoDelay, RefCellDevice};

    let sclk: Pin<'B', 3, Alternate<5>> = gpiob.pb3.into_alternate().internal_pull_up(true);
    let miso: Pin<'B', 4, Alternate<5>> = gpiob.pb4.into_alternate();
    let mosi: Pin<'B', 5, Alternate<5>> = gpiob.pb5.into_alternate();
    let cs_pin: Pin<'B', 6, Output> = gpiob.pb6.into_push_pull_output_in_state(High);

    let spi: Spi<SPI1> = Spi::new(
        dp.SPI1,
        (sclk, miso, mosi),
        MODE_3,
        3.MHz(),
        &clocks,);
        
    let spi: RefCell<Spi<SPI1>> = RefCell::new(spi);
    let spi: RefCellDevice<Spi<SPI1>, Pin<'B', 6, Output>, NoDelay> = RefCellDevice::new_no_delay(&spi, cs_pin).unwrap();
    
    let mut sensor: Vl53l8cx<Vl53l8cxSPI<RefCellDevice<Spi<SPI1>, Pin<'B', 6, Output>, NoDelay>>> = Vl53l8cx::new_spi(
        spi, 
        lpn_pin,
        delay
    ).unwrap();

    sensor.init().unwrap();
    sensor.set_resolution(resolution).unwrap();
    sensor.start_ranging().unwrap();

    write_results(&mut tx, &results, width);
    
    loop {
        while !sensor.check_data_ready().unwrap() {} // Wait for data to be ready
        results = sensor.get_ranging_data().unwrap(); // Get and parse the result data
        write_results(&mut tx, &results, width); // Print the result to the output
    }   

} else if BUS == I2C {

    use stm32f4xx_hal::{
        pac::I2C1,
        i2c::{I2c as StmI2c, I2c1, Mode}};
    use embedded_hal_bus::i2c::RefCellDevice;
    
    let scl: Pin<'B', 8> = gpiob.pb8;
    let sda: Pin<'B', 9> = gpiob.pb9;
    
    let i2c: StmI2c<I2C1> = I2c1::new(
        dp.I2C1,
        (scl, sda),
        Mode::Standard{frequency:400.kHz()},
        &clocks);
        
        let i2c_bus: RefCell<StmI2c<I2C1>> = RefCell::new(i2c);
        let address: SevenBitAddress = VL53L8CX_DEFAULT_I2C_ADDRESS;
        
    let mut sensor = Vl53l8cx::new_i2c(
        RefCellDevice::new(&i2c_bus), 
            address, 
            lpn_pin,
            delay
        ).unwrap();

    sensor.init_sensor(address).unwrap(); 

if ENABLE_THRESHOLD == true {   
    let mut thresholds: [DetectionThresholds; VL53L8CX_NB_THRESHOLDS] = [DetectionThresholds::new(); VL53L8CX_NB_THRESHOLDS];
    for i in 0..resolution as usize {
        thresholds[i].zone_num = i as u8;
        thresholds[i].param_low_thresh = 200;
        thresholds[i].param_high_thresh = 600;
    }
    // thresholds[resolution as usize].zone_num |= VL53L8CX_LAST_THRESHOLD;
    sensor.set_detection_thresholds_enable(0).unwrap(); // Disable thresholds detection
    sensor.set_detection_thresholds(&mut thresholds).unwrap();
    sensor.set_detection_thresholds_enable(1).unwrap();
}

    sensor.set_resolution(resolution).unwrap();
    sensor.start_ranging().unwrap();

    write_results(&mut tx, &results, width);
    
    loop {
        while !sensor.check_data_ready().unwrap() {} // Wait for data to be ready
        results = sensor.get_ranging_data().unwrap(); // Get and parse the result data
        write_results(&mut tx, &results, width); // Print the result to the output
    }

} else { loop {} }
}

