
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embassy_stm32::gpio::{Level, Speed, Output, Input, Pin};
use embassy_stm32::gpio::{AnyPin, OutputOpenDrain};
use embassy_stm32::i2c::{I2c, Config as I2cConfig};
use embassy_stm32::usart::{BufferedUartTx, Config as UartConfig, UartTx};
use embassy_stm32::time::mhz;

use embassy_stm32::peripherals::USART2;
use embassy_stm32::Peripherals;
use embassy_time::{Duration, Timer};
use embedded_hal::delay::{DelayNs};
use embassy_stm32::rcc::{Clocks};
use embassy_stm32::time::Hertz;
use vl53l8cx::{Vl53l8cx, consts::VL53L8CX_DEFAULT_I2C_ADDRESS, ResultsData};
use core::{fmt::Write, cell::RefCell};


// fn write_results(tx: &mut BufferedUartTx<'_, USART2>, results: &ResultsData, width: usize) {    
//     write!(tx, "\x1B[2H").unwrap();

//     writeln!(tx, "VL53L8A1 Simple Ranging demo application\n").unwrap();

//     writeln!(tx, "Cell Format :\n").unwrap();
//     writeln!(
//         tx, 
//         "\x1b[96m{dis:>20}\x1b[0m \x1b[92m{sta:<20}\x1b[0m", 
//         dis="Distance [mm]", 
//         sta="Status"
//     ).unwrap();
//     writeln!(
//         tx, 
//         "\x1b[93m{sig:>20}\x1b[0m \x1b[91m{amb:<20}\x1b[0m", 
//         sig="Signal [kcps/spad]", 
//         amb="Ambient [kcps/spad]"
//     ).unwrap();

//     for j in 0..width {
//         for _ in 0..width { write!(tx, "+----------").unwrap(); } writeln!(tx, "+").unwrap();
        
//         #[cfg(not(any(feature="VL53L8CX_DISABLE_DISTANCE_MM", feature="VL53L8CX_DISABLE_TARGET_STATUS")))]
//         {
//             for i in 0..width {
//                 write!(
//                     tx, 
//                     "|\x1b[96m{dis:>5}\x1b[0m \x1b[92m{sta:<4}\x1b[0m", 
//                 dis=results.distance_mm[width*j+i], 
//                 sta=results.target_status[width*j+i]
//                 ).unwrap();
//             } write!(tx, "|\n").unwrap();
//         }

//         #[cfg(not(any(feature="VL53L8CX_DISABLE_SIGNAL_PER_SPAD", feature="VL53L8CX_DISABLE_AMBIENT_PER_SPAD")))]
//         {
//             for i in 0..width {
//                 let mut sig: u32 = results.signal_per_spad[width*j+i];
//                 if sig > 9999 { sig = 9999; }
//                 write!(
//                     tx, 
//                     "|\x1b[93m{sig:>5}\x1b[0m \x1b[91m{amb:<4}\x1b[0m", 
//                     sig=sig, 
//                     amb=results.ambient_per_spad[width*j+i]
//                 ).unwrap();
//             } write!(tx, "|\n").unwrap();
//         }
//     }
//     for _ in 0..width { write!(tx, "+----------").unwrap(); } writeln!(tx, "+").unwrap();
// }


#[entry]
fn main() -> ! {
   // Safely take ownership of device peripherals
   let p = Peripherals::take().unwrap();

   // Configure RCC (Reset and Clock Control)
   let rcc = Rcc::new(p.RCC);
   let clocks = rcc.cfgr()
       .use_hse(Hertz(8_000_000))  // Use HSE at 8 MHz
       .sysclk(Hertz(48_000_000))  // Set system clock to 48 MHz
       .freeze();

   // Initialize the timer (TIM1) with the current clock settings
   let mut tim_top = Timer::tim1(p.TIM1, clocks);

    let mut results: ResultsData = ResultsData::new();

    // UART setup
    // let tx_pin = p.PA2;

    // let mut tx = UartTx::new(
    //     p.USART2,
    //     tx_pin,
    //     Default::default(),
    //     UartConfig::default()
    // );

    // I2C setup
    let scl = dp.PB8;
    let sda = dp.PB9;

    let i2c = I2c::new(
        dp.I2C1,
        scl,
        sda,
        I2cConfig::default()
    );

    let lpn_pin = Output::new(
        dp.PB0, 
        Level::High, 
        Speed::Low);
        
    let lpn_pin = Output::new(
        dp.PA7, 
        Level::High, 
        Speed::Low);    

    let delay = Timer::after(Duration::from_millis(100));

    let mut tim_top = Timer::new_tim1(dp.TIM1, rcc.clocks);

    let address = VL53L8CX_DEFAULT_I2C_ADDRESS;

    let mut sensor_top = Vl53l8cx::new_i2c(
        i2c, 
        lpn_pin, 
        tim_top).unwrap();

    sensor_top.init_sensor(address).unwrap(); 
    sensor_top.set_frequency_hz(30).unwrap();
    sensor_top.start_ranging().unwrap();

    loop {
        while !sensor_top.check_data_ready().unwrap() {} // Wait for data to be ready
        results = sensor_top.get_ranging_data().unwrap(); // Get and parse the result data
        // write_results(&mut tx, &results, 4); // Print the result to the output
    }
}
