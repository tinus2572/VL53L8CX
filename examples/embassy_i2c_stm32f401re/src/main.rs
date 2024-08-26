#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{self, Config as I2cConfig, I2c};
use embassy_stm32::peripherals::{self, USART2};
use embassy_stm32::rcc::{Config as RccConfig, Hse, HseMode};
use embassy_stm32::time::{khz, mhz};
use embassy_stm32::usart::{BufferedInterruptHandler, Config as UsartConfig, DataBits, Parity, UartTx};
use embassy_stm32::Config;
use embassy_time::Delay;
use embedded_hal::delay::DelayNs;
use core::fmt::Write;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};
use vl53l8cx::{Vl53l8cx, ResultsData, consts::VL53L8CX_DEFAULT_I2C_ADDRESS};

bind_interrupts!(struct Irqs {
    USART2 => BufferedInterruptHandler<USART2>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});


fn write_results(tx: &mut UartTx<USART2>, results: &ResultsData, width: usize) {
    let mut msg: String<64> = String::new();
    writeln!(msg, "\x1B[2H").unwrap();
    let _ = tx.blocking_write(msg.as_bytes());
    msg.clear();
    for i in 0..width {
        for j in 0..width {
            write!(&mut msg, "{:>5} ", results.distance_mm[width*i+j]).unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }
        write!(&mut msg, "\n").unwrap();
        let _ = tx.blocking_write(msg.as_bytes());
        msg.clear();
    }
    
    // writeln!(tx, "VL53L8A1 Simple Ranging demo application\n").unwrap();

    // writeln!(tx, "Cell Format :\n").unwrap();
    // writeln!(
    //     tx, 
    //     "\x1b[96m{dis:>20}\x1b[0m \x1b[92m{sta:<20}\x1b[0m", 
    //     dis="Distance [mm]", 
    //     sta="Status"
    // ).unwrap();
    // writeln!(
    //     tx, 
    //     "\x1b[93m{sig:>20}\x1b[0m \x1b[91m{amb:<20}\x1b[0m", 
    //     sig="Signal [kcps/spad]", 
    //     amb="Ambient [kcps/spad]"
    // ).unwrap();

    // for j in 0..width {
    //     for _ in 0..width { write!(tx, "+----------").unwrap(); } writeln!(tx, "+").unwrap();
        
    //     #[cfg(not(any(feature="VL53L8CX_DISABLE_DISTANCE_MM", feature="VL53L8CX_DISABLE_TARGET_STATUS")))]
    //     {
    //         for i in 0..width {
    //             write!(
    //                 tx, 
    //                 "|\x1b[96m{dis:>5}\x1b[0m \x1b[92m{sta:<4}\x1b[0m", 
    //             dis=results.distance_mm[width*j+i], 
    //             sta=results.target_status[width*j+i]
    //             ).unwrap();
    //         } write!(tx, "|\n").unwrap();
    //     }

    //     #[cfg(not(any(feature="VL53L8CX_DISABLE_SIGNAL_PER_SPAD", feature="VL53L8CX_DISABLE_AMBIENT_PER_SPAD")))]
    //     {
    //         for i in 0..width {
    //             let mut sig: u32 = results.signal_per_spad[width*j+i];
    //             if sig > 9999 { sig = 9999; }
    //             write!(
    //                 tx, 
    //                 "|\x1b[93m{sig:>5}\x1b[0m \x1b[91m{amb:<4}\x1b[0m", 
    //                 sig=sig, 
    //                 amb=results.ambient_per_spad[width*j+i]
    //             ).unwrap();
    //         } write!(tx, "|\n").unwrap();
    //     }
    // }
    // for _ in 0..width { write!(tx, "+----------").unwrap(); } writeln!(tx, "+").unwrap();

}


#[entry]
fn main() -> ! {

    let mut rcc_config: RccConfig = RccConfig::default();
    let hse: Hse = Hse {freq: mhz(8), mode: HseMode::Oscillator};
    rcc_config.hse = Some(hse);

    let mut config: Config = Config::default();
    config.rcc = rcc_config;

    let p = embassy_stm32::init(config);

    let mut usart_config: UsartConfig = UsartConfig::default();
    usart_config.baudrate = 460800;
    usart_config.data_bits = DataBits::DataBits8;
    usart_config.parity = Parity::ParityNone;

    let mut tx: UartTx<_> = UartTx::new(
        p.USART2,
        p.PA2, 
        NoDma,
        usart_config).unwrap();


    let _pwr_pin: Output<_> = Output::new(p.PA7, Level::High, Speed::Low);
    
    let lpn_pin: Output<_> = Output::new(p.PB0, Level::High, Speed::Low);

    let mut tim: Delay = Delay {};

    let i2c: I2c<_> = I2c::new(
        p.I2C1, 
        p.PB8, 
        p.PB9, 
        Irqs, 
        NoDma, 
        NoDma, 
        khz(100), 
        I2cConfig::default());


    let mut sensor = Vl53l8cx::new_i2c(i2c, lpn_pin, tim.clone()).unwrap();

    sensor.init_sensor(VL53L8CX_DEFAULT_I2C_ADDRESS).unwrap();
    sensor.start_ranging().unwrap();

    loop {
        while !sensor.check_data_ready().unwrap() {}
        let results = sensor.get_ranging_data().unwrap();
        write_results(&mut tx, &results, 4);
        tim.delay_ms(500);
    }
}