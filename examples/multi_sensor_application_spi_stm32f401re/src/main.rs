#![no_std]
#![no_main]

use vl53l8cx::{
    Vl53l8cx,
    ResultsData
};

use panic_halt as _; 
use cortex_m_rt::entry;

use core::{fmt::Write, cell::RefCell};

use stm32f4xx_hal::{
    gpio::{
        Output, 
        Pin, 
        PinState::High,
        gpioa, 
        gpiob,
        gpioc,
        Alternate}, 
    pac::{USART2, Peripherals, TIM1, TIM2, TIM3}, 
    prelude::*, 
    serial::{Config, Tx}, 
    rcc::{Rcc, Clocks},
    timer::Delay
};

// SPI related imports
use stm32f4xx_hal::spi::Spi;
use stm32f4xx_hal::pac::SPI1;
use embedded_hal::spi::MODE_3;
use embedded_hal_bus::spi::{NoDelay, RefCellDevice};

fn write_results_multi(tx: &mut Tx<USART2>, results_top: &ResultsData, results_left: &ResultsData, results_right: &ResultsData, width: usize) {
    writeln!(tx, "\x1B[2J").unwrap();

    writeln!(tx, "VL53L8A1 Multi Sensor demo application\n").unwrap();
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

    writeln!(tx, "\nTop :").unwrap();
    write_results(tx, results_top, width);
    
    writeln!(tx, "\nLeft :").unwrap();
    write_results(tx, results_left, width);
    
    writeln!(tx, "\nRight :").unwrap();
    write_results(tx, results_right, width);
}

fn write_results(tx: &mut Tx<USART2>, results: &ResultsData, width: usize) {
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

const WIDTH: usize = 4;

#[entry]
fn main() -> ! {
    let mut results_top: ResultsData;
    let mut results_left: ResultsData;
    let mut results_right: ResultsData;
    
    let dp: Peripherals = Peripherals::take().unwrap();
    let rcc: Rcc = dp.RCC.constrain();
    let clocks: Clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();
    
    let tim_top: Delay<TIM1, 1000> = dp.TIM1.delay_ms(&clocks);
    let tim_left: Delay<TIM2, 1000> = dp.TIM2.delay_ms(&clocks);
    let tim_right: Delay<TIM3, 1000> = dp.TIM3.delay_ms(&clocks);
    
    let gpioa: gpioa::Parts = dp.GPIOA.split();
    let gpiob: gpiob::Parts = dp.GPIOB.split();
    let gpioc: gpioc::Parts = dp.GPIOC.split();
    
    let _pwr_pin_top: Pin<'A', 7, Output> = gpioa.pa7.into_push_pull_output_in_state(High);
    let _pwr_pin_left: Pin<'B', 10, Output> = gpiob.pb10.into_push_pull_output_in_state(High);
    let _pwr_pin_right: Pin<'A', 6, Output> = gpioa.pa6.into_push_pull_output_in_state(High);

    let lpn_pin_top: Pin<'B', 0, Output> = gpiob.pb0.into_push_pull_output_in_state(High);
    let lpn_pin_left: Pin<'A', 0, Output> = gpioa.pa0.into_push_pull_output_in_state(High);
    let lpn_pin_right: Pin<'A', 5, Output> = gpioa.pa5.into_push_pull_output_in_state(High);

    let tx_pin: Pin<'A', 2, Alternate<7>> = gpioa.pa2.into_alternate();
    
    let mut tx: Tx<USART2> = dp.USART2.tx(
        tx_pin,
        Config::default()
        .baudrate(460800.bps())
        .wordlength_8()
        .parity_none(),
        &clocks).unwrap();
    
    let resolution: u8 = (WIDTH * WIDTH) as u8;

    let sclk: Pin<'B', 3, Alternate<5>> = gpiob.pb3.into_alternate().internal_pull_up(true);
    let miso: Pin<'B', 4, Alternate<5>> = gpiob.pb4.into_alternate();
    let mosi: Pin<'B', 5, Alternate<5>> = gpiob.pb5.into_alternate();

    let cs_pin_top: Pin<'B', 6, Output> = gpiob.pb6.into_push_pull_output_in_state(High);
    let cs_pin_left: Pin<'C', 1, Output> = gpioc.pc1.into_push_pull_output_in_state(High);
    let cs_pin_right: Pin<'A', 8, Output> = gpioa.pa8.into_push_pull_output_in_state(High);

    let spi: Spi<SPI1> = Spi::new(
        dp.SPI1,
        (sclk, miso, mosi),
        MODE_3,
        3.MHz(),
        &clocks,);
        
    let spi: RefCell<Spi<SPI1>> = RefCell::new(spi);

    let spi_top: RefCellDevice<Spi<SPI1>, Pin<'B', 6, Output>, NoDelay> = RefCellDevice::new_no_delay(&spi, cs_pin_top).unwrap();
    let spi_left: RefCellDevice<Spi<SPI1>, Pin<'C', 1, Output>, NoDelay> = RefCellDevice::new_no_delay(&spi, cs_pin_left).unwrap();
    let spi_right: RefCellDevice<Spi<SPI1>, Pin<'A', 8, Output>, NoDelay> = RefCellDevice::new_no_delay(&spi, cs_pin_right).unwrap();
    
    let mut sensor_top = Vl53l8cx::new_spi(
        spi_top, 
        lpn_pin_top,
        tim_top).unwrap();
    let mut sensor_left = Vl53l8cx::new_spi(
        spi_left, 
        lpn_pin_left,
        tim_left).unwrap();
    let mut sensor_right = Vl53l8cx::new_spi(
        spi_right, 
        lpn_pin_right,
        tim_right).unwrap();

    sensor_top.init_sensor().unwrap();
    sensor_left.init_sensor().unwrap();
    sensor_right.init_sensor().unwrap();
    
    sensor_top.set_resolution(resolution).unwrap();
    sensor_left.set_resolution(resolution).unwrap();
    sensor_right.set_resolution(resolution).unwrap();
    
    sensor_top.start_ranging().unwrap();
    sensor_left.start_ranging().unwrap();
    sensor_right.start_ranging().unwrap();
    
    loop {
        while !sensor_top.check_data_ready().unwrap() {}  // Wait for data to be ready
        results_top = sensor_top.get_ranging_data().unwrap(); // Get and parse the result data

        while !sensor_left.check_data_ready().unwrap() {} // Wait for data to be ready
        results_left = sensor_left.get_ranging_data().unwrap(); // Get and parse the result data

        while !sensor_right.check_data_ready().unwrap() {} // Wait for data to be ready
        results_right = sensor_right.get_ranging_data().unwrap(); // Get and parse the result data

        write_results_multi(
            &mut tx, 
            &results_top, 
            &results_left, 
            &results_right, 
            WIDTH); // Print the result to the output
    }   
}
