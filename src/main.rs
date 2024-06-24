#![no_std]
#![no_main]

use panic_halt as _; 
use cortex_m_rt::entry;
use bitfield::bitfield;

use core::{fmt::Write, cell::RefCell};

use embedded_hal::{
    i2c::SevenBitAddress,
    spi::{SpiDevice, Operation}
};

use embedded_hal_bus::i2c::RefCellDevice;

use stm32f4xx_hal::{
    gpio::{
        Output, 
        Pin, 
        PushPull, 
        PinState::High,
        gpioa, 
        gpiob,
        Alternate}, 
    i2c::{I2c, I2c1, Mode}, 
    pac::{I2C1, USART2, Peripherals, CorePeripherals}, 
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

mod buffers;
mod consts;
mod bus_operation;
mod detection_thresholds;
mod motion_indicator;
mod xtalk;
mod driver;

fn write_results(tx: &mut Tx<USART2>, results: &ResultsData, width: usize) {

    // ┼ ├ ┤ ┬ ┴ ┌ └ ┐ ┘ │ ─

    writeln!(tx, "\x1B[2J").unwrap();

    writeln!(tx, "VL53L8A1 Simple Ranging demo application\n").unwrap();
    writeln!(tx, "Cell Format :\n").unwrap();
    writeln!(
        tx, 
        "\x1b[96m{dis:>20}\x1b[0m : \x1b[92m{sta:<20}\x1b[0m", 
        dis="Distance [mm]", 
        sta="Status"
    ).unwrap();
    writeln!(
        tx, 
        "\x1b[93m{sig:>20}\x1b[0m : \x1b[91m{amb:<20}\x1b[0m", 
        sig="Signal [kcps/spad]", 
        amb="Ambient [kcps/spad]"
    ).unwrap();

    for j in 0..width {
        for _ in 0..width { write!(tx, "+-----------").unwrap(); } writeln!(tx, "+").unwrap();
        
        for i in 0..width {
            write!(
                tx, 
                "|\x1b[96m{dis:>4}\x1b[0m : \x1b[92m{sta:<4}\x1b[0m", 
                dis=results.distance_mm[width*j+i], 
                sta=results.target_status[width*j+i]
            ).unwrap();
        } write!(tx, "|\n").unwrap();

        for i in 0..width {
            write!(
                tx, 
                "|\x1b[93m{sig:>4}\x1b[0m : \x1b[91m{amb:<4}\x1b[0m", 
                sig=results.signal_per_spad[width*j+i], 
                amb=results.ambient_per_spad[width*j+i]
            ).unwrap();
        } write!(tx, "|\n").unwrap();
    }
    for _ in 0..width { write!(tx, "+-----------").unwrap(); } writeln!(tx, "+").unwrap();

}

#[entry]
fn main() -> ! {
    let mut results: ResultsData = ResultsData::new();
    
    
    let dp: Peripherals = Peripherals::take().unwrap();
    let cp: CorePeripherals = CorePeripherals::take().unwrap();
    let rcc: Rcc = dp.RCC.constrain();
    let clocks: Clocks = rcc.cfgr.use_hse(8.MHz()).freeze();
    let gpioa: gpioa::Parts = dp.GPIOA.split();
    let gpiob: gpiob::Parts = dp.GPIOB.split();
    let _pwr_pin: Pin<'A', 7, Output> = gpioa.pa7.into_push_pull_output_in_state(High);
    let lpn_pin: Pin<'B', 0, Output> = gpiob.pb0.into_push_pull_output_in_state(High);
    let tx_pin: Pin<'A', 2, Alternate<7>> = gpioa.pa2.into_alternate();
    let scl: Pin<'B', 8> = gpiob.pb8;
    let sda: Pin<'B', 9> = gpiob.pb9;
    let delay: SysDelay = cp.SYST.delay(&clocks);
    
    let mut tx: Tx<USART2> = dp.USART2.tx(
        tx_pin,
        Config::default()
        .baudrate(115200.bps())
        .wordlength_8()
        .parity_none(),
        &clocks).unwrap(); 
    
    let i2c: I2c<I2C1> = I2c1::new(
        dp.I2C1,
        (scl, sda),
        Mode::Standard{frequency:100.kHz()},
        &clocks);
    
    let width: usize = 4;
    write_results(&mut tx, &results, width);

    let i2c_bus: RefCell<I2c<I2C1>> = RefCell::new(i2c);
    let address: SevenBitAddress = VL53L8CX_DEFAULT_I2C_ADDRESS;
    let i2c_rst_pin: i8 = -1;

    let mut sensor: Vl53l8cx<Vl53l8cxI2C<RefCellDevice<I2c<I2C1>>>> = Vl53l8cx::new_i2c(
        RefCellDevice::new(&i2c_bus), 
        address, 
        lpn_pin, 
        i2c_rst_pin, 
        delay
    ).unwrap();

    sensor.init_sensor(VL53L8CX_DEFAULT_I2C_ADDRESS).unwrap();
    sensor.start_ranging().unwrap();
    
    let mut ready: u8 = 0;
    
    loop {
        while ready == 0 {
            ready = sensor.check_data_ready().unwrap();
        }
        results = sensor.get_ranging_data().unwrap();
        write_results(&mut tx, &results, width);
        sensor.delay(1000);
    }
}


