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
use embassy_stm32::usart::{self, BufferedInterruptHandler, BasicInstance, Config as UsartConfig, DataBits, Parity, Uart, UartTx};
use embassy_stm32::Config;
use embassy_time::Delay;
use cortex_m::asm::delay;
use core::fmt::Write;
use heapless::String;
use defmt::*;
use {defmt_rtt as _, panic_probe as _};
// use core::fmt::{self, Write};
// use vl53l8cx::Vl53l8cx;

bind_interrupts!(struct Irqs {
    USART2 => BufferedInterruptHandler<USART2>;
    // I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    // I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[entry]
fn main() -> ! {

    let mut rcc_config = RccConfig::default();
    let hse = Hse {freq: mhz(8), mode: HseMode::Oscillator};
    rcc_config.hse = Some(hse);

    let mut config = Config::default();
    config.rcc = rcc_config;

    let p = embassy_stm32::init(config);

    let mut usart_config = UsartConfig::default();
    usart_config.baudrate = 460800;
    usart_config.data_bits = DataBits::DataBits8;
    usart_config.parity = Parity::ParityNone;

    let mut tx = UartTx::new(
        p.USART2,
        p.PA2, 
        NoDma,
        usart_config).unwrap();


    // let _pwr_pin = Output::new(p.PA7, Level::High, Speed::Low);
    
    // let mut lpn_pin = Output::new(p.PB0, Level::High, Speed::Low);

    // let tim: Delay = Delay {};

    // let i2c = I2c::new(
    //     p.I2C1, 
    //     p.PB8, 
    //     p.PB9, 
    //     Irqs, 
    //     NoDma, 
    //     NoDma, 
    //     khz(400), 
    //     I2cConfig::default());


    // let mut sensor = Vl53l8cx::new_i2c(i2c, lpn_pin, tim).unwrap();

    // sensor.init_sensor(0x29).unwrap();
    // sensor.start_ranging().unwrap();

    let mut n: u8 = 0;
    let mut msg: String<16> = String::new();
    loop {
        // while !sensor.check_data_ready().unwrap() {}
        // let results = sensor.get_ranging_data().unwrap();
        core::writeln!(&mut msg, "hello {}", n).unwrap();
        let _ = tx.blocking_write(msg.as_bytes());
        n += 1;
        msg.clear();
    }
}