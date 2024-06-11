#![no_std]
#![no_main]
#[deny(unsafe_code)]

// pick a panicking behavior
use panic_halt as _; 

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use cortex_m::asm::wfi;

use core::cell::RefCell;
use core::fmt::Write;

use stm32f4xx_hal::{
    gpio::{self, Edge, Input},
    i2c::{I2c1, Mode},
    pac::{self, interrupt},
    prelude::*,
    serial::Config,
};

use embedded_hal_bus::i2c;

use crate::vl53l8cx::*;
use crate::buffers::*;
use crate::consts::*;

mod vl53l8cx;
mod buffers;
mod consts;



type IntPin = gpio::PB0<Input>;

static INT_PIN: Mutex<RefCell<Option<IntPin>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
    //let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();

    let tim1 = dp.TIM1.delay_us(&clocks);
    let tim2 = dp.TIM2.delay_us(&clocks);

    let gpiob = dp.GPIOB.split();
    let gpioa = dp.GPIOA.split();

    let scl = gpiob.pb8;
    let sda = gpiob.pb9;

    let mut int_pin = gpiob.pb0.into_input();
    // Configure Pin for Interrupts
    // 1) Promote SYSCFG structure to HAL to be able to configure interrupts
    let mut syscfg = dp.SYSCFG.constrain();
    // 2) Make an interrupt source
    int_pin.make_interrupt_source(&mut syscfg);
    // 3) Make an interrupt source
    int_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    // 4) Enable gpio interrupt
    int_pin.enable_interrupt(&mut dp.EXTI);

    // Enable the external interrupt in the NVIC by passing the interrupt number
    unsafe {
        cortex_m::peripheral::NVIC::unmask(int_pin.interrupt());
    }

    // Now that pin is configured, move pin into global context
    cortex_m::interrupt::free(|cs| {
        INT_PIN.borrow(cs).replace(Some(int_pin));
    });

    let i2c = I2c1::new(
        dp.I2C1,
        (scl, sda),
        Mode::Standard { frequency:  100.kHz() },
        &clocks,
    );

    let i2c_bus = RefCell::new(i2c);

    let tx_pin = gpioa.pa2.into_alternate();

    let mut tx = dp
        .USART2
        .tx(
            tx_pin,
            Config::default()
                .baudrate(115200.bps())
                .wordlength_8()
                .parity_none(),
            &clocks,
        )
        .unwrap();

    writeln!(tx, "demo2").unwrap();

    let mut vl53l8cx = Vl53l8cx::new_i2c(i2c::RefCellDevice::new(&i2c_bus), 0x52, ).unwrap();

    lis2duxs12.power_up().unwrap();

    lis2duxs12.set_full_scale(4).unwrap();

    lis2duxs12
        .enable_basic_interrupts(SignalMode::PulsedLevel)
        .unwrap();

    lis2duxs12.set_odr(400.0, OperatingMode::LowPower).unwrap();

    lis2duxs12
        .set_tap_params(
            TapAxis::Zaxis,
            125.0,
            (35.0, 25.0),
            1000.0,
            10.0,
            500.0,
            (15.0, 80.0),
            5.0,
            false,
            true,
            false,
            320.0,
            false,
        )
        .unwrap();
    
    lis2duxs12
        .set_sleep_params(true, InactivityOdr::Hz3, 125.0, 2560.0, 0.0)
        .unwrap();

    lis2duxs12.enable_tap_to_pin(Pin::Int1).unwrap();
    
    let mut lps22df = Lps22df::new_i2c(i2c::RefCellDevice::new(&i2c_bus), Lps22dfAddress::Address1).unwrap();

    lps22df.set_odr(0).unwrap();

    let mut sht40 = Sht4x::new_i2c(i2c::RefCellDevice::new(&i2c_bus), Sht4xAddress::Address0, tim2).unwrap();

    loop {
        wfi();
        lps22df.one_shot().unwrap();
        let values = lps22df.get_values().unwrap();
        let humi = sht40.get_values_high_precision().unwrap().1;
        writeln!(tx, "pressure: {} hPa, temperature: {} C, humidity: {} %", values.0, values.1, humi).unwrap();
    }
}

#[interrupt]
fn EXTI0() {
    // Start a Critical Section
    cortex_m::interrupt::free(|cs| {
        // Obtain Access to Global Data
        //INTERRUPT.borrow(cs).set(true);
        // Obtain access to Peripheral and Clear Interrupt Pending Flag
        let mut int_pin = INT_PIN.borrow(cs).borrow_mut();
        int_pin.as_mut().unwrap().clear_interrupt_pending_bit();
    });
}


















