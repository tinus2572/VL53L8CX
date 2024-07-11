# VL53L8CX drivers and example applications

This crate provides a platform-agnostic driver for the ST VL53L8CX proximity sensor driver.
The datasheet is available at <https://www.st.com/en/imaging-and-photonics-solutions/VL53L8CX.html>.
The schematics are at <https://www.st.com/resource/en/schematic_pack/x-nucleo-53l8a1-schematic.pdf>.
This driver was built using the [embedded-hal](https://docs.rs/embedded-hal/latest/embedded_hal/) traits.
The [stm32f4xx-hal](https://docs.rs/stm32f4xx-hal/latest/stm32f4xx_hal/) crate is also mandatory.
Ensure that the hardware abstraction layer of your microcontroller implements the embedded-hal traits.

## Instantiating

Create an instance of the driver with the `new_i2c` or `new_spi` associated function, by passing i2c and address instances
or an spi (SpiDevice) instance.
 
### Common setup:
```rust
let dp = Peripherals::take().unwrap();
let rcc = dp.RCC.constrain();
let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();
let tim_top = dp.TIM1.delay_ms(&clocks);

let gpioa = dp.GPIOA.split();
let gpiob = dp.GPIOB.split();

let _pwr_pin = gpioa.pa7.into_push_pull_output_in_state(High);
let lpn_pin = gpiob.pb0.into_push_pull_output_in_state(High);
let tx_pin = gpioa.pa2.into_alternate();
    
let mut tx = dp.USART2.tx(
    tx_pin,
    Config::default()
    .baudrate(460800.bps())
    .wordlength_8()
    .parity_none(),
    &clocks).unwrap();
```

#### I2C setup:
```rust    
let scl = gpiob.pb8;
let sda = gpiob.pb9;

let i2c = I2c1::new(
    dp.I2C1,
    (scl, sda),
    Mode::Standard{frequency:400.kHz()},
    &clocks);
    
let i2c_bus = RefCell::new(i2c);
let address = VL53L8CX_DEFAULT_I2C_ADDRESS;
    
let mut sensor_top = Vl53l8cx::new_i2c(
    RefCellDevice::new(&i2c_bus), 
        lpn_pin,
        tim_top
    ).unwrap();

sensor_top.init_sensor(address).unwrap(); 
sensor_top.start_ranging().unwrap();
```

#### SPI setup:
```rust
let sclk = gpiob.pb3.into_alternate().internal_pull_up(true);
let miso = gpiob.pb4.into_alternate();
let mosi = gpiob.pb5.into_alternate();
let cs_pin = gpiob.pb6.into_push_pull_output_in_state(High);

let spi = Spi::new(
    dp.SPI1,
    (sclk, miso, mosi),
    MODE_3,
    3.MHz(),
    &clocks);
    
let spi = RefCell::new(spi);
let spi = RefCellDevice::new_no_delay(&spi, cs_pin).unwrap();

let mut sensor_top = Vl53l8cx::new_spi(
    spi, 
    lpn_pin,
    tim_top
).unwrap();

sensor.init_sensor().unwrap();
sensor.start_ranging().unwrap();
```

### Common loop:
```rust
loop {
    while !sensor_top.check_data_ready().unwrap() {} // Wait for data to be ready
    let results = sensor_top.get_ranging_data().unwrap(); // Get and parse the result data
    write_results(&mut tx, &results, WIDTH); // Print the result to the output
}    
```

## Multiple instances with I2C

The default I2C address for this device (cf. datasheet) is 0x52.

If multiple sensors are used on the same I2C bus, consider setting off
all the instances, then initializating them one by one to set up unique I2C addresses.

```rust
sensor_top.off().unwrap();
sensor_left.off().unwrap();
sensor_right.off().unwrap();

sensor_top.init_sensor(address_top).unwrap(); 
sensor_left.init_sensor(address_left).unwrap(); 
sensor_right.init_sensor(address_right).unwrap(); 
```