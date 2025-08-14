#![no_std]
#![no_main]


#[allow(unused_imports)]
use defmt::{panic, *};
use {defmt_rtt as _, panic_probe as _};
use embassy_executor::Spawner;
use embassy_stm32::{exti::ExtiInput, gpio::{Input, Level, OutputOpenDrain, Output, Speed}};
use embassy_stm32::gpio::Pull;
use embassy_stm32::time::Hertz;

use embassy_stm32::i2c::{Address, OwnAddresses};

use embassy_stm32::usart::{Config as UsartConfig, DataBits, StopBits, Uart};
use embassy_stm32::spi::{Config as SpiConfig, Mode as SpiMode, Spi, Phase, Polarity};
use embassy_stm32::{bind_interrupts, i2c, peripherals, usart};

mod event_router;
use event_router::{event_router, Router};

mod i2c_device;
use i2c_device::i2c_task;

mod spi_device;
use spi_device::spi_task;

mod channels;
use channels::*;

mod dmx;
use dmx::dmx_task;

// D1 - PB6  - USART1_TX
// D0 - PB7  - USART1_RX
// D2 - PA15 - Doubled up RX for DMX break detection
//
// D4 - PA10 - TIM1_CH3 / I2C1_SDA
// D5 - PA9  - TIM1_CH2 / I2C1_SCL
//
// D10 - PB9 - SPI1_CS(1) / TIM17_CH1
// D11 - PB5 - SPI1_MOSI / TIM3_CH2
// D12 - PB4 - SPI1_MISO
// D13 - PB3 - SPI1_SCK
//
// D7 - PB2 - input pin (device mode select)
// D8 - PB8 - output open drain to pull input low
//
// na - PC6 - LED


bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    I2C1 => i2c::EventInterruptHandler<peripherals::I2C1>, i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

const DEV_ADDR: u8 = 0x42;

#[derive(PartialEq)]
enum DeviceMode {
    I2c,
    Spi
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let led = Output::new(p.PC6, Level::Low, Speed::Medium);

    // -----------------------------------
    // Configure Device Model Selection Pins
    // -----------------------------------
    // D7 = PB2 - input pin
    // D8 = PB8 - output open drain to pull input low
    let mode_inpout_pin = Input::new(p.PB2, Pull::Up);
    let mut mode_output_pin = OutputOpenDrain::new(p.PB8, Level::Low, Speed::Medium);
    mode_output_pin.set_low();
    let device_mode = if mode_inpout_pin.is_high() {
        // high means no jumper between D7 & D8
        DeviceMode::I2c
    } else {
        DeviceMode::Spi
    };


    match device_mode {
        DeviceMode::I2c => {
            // -----------------------------------
            // Configure I2C
            // -----------------------------------
            let mut config = i2c::Config::default();
            config.frequency = Hertz::khz(400);

            let d_addr_config = i2c::SlaveAddrConfig {
                addr: OwnAddresses::OA1(Address::SevenBit(DEV_ADDR)),
                general_call: false,
            };
            let d_sda = p.PA10;
            let d_scl = p.PA9;
            let device =
                i2c::I2c::new(
                    p.I2C1, 
                    d_scl, 
                    d_sda, 
                    Irqs, 
                    p.DMA1_CH1,
                    p.DMA1_CH2,
                    config
                ).into_slave_multimaster(d_addr_config);

            spawner
                .spawn(i2c_task(device, CHANNEL_I2C.receiver(), CHANNEL.sender()))
                .unwrap();
        },
        DeviceMode::Spi => {
            // -----------------------------------
            // Configure SPI
            // -----------------------------------

            let mut spi_config = SpiConfig::default();
            spi_config.frequency = Hertz(3_000_000);
            spi_config.mode = SpiMode { polarity: Polarity::IdleLow, phase: Phase::CaptureOnFirstTransition };

            let spi = Spi::new(
                p.SPI1,
                p.PB3,
                p.PB5,
                p.PB4,
                p.DMA1_CH1,
                p.DMA1_CH2,
                spi_config
            );
            spawner
                .spawn(spi_task(spi, CHANNEL_SPI.receiver(), CHANNEL.sender()))
                .unwrap();
        },
    }


    // -----------------------------------
    // Setup USART for RS485 / DMX
    // https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ApplicationNotes/ApplicationNotes/00001659A.pdf
    // -----------------------------------

    //A data byte is a Start bit, eight data bits and two Stop bits with LSB sent first
    let mut usart_config = UsartConfig::default();
    
    usart_config.baudrate = 250000;
    usart_config.data_bits = DataBits::DataBits9; // set to 9 data bits but we will ignore the start bit
    usart_config.stop_bits = StopBits::STOP2; //StopBits::STOP2;

    // CN10 pin 14 (D1) = p.PG14, CN10 pin 16 (D0) = p.PG9
    let usart = Uart::new(p.USART1, p.PB7, p.PB6, Irqs, p.DMA1_CH3, p.DMA1_CH4, usart_config).unwrap();
    
    // Connect this pin to RX pin so we can detect DMX BREAK and MAB independent of the USART peripheral
    // let dmx_break_pin = ExtiInput::new(p.PD7, p.EXTI7, Pull::None);
    let dmx_break_pin = ExtiInput::new(p.PA15, p.EXTI15, Pull::None);
    spawner
        .spawn(dmx_task(usart, dmx_break_pin, CHANNEL.sender()))
        .unwrap();

    // -----------------------------------
    // Initialize event router
    // -----------------------------------
    info!("Initializing event router.");
    let router = Router::new(CHANNEL.receiver(), device_mode, CHANNEL_I2C.sender(), CHANNEL_SPI.sender());
    spawner.spawn(event_router(router, led)).unwrap();
}