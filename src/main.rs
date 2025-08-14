#![no_std]
#![no_main]


#[allow(unused_imports)]
use defmt::{panic, *};
use embassy_time::Instant;
use crate::event_router::RouterEvent;

use {defmt_rtt as _, panic_probe as _};
use embassy_executor::Spawner;
use embassy_stm32::{exti::ExtiInput, gpio::{Input, Level, Output, OutputOpenDrain, Speed}, usart::{BufferedUart, Parity}};
use embassy_stm32::gpio::Pull;
use embassy_stm32::time::Hertz;

use embedded_io_async::{Read, Write};
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
    // USART1 => usart::InterruptHandler<peripherals::USART1>;
    USART1 => usart::BufferedInterruptHandler<peripherals::USART1>;
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

    let mut led = Output::new(p.PC6, Level::High, Speed::Medium);
    led.set_high();

    info!("Starting");
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

            match spawner.spawn(i2c_task(device, CHANNEL_I2C.receiver(), CHANNEL.sender())) {
                Ok(()) => {},
                Err(e) => error!("Error {}",e)
            }
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
            match spawner.spawn(spi_task(spi, CHANNEL_SPI.receiver(), CHANNEL.sender())) {
                Ok(()) => {},
                Err(e) => error!("Error {}",e)        
            }
        },
    }


    // -----------------------------------
    // Setup USART for RS485 / DMX
    // https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ApplicationNotes/ApplicationNotes/00001659A.pdf
    // -----------------------------------

    //A data byte is a Start bit, eight data bits and two Stop bits with LSB sent first
    let mut usart_config = UsartConfig::default();
    
    usart_config.baudrate = 250000;
    usart_config.data_bits = DataBits::DataBits8; // set to 9 data bits but we will ignore the start bit
    usart_config.stop_bits = StopBits::STOP2; //StopBits::STOP2;
    usart_config.parity = Parity::ParityNone;
    // usart_config.assume_noise_free = true;

    // CN10 pin 14 (D1) = p.PG14, CN10 pin 16 (D0) = p.PG9
    //let usart = Uart::new(p.USART1, p.PB7, p.PB6, Irqs, p.DMA1_CH3, p.DMA1_CH4, usart_config).unwrap();
    let mut tx_buf = [0u8; 10];
    let mut rx_buf = [0u8; 513]; 

    let mut usart = BufferedUart::new(p.USART1, p.PB7, p.PB6, &mut tx_buf, &mut rx_buf, Irqs, usart_config).unwrap();
    
    // Connect this pin to RX pin so we can detect DMX BREAK and MAB independent of the USART peripheral
    // let dmx_break_pin = ExtiInput::new(p.PD7, p.EXTI7, Pull::None);
    let mut dmx_break_pin = ExtiInput::new(p.PA15, p.EXTI15, Pull::None);
    // match spawner.spawn(dmx_task(usart, dmx_break_pin, CHANNEL.sender())) {
    //     Ok(()) => {},
    //     Err(e) => error!("Error {}",e)
    // }

    // -----------------------------------
    // Initialize event router
    // -----------------------------------
    info!("Initializing event router.");
    let router = Router::new(CHANNEL.receiver(), device_mode, CHANNEL_I2C.sender(), CHANNEL_SPI.sender());
    match spawner.spawn(event_router(router, led)) {
        Ok(()) => {},
        Err(e) => error!("Error {}",e)
    }

    let tx = CHANNEL.sender();
    

    const MAB_DELAY: u64 = 8;
    const BREAK_DELAY: u64 = 88;
    const BREAK_TIMEOUT: u64 = 1000000;

    let mut dmx_buffer: [u8; 513] = [0_u8; 513];

    info!("Starting DMX task");
    loop {
        dmx_break_pin.wait_for_falling_edge().await;
        let break_fall = Instant::now();
        dmx_break_pin.wait_for_rising_edge().await;
        let rise = Instant::now();

        let break_time = (rise - break_fall).as_micros();
        
        if (BREAK_DELAY..BREAK_TIMEOUT).contains(&break_time) { // if (break_time >= BREAK_DELAY) & (break_time < BREAK_TIMEOUT) {
            // info!("DMX BREAK detected");
        } else {
            // info!("DMX break timeout {}", break_time);
            continue
        }

        dmx_break_pin.wait_for_falling_edge().await;
        let mab_fall = Instant::now();


        let mab_time = (mab_fall - rise).as_micros();
        if (MAB_DELAY..BREAK_TIMEOUT).contains(&mab_time) {  // if (mab_time >= MAB_DELAY) & (mab_time < BREAK_TIMEOUT) {
            // info!("DMX MAB detected");
        } else {
            // info!("DMX MAB timeout {}", mab_time);
            continue
        }

        // match  usart.read(&mut dmx_buffer).await {
        match usart.read_exact(&mut dmx_buffer[..]).await {
            Ok(()) => {
                if dmx_buffer[0] == 0x00 {
                    info!("DMX sending packet to router");
                    if !tx.is_empty() {
                        info!("Clearing DMX channel");
                        tx.clear(); // clear any existing message on the channel
                    }
                    match tx.try_send(RouterEvent::DmxPacket(dmx_buffer)) {
                        Ok(()) => {},
                        Err(e) => error!("Error routing DMX data {}",e)
                    }
                } else {
                    info!("DMX packet start byte was not 0x00");
                }
            },
            Err(e) => {
                error!("DMX error reading data break: {}, mab: {}", break_time, mab_time);
                error!("Error {}", e);
            }
        }
        
    }
}