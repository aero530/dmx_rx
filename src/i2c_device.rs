use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32::i2c::{Address, SlaveCommandKind};
use embassy_stm32::mode::Async;
use embassy_stm32::i2c;

// DEV_ADDR must be defined in the main application
// const DEV_ADDR: u8 = 0x42;
use crate::{channels::{I2cChannelRx, RouterChannelTx}, DEV_ADDR};

#[derive(Clone, Format)]
pub enum I2cEvent {
    Write([u8; 513]),
}

// https://github.com/embassy-rs/embassy/blob/7672229ffe00189fd2258020f933fab6146889cb/examples/stm32g4/src/bin/i2c_slave.rs#L37

#[embassy_executor::task]
pub async fn i2c_task(mut dev: i2c::I2c<'static, Async, i2c::MultiMaster>, rx: I2cChannelRx, _tx: RouterChannelTx) -> ! {
    info!("I2C device start");

    let mut dmx_data = [0u8; 513];

    loop {
        let mut incoming_command = [0u8; 4];

        match dev.listen().await {

            Ok(i2c::SlaveCommand {
                kind: SlaveCommandKind::Read,
                address: Address::SevenBit(DEV_ADDR),
            }) => {
                get_dmx_update(rx, &mut dmx_data);

                match dev.respond_to_read(&dmx_data).await {
                    Ok(i2c::SendStatus::LeftoverBytes(x)) => info!("tried to write {} extra bytes", x),
                    Ok(i2c::SendStatus::Done) => {}
                    Err(e) => error!("error while responding {}", e),
                }
            },

            Ok(i2c::SlaveCommand {
                kind: SlaveCommandKind::Write,
                address: Address::SevenBit(DEV_ADDR),
            }) => match dev.respond_to_write(&mut incoming_command).await {
                Ok(rx_len) => {
                    info!("Device received write: {}", incoming_command[..rx_len]);
                    
                    match incoming_command[0] {
                        0xDA => {
                            get_dmx_update(rx, &mut dmx_data);

                            match dev.respond_to_read(&dmx_data).await {
                                Ok(read_status) => info!("This read is part of a write/read transaction. The response read status {}", read_status),
                                Err(i2c::Error::Timeout) => info!("The device only performed a write and it not also do a read"),
                                Err(e) => error!("error while responding {}", e),
                            }
                        },
                        x => {
                            error!("Invalid Write Read {:x}", x);
                        }
                    }
                }
                Err(e) => error!("error while receiving {}", e),
            },

            Ok(i2c::SlaveCommand { address, .. }) => {
                defmt::unreachable!(
                    "The slave matched address: {}, which it was not configured for",
                    address
                );
            }
            Err(e) => error!("{}", e),
        }
    }
}

// Check to see if the RX channel has a message (new DMX data) from the router.
// If there is new data then save that data to local dmx_data
fn get_dmx_update(rx: I2cChannelRx, dmx_data: &mut [u8;513] ) {
    info!("I2C pull dmx data");
    if let Ok(new_message) = rx.try_receive() {
        match new_message {
            // update our version of DMX data from the router
            I2cEvent::Write(data) => {
                *dmx_data = data
            },
        }
    }
}
