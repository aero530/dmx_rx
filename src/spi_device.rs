use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32::spi::Spi;
use embassy_stm32::mode::Async;

use crate::{channels::{SpiChannelRx, RouterChannelTx}};

#[derive(Clone, Format)]
pub enum SpiEvent {
    Write([u8; 513]),
}

#[embassy_executor::task]
pub async fn spi_task(mut spi: Spi<'static, Async>, rx: SpiChannelRx, _tx: RouterChannelTx) -> ! {
    info!("SPI device start");

    let mut dmx_data = [0u8; 513];

    let _ = spi.write(&dmx_data).await;

    loop {
        let mut incoming_command = [0_u8; 2];

        match spi.read(&mut incoming_command).await {
            Ok(_) => {
                info!("SPI command received: {}", incoming_command);
                match incoming_command[0] {
                    0xDA => {
                        // Check to see if the RX channel has a message (new DMX data) from the router.
                        // If there is new data then save that data to local dmx_data
                        if let Ok(new_message) = rx.try_receive() {
                            match new_message {
                                SpiEvent::Write(data) => {
                                    dmx_data = data
                                },
                            }
                        }
                        let _ = spi.write(&dmx_data).await;
                    },
                    x => {
                        error!("Invalid SPI command {:x}", x);
                    }
                }
            },
            Err(e) => error!("Error SPI: {}", e),
        }
    }
}
