//! Event router to send commands between tasks
use defmt::{error, info, Format};
use embassy_stm32::gpio::Output;

use crate::{channels::*, i2c_device::I2cEvent, spi_device::SpiEvent, DeviceMode};
// use embassy_time::{with_timeout, Duration};

// /// Data stored for global use (primarily for logging / terminal display)
// #[derive(Debug, PartialEq, Copy, Clone)]
// pub struct GlobalData {
//     pub dmx_address: u16,
//     pub dmx: [u8; 513],
// }

// impl Default for GlobalData {
//     fn default() -> Self { 
//         Self {
//             dmx_address: 0,
//             dmx: [0; 513]
//         }
//     }
// }

/// Events the router watches for.  These trigger the router to pass along an event to another object.
#[derive(Clone, Format)]
pub enum RouterEvent {
    DmxPacket([u8;513]),
}

pub struct Router {
    /// Listen for event router tasks
    pub channel: RouterChannelRx,
    pub device_mode: DeviceMode,
    pub channel_i2c: I2cChannelTx,
    pub channel_spi: SpiChannelTx,
    // Global data store
    // pub data: GlobalData,
}

impl Router {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        channel: RouterChannelRx,
        device_mode: DeviceMode,
        channel_i2c: I2cChannelTx,
        channel_spi: SpiChannelTx,
    ) -> Self {
        Self {
            channel,
            device_mode,
            channel_i2c,
            channel_spi,
            // data: GlobalData::default(),
        }
    }

    pub async fn process_event(&mut self, event: RouterEvent) {
        match event {
            RouterEvent::DmxPacket(dmx_data) => {
                info!("Router got DMX data");
                // self.data.dmx = input;

                match self.device_mode {
                    DeviceMode::I2c => {
                        if self.channel_i2c.is_full() {
                            self.channel_i2c.clear(); // clear any existing message on the channel
                        }
                        match self.channel_i2c.try_send(I2cEvent::Write(dmx_data)) {
                            Ok(()) => {},
                            Err(e) => error!("Error routing DMX data to I2C {}",e)
                        }
                    },
                    DeviceMode::Spi => {
                        if self.channel_spi.is_full() {
                            self.channel_spi.clear(); // clear any existing message on the channel
                        }
                        match self.channel_spi.try_send(SpiEvent::Write(dmx_data)) {
                            Ok(()) => {},
                            Err(e) => error!("Error routing DMX data to SPI {}",e)
                        }
                    },
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn event_router(mut router: Router, mut led: Output<'static>) {
    loop {
        let new_message = router.channel.receive().await; // the only router event is dmx data
        led.set_high();
        router.process_event(new_message).await;
        led.set_low();
    }
}
