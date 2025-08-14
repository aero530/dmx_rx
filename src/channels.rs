//! Communication channels between tasks
// use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_sync::channel::{Channel, Receiver, Sender};
// use embassy_sync::signal::Signal;
// use embassy_sync::watch::{Receiver, Sender, Watch};
// use embassy_sync::watch::{Receiver as WatchReceiver, Sender as WatchSender, Watch};

use crate::event_router::RouterEvent;
use crate::i2c_device::I2cEvent;
use crate::spi_device::SpiEvent;
// use crate::event_router::GlobalData;

pub type RouterChannel = Channel<CriticalSectionRawMutex, RouterEvent, 1>;
pub type RouterChannelRx = Receiver<'static, CriticalSectionRawMutex, RouterEvent, 1>;
pub type RouterChannelTx = Sender<'static, CriticalSectionRawMutex, RouterEvent, 1>;
pub static CHANNEL: RouterChannel = Channel::new();

pub type I2cChannel = Channel<CriticalSectionRawMutex, I2cEvent, 1>;
pub type I2cChannelRx = Receiver<'static, CriticalSectionRawMutex, I2cEvent, 1>;
pub type I2cChannelTx = Sender<'static, CriticalSectionRawMutex, I2cEvent, 1>;
pub static CHANNEL_I2C: I2cChannel = Channel::new();

pub type SpiChannel = Channel<CriticalSectionRawMutex, SpiEvent, 1>;
pub type SpiChannelRx = Receiver<'static, CriticalSectionRawMutex, SpiEvent, 1>;
pub type SpiChannelTx = Sender<'static, CriticalSectionRawMutex, SpiEvent, 1>;
pub static CHANNEL_SPI: SpiChannel = Channel::new();
