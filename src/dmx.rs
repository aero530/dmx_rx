//! DMX interaction
use defmt::{error, info};
use embassy_stm32::exti::ExtiInput;
use embassy_time::Instant;

use embassy_stm32::usart::{BufferedUart, Uart};
use embedded_io_async::{Read, Write};

use crate::channels::RouterChannelTx;
use crate::event_router::RouterEvent;

/// Monitor dmx_break_pin interrupt
#[embassy_executor::task]
//pub async fn dmx_task(mut usart: Uart<'static, embassy_stm32::mode::Async>, mut dmx_break_pin: ExtiInput<'static>, tx: RouterChannelTx) {
// pub async fn dmx_task(mut usart: Uart<'static, embassy_stm32::mode::Blocking>, mut dmx_break_pin: ExtiInput<'static>, tx: RouterChannelTx) {
pub async fn dmx_task(mut usart: BufferedUart<'static>, mut dmx_break_pin: ExtiInput<'static>, tx: RouterChannelTx) {
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
