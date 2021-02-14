#![no_std]
#![no_main]

extern crate stm32l1xx_hal;
extern crate cortex_m_rt as rt;

use rt::{entry};
use stm32l1xx_hal::stm32::Peripherals;
use core::panic::PanicInfo;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut usart = peripherals.USART1;


    loop {}
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

