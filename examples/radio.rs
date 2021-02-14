#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate nb;
extern crate panic_semihosting;
extern crate stm32l1xx_hal as hal;

use core::fmt::Write;
use hal::prelude::*;
use hal::rcc;
use hal::serial;
use hal::serial::SerialExt;
use hal::stm32;
use nb::block;
use rt::entry;
use core::borrow::BorrowMut;


#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(rcc::Config::hsi());

    let _gpiob = dp.GPIOB.split();
    let gpioa = dp.GPIOA.split();
    let tx = gpioa.pa9;
    let rx = gpioa.pa10;

    let mut timer = dp.TIM2.timer(1.hz(), rcc.borrow_mut());

    let mut uart_cfg = serial::Config::default();
    uart_cfg.baudrate =115200_u32.bps();// time::Bps(115200_u32) ; //19200_u32.bsp();

    let serial = dp
        .USART1
        .usart((tx, rx), uart_cfg, &mut rcc)
        .unwrap();

    let (mut tx, mut _rx) = serial.split();

    loop {
        //let received = block!(rx.read()).unwrap();
        //tx.write_str("\r\n").unwrap();
        //block!(tx.write(received)).ok();
        writeln!(tx, "hello world\n").expect("failed to send");
        block!(timer.wait()).unwrap();
    }
}