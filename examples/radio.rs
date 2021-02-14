#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate nb;
extern crate panic_semihosting;
extern crate stm32l1xx_hal as hal;
extern crate radio_sx127x as sx;
extern crate embedded_spi;
extern crate embedded_hal;
extern crate radio;

use core::convert::{Infallible};
use core::fmt::{Write};
use hal::prelude::*;
use hal::rcc;
use hal::serial;
use hal::serial::SerialExt;
use hal::stm32;
use nb::block;
use rt::entry;
use embedded_hal::digital::v2::OutputPin;
//use sx::Sx127x;
//use hal::delay::Delay;
//use hal::spi::{Mode, Phase, Polarity};
use radio::Transmit;

use embedded_spi::{wrapper::Wrapper as SpiWrapper};

//use hal::{          spi::{Spi, Mode, Phase, Polarity},};
use sx::Sx127x;
use sx::device::{Modem, Channel, PaConfig, PaSelect};
use sx::device::lora::{LoRaChannel, Bandwidth, SpreadingFactor, CodingRate, LoRaConfig, PayloadLength, PayloadCrc, FrequencyHopping};
use core::borrow::{BorrowMut, Borrow};


const FREQUENCY: u32 = 907_400_000;
// frequency in hertz ch_12: 915_000_000, ch_2: 907_400_000
const CONFIG_CH: LoRaChannel = LoRaChannel {
    freq: FREQUENCY as u32,          // frequency in hertz
    bw: Bandwidth::Bw125kHz,
    sf: SpreadingFactor::Sf7,
    cr: CodingRate::Cr4_8,
};

const CONFIG_LORA: LoRaConfig = LoRaConfig {
    preamble_len: 0x8,
    symbol_timeout: 0x64,
    payload_len: PayloadLength::Variable,
    payload_crc: PayloadCrc::Enabled,
    frequency_hop: FrequencyHopping::Disabled,
    invert_iq: false,
};

const CONFIG_PA: PaConfig = PaConfig {
    output: PaSelect::Boost,
    power: 10,
};

const CONFIG_RADIO: radio_sx127x::device::Config = radio_sx127x::device::Config {
    modem: Modem::LoRa(CONFIG_LORA),
    channel: Channel::LoRa(CONFIG_CH),
    pa_config: CONFIG_PA,
    xtal_freq: 32000000,                  // CHECK
    timeout_ms: 100,
};


#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(rcc::Config::hsi());

    let mut delay = cp.SYST.delay(rcc.clocks);

    let gpiob = dp.GPIOB.split();
    let gpioa = dp.GPIOA.split();
    let tx = gpioa.pa9;
    let rx = gpioa.pa10;

    let mut timer = dp.TIM2.timer(1.hz(), rcc.borrow_mut());

    let mut uart_cfg = serial::Config::default();
    uart_cfg.baudrate = 115200_u32.bps();// time::Bps(115200_u32) ; //19200_u32.bsp();


    let serial = dp
        .USART1
        .usart((tx, rx), uart_cfg, &mut rcc)
        .unwrap();

    let (mut tx, mut _rx) = serial.split();

    let sck = gpiob.pb3;
    let miso = gpiob.pb4;
    let mosi = gpiob.pb5;

    let mut spi = dp
        .SPI3
        .spi((sck, miso, mosi), hal::spi::MODE_0, 100.khz(), &mut rcc);

    /*
    let spi1 = Spi::spi1(
        cp.SPI1,
        (gpioa.pa5.into_alternate_af5(),  // sck   on PA5
         gpioa.pa6.into_alternate_af5(),  // miso  on PA6
         gpioa.pa7.into_alternate_af5()   // mosi  on PA7
        ),
        Mode {              //  SPI mode for radio
            phase: Phase::CaptureOnSecondTransition,
            polarity: Polarity::IdleHigh,
        },
        100.khz().into(),
        rcc.borrow_mut(),
    );
*/

    let cp = cortex_m::Peripherals::take().unwrap();
    let p = cortex_m::Peripherals::take().unwrap();


    //let cs_pin = gpiob.pb0.into_push_pull_output().borrow() as &dyn OutputPin<Error = Infallible>; // B0
    let cs_pin = gpiob.pb0.into_push_pull_output(); // B0
    let busy_pin = gpiob.pb8.into_floating_input();
    let sdn_pin = gpioa.pa2.into_push_pull_output().borrow() as &dyn OutputPin<Error = Infallible>; // SX1272_RST_PA2_DEFAULT
    let xxx : dyn embedded_hal::digital::v2::OutputPin<Error = Infallible> = cs_pin;
    // Create SpiWrapper over spi/cs/busy
    let mut hal = SpiWrapper::new(spi, cs_pin, delay);
    hal.with_busy(busy_pin.into());
    hal.with_reset(sdn_pin); //
    // Create instance with new hal
    let mut lora = Sx127x::new(hal, &CONFIG_RADIO).unwrap();

    // let mut _radio1 = Sx127x::new(lora, &sx::prelude::Config::default()).expect("error creating radio1");

    let message = b"Hello, LoRa!";
    loop {
        writeln!(tx, "hello world\n").unwrap();

        lora.start_transmit(message).unwrap();    // should handle error

        match lora.check_transmit() {
            Ok(b) => if b { writeln!(tx, "TX complete").unwrap() } else { writeln!(tx,"TX not complete").unwrap() },

            Err(_err) => writeln!(tx,"Error in lora.check_transmit(). Should return True or False.").unwrap(),
        };


        block!(timer.wait()).unwrap();
        delay.delay(300.ms());
    }
}

/*
hal_extIRQ_disableGpioEXTI(1); 		// DIO0: PB1   (line 1 irq handler), RxDone
hal_extIRQ_disableGpioEXTI(10);		// DIO1: PB10  (line 10-15 irq handler), RxTimeout
hal_extIRQ_disableGpioEXTI(11);		// DIO2: PB11  (line 10-15 irq handler), FhssChangeChannel
hal_extIRQ_disableGpioEXTI(7); 		// DIO3: PB7   (line 5-9 irq handler), CadDone
hal_extIRQ_disableGpioEXTI(5);		// DIO4: PB5   (line 5-9 irq handler), CadDetected
hal_extIRQ_disableGpioEXTI(4); 		// DIO5: PB4   (line 4 irq handler), not used yet by LMIC/or Lobaro radio driver
 */

//let received = block!(rx.read()).unwrap();
//tx.write_str("\r\n").unwrap();
//block!(tx.write(received)).ok();