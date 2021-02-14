# rust-iM880B-L
Rust based board / driver for the iM880B-L LoRaWAN Module from IMST


## Setup

    rustup target add thumbv7m-none-eabi

    rustup default nightly-x86_64-pc-windows-gnu // TODO: Build does not finish with GNU Toolchain
    rustup default nightly-x86_64-pc-windows-msvc
    rustup target add thumbv7m-none-eabi

### Radio example

Build

    cargo build --example radio

Create hex file to be flashed:

    cargo objcopy --example radio -- -O ihex radio.hex