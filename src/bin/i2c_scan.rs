//! Diagnostic: scans the I2C bus on GPIO21/22 (both pin orders) and reports
//! responding addresses as text packets readable by `tools/mlx_reader.py`.
#![no_std]
#![no_main]

use core::fmt::Write;

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::main;
use esp_hal::time::Rate;
use esp_hal::uart::{Config as UartConfig, Uart};
use mlx90640_esp32::protocol::{self, TYPE_TEXT};

esp_bootloader_esp_idf::esp_app_desc!();

struct Text {
    buf: [u8; 200],
    len: usize,
}

impl Write for Text {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let n = s.len().min(self.buf.len() - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

macro_rules! report {
    ($serial:expr, $($arg:tt)*) => {{
        let mut t = Text { buf: [0; 200], len: 0 };
        write!(t, $($arg)*).ok();
        let (h, c) = protocol::wrap(TYPE_TEXT, &t.buf[..t.len]);
        for part in [&h[..], &t.buf[..t.len], &c[..]] {
            let mut d = part;
            while !d.is_empty() {
                match $serial.write(d) { Ok(n) => d = &d[n..], Err(_) => break }
            }
        }
    }};
}

#[main]
fn main() -> ! {
    let mut p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let delay = Delay::new();
    let mut serial = Uart::new(p.UART0, UartConfig::default().with_baudrate(921_600))
        .unwrap()
        .with_tx(p.GPIO1)
        .with_rx(p.GPIO3);

    loop {
        // Idle line levels with only the internal pull-ups: both should read high.
        {
            let cfg = InputConfig::default().with_pull(Pull::None);
            let a = Input::new(p.GPIO21.reborrow(), cfg).is_high();
            let b = Input::new(p.GPIO22.reborrow(), cfg).is_high();
            report!(serial, "idle levels (no internal pull): GPIO21={} GPIO22={}", a as u8, b as u8);
        }
        for (name, swapped) in [("SDA=21 SCL=22", false), ("SDA=22 SCL=21", true)] {
            let cfg = I2cConfig::default().with_frequency(Rate::from_khz(100));
            let mut i2c = I2c::new(p.I2C0.reborrow(), cfg).unwrap();
            i2c = if swapped {
                i2c.with_sda(p.GPIO22.reborrow()).with_scl(p.GPIO21.reborrow())
            } else {
                i2c.with_sda(p.GPIO21.reborrow()).with_scl(p.GPIO22.reborrow())
            };
            let mut found = [0u8; 16];
            let mut n = 0;
            for addr in 0x08u8..0x78 {
                let mut b = [0u8; 1];
                if i2c.read(addr, &mut b).is_ok() && n < found.len() {
                    found[n] = addr;
                    n += 1;
                }
            }
            report!(serial, "{name}: {n} device(s) {:02x?}", &found[..n]);
        }
        delay.delay_millis(1000);
    }
}
