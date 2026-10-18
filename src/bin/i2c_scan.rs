//! Diagnostic: finds which GPIO pair the I2C bus is wired to and reports the
//! responding addresses as text packets readable by `tools/mlx_reader.py`.
//!
//! Pins with external pull-ups (as on sensor breakout boards) read high even
//! with the internal pull-down enabled; every pair of those is then scanned in
//! both orders.
#![no_std]
#![no_main]

use core::fmt::Write;

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{AnyPin, Input, InputConfig, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::main;
use esp_hal::time::{Instant, Rate};
use esp_hal::usb::usb_serial_jtag::UsbSerialJtag;
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

/// Writes bytes, giving up if the host stops reading for 100 ms.
fn write_all(serial: &mut UsbSerialJtag<'_, esp_hal::Blocking>, data: &[u8]) {
    for &b in data {
        let start = Instant::now();
        while serial.write_byte_nb(b).is_err() {
            if start.elapsed().as_millis() > 100 {
                return;
            }
        }
    }
    serial.flush_tx_nb().ok();
}

macro_rules! report {
    ($serial:expr, $($arg:tt)*) => {{
        let mut t = Text { buf: [0; 200], len: 0 };
        write!(t, $($arg)*).ok();
        let (h, c) = protocol::wrap(TYPE_TEXT, &t.buf[..t.len]);
        write_all($serial, &h);
        write_all($serial, &t.buf[..t.len]);
        write_all($serial, &c);
    }};
}

#[main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let delay = Delay::new();
    let mut serial = UsbSerialJtag::new(p.USB_DEVICE);

    // All free GPIOs on the ESP32-C3 (11-17 are flash, 18/19 are USB).
    let mut pins: [(u8, AnyPin<'static>); 13] = [
        (0, p.GPIO0.into()),
        (1, p.GPIO1.into()),
        (2, p.GPIO2.into()),
        (3, p.GPIO3.into()),
        (4, p.GPIO4.into()),
        (5, p.GPIO5.into()),
        (6, p.GPIO6.into()),
        (7, p.GPIO7.into()),
        (8, p.GPIO8.into()),
        (9, p.GPIO9.into()),
        (10, p.GPIO10.into()),
        (20, p.GPIO20.into()),
        (21, p.GPIO21.into()),
    ];
    let mut i2c0 = p.I2C0;

    loop {
        let mut pulled = [0usize; 13];
        let mut n_pulled = 0;
        let mut levels = Text { buf: [0; 200], len: 0 };
        for (i, (num, pin)) in pins.iter_mut().enumerate() {
            let cfg = InputConfig::default().with_pull(Pull::Down);
            let input = Input::new(pin.reborrow(), cfg);
            delay.delay_micros(50);
            let high = input.is_high();
            write!(levels, "{num}={} ", high as u8).ok();
            if high {
                pulled[n_pulled] = i;
                n_pulled += 1;
            }
        }
        let levels = core::str::from_utf8(&levels.buf[..levels.len]).unwrap_or("");
        report!(&mut serial, "levels with pull-down: {levels}");

        let mut any = false;
        for &a in &pulled[..n_pulled] {
            for &b in &pulled[..n_pulled] {
                if a == b {
                    continue;
                }
                let [(sda_num, sda), (scl_num, scl)] = pins.get_disjoint_mut([a, b]).unwrap();
                let cfg = I2cConfig::default().with_frequency(Rate::from_khz(100));
                let mut i2c = I2c::new(i2c0.reborrow(), cfg)
                    .unwrap()
                    .with_sda(sda.reborrow())
                    .with_scl(scl.reborrow());
                let mut found = [0u8; 16];
                let mut n = 0;
                for addr in 0x08u8..0x78 {
                    let mut buf = [0u8; 1];
                    if i2c.read(addr, &mut buf).is_ok() && n < found.len() {
                        found[n] = addr;
                        n += 1;
                    }
                }
                if n > 0 {
                    any = true;
                    report!(&mut serial, "SDA={sda_num} SCL={scl_num}: {n} device(s) {:02x?}", &found[..n]);
                }
            }
        }
        if !any {
            report!(&mut serial, "no I2C devices found on {n_pulled} pulled-up pins");
        }
        delay.delay_millis(2000);
    }
}
