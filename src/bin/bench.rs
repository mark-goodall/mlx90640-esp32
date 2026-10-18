//! Diagnostic: times each stage of a sub-page read (I2C transfer at several bus
//! speeds, temperature calculation, packet send) and reports via text packets.
#![no_std]
#![no_main]
#![allow(clippy::large_stack_frames)]

use core::fmt::Write;

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::main;
use esp_hal::time::{Instant, Rate};
use esp_hal::usb::usb_serial_jtag::UsbSerialJtag;
use mlx90640_esp32::mlx90640::{DEFAULT_ADDRESS, Mlx90640, RefreshRate};
use mlx90640_esp32::protocol::{self, FRAME_PAYLOAD_LEN, TYPE_FRAME, TYPE_TEXT};

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

fn send(serial: &mut UsbSerialJtag<'_, esp_hal::Blocking>, kind: u8, payload: &[u8]) {
    let (h, c) = protocol::wrap(kind, payload);
    write_all(serial, &h);
    write_all(serial, payload);
    write_all(serial, &c);
}

macro_rules! report {
    ($serial:expr, $($arg:tt)*) => {{
        let mut t = Text { buf: [0; 200], len: 0 };
        write!(t, $($arg)*).ok();
        send($serial, TYPE_TEXT, &t.buf[..t.len]);
    }};
}

#[main]
fn main() -> ! {
    let mut p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let delay = Delay::new();
    let mut serial = UsbSerialJtag::new(p.USB_DEVICE);
    delay.delay_millis(2000);

    // Raw RAM read (832 words) at several bus speeds and chunk sizes.
    let mut bytes = [0u8; 1664];
    for khz in [400u32, 800, 1000] {
        let cfg = I2cConfig::default().with_frequency(Rate::from_khz(khz));
        let mut i2c = I2c::new(p.I2C0.reborrow(), cfg)
            .unwrap()
            .with_sda(p.GPIO8.reborrow())
            .with_scl(p.GPIO9.reborrow());
        for chunk_words in [14usize, 64, 832] {
            let start = Instant::now();
            let mut errors = 0;
            for (i, chunk) in bytes.chunks_mut(chunk_words * 2).enumerate() {
                let addr = 0x0400u16 + (i * chunk_words) as u16;
                if i2c.write_read(DEFAULT_ADDRESS, &addr.to_be_bytes(), chunk).is_err() {
                    errors += 1;
                }
            }
            let us = start.elapsed().as_micros();
            report!(&mut serial, "I2C {khz} kHz, {chunk_words}-word chunks: RAM read {us} us, {errors} errors");
        }
    }

    let i2c = I2c::new(p.I2C0, I2cConfig::default().with_frequency(Rate::from_khz(400)))
        .unwrap()
        .with_sda(p.GPIO8)
        .with_scl(p.GPIO9);
    let mut sensor = Mlx90640::new(i2c, delay, DEFAULT_ADDRESS);
    sensor.load_calibration().unwrap();

    // Temperature calculation for one sub-page from the last-read RAM.
    sensor.configure(RefreshRate::Hz64).unwrap();
    sensor.read_subpage(0.95).unwrap();
    let start = Instant::now();
    for i in 0..10 {
        sensor.calculate_to(0x1901, (i % 2) as u8, 0.95);
    }
    report!(&mut serial, "calculate_to: {} us per sub-page", start.elapsed().as_micros() / 10);

    let payload = [0u8; FRAME_PAYLOAD_LEN];
    let start = Instant::now();
    send(&mut serial, TYPE_FRAME, &payload);
    report!(&mut serial, "frame packet send: {} us", start.elapsed().as_micros());

    // Whole read_subpage loop at the fastest refresh rate.
    let start = Instant::now();
    let n = 32;
    for _ in 0..n {
        sensor.read_subpage(0.95).unwrap();
    }
    let us = start.elapsed().as_micros() / n;
    report!(&mut serial, "read_subpage at Hz64: {us} us per sub-page, retries {}", sensor.retries);

    loop {
        delay.delay_millis(1000);
    }
}
