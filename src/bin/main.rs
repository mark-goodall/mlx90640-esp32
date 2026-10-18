#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::fmt::{self, Write};

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::main;
use esp_hal::time::{Instant, Rate};
use esp_hal::Blocking;
use esp_hal::uart::{Config as UartConfig, Uart};
use mlx90640_esp32::mlx90640::{DEFAULT_ADDRESS, Mlx90640, RefreshRate};
use mlx90640_esp32::protocol::{self, FRAME_PAYLOAD_LEN, FrameInfo, TYPE_FRAME, TYPE_TEXT};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

/// Serial baud rate for binary output (UART0, the USB-serial bridge).
const BAUD: u32 = 921_600;
/// Sub-page rate; complete frames are output at half this rate.
const REFRESH: RefreshRate = RefreshRate::Hz4;
const EMISSIVITY: f32 = 0.95;

fn write_all(serial: &mut Uart<'_, Blocking>, mut data: &[u8]) {
    while !data.is_empty() {
        match serial.write(data) {
            Ok(n) => data = &data[n..],
            Err(_) => return,
        }
    }
}

fn send_packet(serial: &mut Uart<'_, Blocking>, kind: u8, payload: &[u8]) {
    let (header, crc) = protocol::wrap(kind, payload);
    write_all(serial, &header);
    write_all(serial, payload);
    write_all(serial, &crc);
}

/// Fixed-size buffer for formatting text messages; overflow is truncated.
struct TextBuf {
    buf: [u8; 128],
    len: usize,
}

impl Write for TextBuf {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let n = s.len().min(self.buf.len() - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

fn send_text(serial: &mut Uart<'_, Blocking>, args: fmt::Arguments) {
    let mut text = TextBuf { buf: [0; 128], len: 0 };
    text.write_fmt(args).ok();
    send_packet(serial, TYPE_TEXT, &text.buf[..text.len]);
}

#[allow(
    clippy::large_stack_frames,
    reason = "the sensor driver holds ~17 KB of calibration data"
)]
#[main]
fn main() -> ! {
    // generator version: 1.4.0
    // generator parameters: -o esp32 -o esp-backtrace

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let delay = Delay::new();

    let mut serial = Uart::new(peripherals.UART0, UartConfig::default().with_baudrate(BAUD))
        .unwrap()
        .with_tx(peripherals.GPIO1)
        .with_rx(peripherals.GPIO3);

    // Standard ESP32 dev board I2C pins: SDA = GPIO21, SCL = GPIO22.
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO21)
    .with_scl(peripherals.GPIO22);

    let mut sensor = Mlx90640::new(i2c, delay, DEFAULT_ADDRESS);
    while let Err(e) = sensor
        .load_calibration()
        .and_then(|()| sensor.configure(REFRESH))
    {
        send_text(&mut serial, format_args!("MLX90640 init failed: {e:?}, retrying"));
        delay.delay_millis(1000);
    }
    send_text(&mut serial, format_args!("MLX90640 ready"));

    let mut payload = [0u8; FRAME_PAYLOAD_LEN];
    let mut frame = 0u32;
    let mut subpages = 0u32;
    // A frame is complete once both chess sub-pages have been refreshed.
    let mut have_subpage0 = false;
    loop {
        match sensor.read_subpage(EMISSIVITY) {
            Ok(subpage) => {
                subpages += 1;
                if subpage == 0 {
                    have_subpage0 = true;
                } else if have_subpage0 {
                    have_subpage0 = false;
                    let info = FrameInfo {
                        frame,
                        time_ms: Instant::now().duration_since_epoch().as_millis() as u32,
                        subpages,
                        retries: sensor.retries,
                        ambient: sensor.ambient,
                    };
                    protocol::encode_frame(&info, &sensor.temperatures, &mut payload);
                    send_packet(&mut serial, TYPE_FRAME, &payload);
                    frame = frame.wrapping_add(1);
                }
            }
            Err(e) => {
                send_text(&mut serial, format_args!("read failed: {e:?}"));
                have_subpage0 = false;
                delay.delay_millis(100);
            }
        }
    }
}
