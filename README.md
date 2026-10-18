# mlx90640-esp32

Bare-metal Rust (esp-hal) firmware for an ESP32-C3 that reads an MLX90640 32x24
thermal sensor over I2C and streams calibrated temperatures (°C) over the chip's
built-in USB-Serial-JTAG port (`/dev/ttyACM0`).

The original ESP32 (Xtensa, UART0 via CH340) version is the first commit in
this repository's history.

## Wiring

| MLX90640 | ESP32-C3 |
|----------|----------|
| SDA      | GPIO8    |
| SCL      | GPIO9    |
| VIN      | 3V3    |
| GND      | GND    |

Change the pins in `src/bin/main.rs` if your board differs. The `i2c_scan`
binary (`cargo run --release --bin i2c_scan`) finds which GPIO pair the bus is on.

## Output format

The USB-Serial-JTAG port (baud rate is irrelevant) carries binary packets (format documented
in `src/protocol.rs`): a magic marker, type, length, payload and CRC-16. Frame
packets hold a 24x32 array of temperatures as little-endian `i16` centi-°C
(1.5 KB per frame instead of ~4.6 KB of text), plus a frame counter, timestamp,
sub-page and I2C-retry counters, and the sensor's ambient temperature.

`tools/mlx_reader.py` decodes the stream back to CSV: 24
lines of 32 comma-separated values (°C), with a blank line between frames.
Device status messages go to stderr. The script declares its dependencies
(PEP 723), so running it directly uses `uv` to fetch `pyserial` automatically.

```sh
tools/mlx_reader.py /dev/ttyACM0                 # CSV to stdout, forever
tools/mlx_reader.py -n 100 -o frames.csv --stats # 100 frames to a file, stats on stderr
```

## Live view

`tools/mlx_view.py` shows the stream as a live matplotlib heatmap (auto-ranged
colours, min/max markers with temperatures, hover for a pixel's temperature). Like the reader it
uses `uv` to fetch its dependencies (pyserial, numpy, matplotlib).

```sh
tools/mlx_view.py                          # /dev/ttyACM0, inferno colormap
tools/mlx_view.py --vmin 15 --vmax 40      # fixed colour range
tools/mlx_view.py --flip-h --rotate 180    # orient to match the camera mounting
tools/mlx_view.py --snapshot frame.png     # headless: save a PNG and exit
```

The image is mirrored left/right by default (like looking in a mirror when the
camera faces you); `--flip-h` shows the sensor's native orientation instead.

It needs a GUI display (run it from the desktop session, or with X forwarding).

## Build / flash

```sh
cargo run --release          # flash + monitor (espflash)
```

`cargo run` opens espflash's monitor, which will show binary noise; use
`tools/mlx_reader.py` to read the data.

## Notes

- Calibration math is a port of Melexis' `MLX90640_API.c`. It matches the
  Melexis worked example (datasheet EEPROM + two frames) to within 0.0005 °C
  on all 768 pixels.
- The driver retries failed I2C transactions (up to 50 x 1 ms). On the original
  ESP32 board the sensor faulted around the end of each sub-page; on the C3
  setup retries are 0.
- Runs at the 16 Hz sub-page rate (8 full frames/s) with I2C at 1 MHz. Per
  sub-page: ~18.5 ms I2C read + ~15 ms temperature math (soft-float, the C3
  has no FPU) + ~3 ms USB send per frame, so 16 fps (Hz32, 31 ms budget) would
  need faster math. Pixel noise grows with rate: ~0.25 °C rms at 2 fps vs
  ~0.55 °C at 8 fps. Lower `REFRESH` in `src/bin/main.rs` for less noise.
  `cargo run --release --bin bench` re-measures the stage timings.
- If no program is reading the port, output is dropped after 100 ms so the
  sensor loop keeps running.
- Builds on the stable toolchain (target `riscv32imc-unknown-none-elf`); no
  `espup`/Xtensa toolchain needed.
- Emissivity defaults to 0.95; reflected temperature is taken as Ta - 8 °C.
