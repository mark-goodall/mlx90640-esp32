# mlx90640-esp32

Bare-metal Rust (esp-hal) firmware for an ESP32 that reads an MLX90640 32x24
thermal sensor over I2C and streams calibrated temperatures (°C) over the USB
serial port as CSV.

## Wiring

| MLX90640 | ESP32  |
|----------|--------|
| SDA      | GPIO21 |
| SCL      | GPIO22 |
| VIN      | 3V3    |
| GND      | GND    |

Change the pins in `src/bin/main.rs` if your board differs.

## Output format

Serial runs at **921600 baud** and carries binary packets (format documented
in `src/protocol.rs`): a magic marker, type, length, payload and CRC-16. Frame
packets hold a 24x32 array of temperatures as little-endian `i16` centi-°C
(1.5 KB per frame instead of ~4.6 KB of text), plus a frame counter, timestamp,
sub-page and I2C-retry counters, and the sensor's ambient temperature.

`tools/mlx_reader.py` decodes the stream back to CSV: 24
lines of 32 comma-separated values (°C), with a blank line between frames.
Device status messages go to stderr. The script declares its dependencies
(PEP 723), so running it directly uses `uv` to fetch `pyserial` automatically.

```sh
tools/mlx_reader.py /dev/ttyUSB0                 # CSV to stdout, forever
tools/mlx_reader.py -n 100 -o frames.csv --stats # 100 frames to a file, stats on stderr
```

## Build / flash

```sh
. ~/export-esp.sh
cargo run --release          # flash + monitor (espflash)
```

`cargo run` opens espflash's monitor, which will show binary noise; use
`tools/mlx_reader.py` to read the data.

## Notes

- Calibration math is a port of Melexis' `MLX90640_API.c`. It matches the
  Melexis worked example (datasheet EEPROM + two frames) to within 0.0005 °C
  on all 768 pixels.
- The sensor briefly stops responding on I2C around the end of each sub-page;
  the driver retries failed transactions (up to 50 x 1 ms).
- Emissivity defaults to 0.95; reflected temperature is taken as Ta - 8 °C.
