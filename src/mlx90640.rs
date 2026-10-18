//! Minimal MLX90640 driver: I2C access, EEPROM calibration extraction and
//! per-pixel object temperature calculation.
//!
//! The math is a port of the Melexis reference implementation
//! (`MLX90640_API.c`, see datasheet section 11), kept in `f32` throughout.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use libm::{powf, sqrtf};

pub const DEFAULT_ADDRESS: u8 = 0x33;
pub const WIDTH: usize = 32;
pub const HEIGHT: usize = 24;
pub const PIXELS: usize = WIDTH * HEIGHT;

const REG_STATUS: u16 = 0x8000;
const REG_CONTROL1: u16 = 0x800D;
const RAM_START: u16 = 0x0400;
const EEPROM_START: u16 = 0x2400;
const EEPROM_WORDS: usize = 832;
/// 768 pixel words followed by 64 auxiliary words.
const RAM_WORDS: usize = 832;
/// Words read per I2C transaction.
const CHUNK_WORDS: usize = 64;
/// Attempts per I2C transaction before giving up. The sensor briefly stops
/// responding while it updates RAM at the end of each sub-page, so failed
/// transactions are retried after `RETRY_DELAY_US`.
const MAX_ATTEMPTS: u32 = 50;
const RETRY_DELAY_US: u32 = 1000;

/// Sub-page refresh rate (control register 1, bits 7..9).
#[derive(Clone, Copy, Debug)]
#[allow(dead_code)]
pub enum RefreshRate {
    Hz0_5 = 0,
    Hz1 = 1,
    Hz2 = 2,
    Hz4 = 3,
    Hz8 = 4,
    Hz16 = 5,
    Hz32 = 6,
    Hz64 = 7,
}

#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
    /// EEPROM contents failed basic sanity checks (wiring / wrong device).
    BadEeprom,
    /// Data ready never asserted.
    Timeout,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::I2c(e)
    }
}

/// Calibration parameters extracted from EEPROM.
struct Params {
    k_vdd: f32,
    vdd25: f32,
    kv_ptat: f32,
    kt_ptat: f32,
    v_ptat25: f32,
    alpha_ptat: f32,
    gain_ee: f32,
    tgc: f32,
    cp_kv: f32,
    cp_kta: f32,
    resolution_ee: u8,
    calibration_mode_ee: u16,
    ks_ta: f32,
    ks_to: [f32; 5],
    ct: [f32; 5],
    alpha: [f32; PIXELS],
    offset: [f32; PIXELS],
    kta: [f32; PIXELS],
    kv: [f32; PIXELS],
    cp_alpha: [f32; 2],
    cp_offset: [f32; 2],
    il_chess_c: [f32; 3],
    broken: [bool; PIXELS],
}

pub struct Mlx90640<I, D> {
    i2c: I,
    delay: D,
    address: u8,
    params: Params,
    ram: [u16; RAM_WORDS],
    /// Object temperatures in °C, row-major, 24 rows x 32 columns.
    pub temperatures: [f32; PIXELS],
    /// Most recent ambient (die) temperature in °C.
    pub ambient: f32,
    /// Number of I2C transactions that failed and were retried.
    pub retries: u32,
}

/// Signed interpretation of the low `bits` bits of a value.
fn signed(value: u16, bits: u32) -> f32 {
    let v = value as i32;
    let half = 1i32 << (bits - 1);
    (if v >= half { v - (1 << bits) } else { v }) as f32
}

fn pow2(exp: i32) -> f32 {
    powf(2.0, exp as f32)
}

/// Split a word into four 4-bit signed nibbles, least significant first.
fn nibbles(word: u16) -> [f32; 4] {
    core::array::from_fn(|i| signed((word >> (4 * i)) & 0xF, 4))
}

impl<I: I2c, D: DelayNs> Mlx90640<I, D> {
    pub fn new(i2c: I, delay: D, address: u8) -> Self {
        Self {
            i2c,
            delay,
            address,
            params: Params {
                k_vdd: 0.0,
                vdd25: 0.0,
                kv_ptat: 0.0,
                kt_ptat: 0.0,
                v_ptat25: 0.0,
                alpha_ptat: 0.0,
                gain_ee: 0.0,
                tgc: 0.0,
                cp_kv: 0.0,
                cp_kta: 0.0,
                resolution_ee: 0,
                calibration_mode_ee: 0,
                ks_ta: 0.0,
                ks_to: [0.0; 5],
                ct: [0.0; 5],
                alpha: [0.0; PIXELS],
                offset: [0.0; PIXELS],
                kta: [0.0; PIXELS],
                kv: [0.0; PIXELS],
                cp_alpha: [0.0; 2],
                cp_offset: [0.0; 2],
                il_chess_c: [0.0; 3],
                broken: [false; PIXELS],
            },
            ram: [0; RAM_WORDS],
            temperatures: [0.0; PIXELS],
            ambient: 0.0,
            retries: 0,
        }
    }

    /// Reads the EEPROM and extracts calibration. Must succeed before
    /// `read_subpage` produces meaningful values.
    pub fn load_calibration(&mut self) -> Result<(), Error<I::Error>> {
        let mut ee = [0u16; EEPROM_WORDS];
        self.read_words(EEPROM_START, &mut ee)?;
        if ee.iter().all(|&w| w == 0) || ee.iter().all(|&w| w == 0xFFFF) {
            return Err(Error::BadEeprom);
        }
        self.extract_params(&ee);
        Ok(())
    }

    fn read_words(&mut self, start: u16, out: &mut [u16]) -> Result<(), I::Error> {
        let mut bytes = [0u8; CHUNK_WORDS * 2];
        for (i, chunk) in out.chunks_mut(CHUNK_WORDS).enumerate() {
            let addr = start + (i * CHUNK_WORDS) as u16;
            let buf = &mut bytes[..chunk.len() * 2];
            let mut attempt = 0;
            while let Err(e) = self.i2c.write_read(self.address, &addr.to_be_bytes(), buf) {
                attempt += 1;
                self.retries += 1;
                if attempt >= MAX_ATTEMPTS {
                    return Err(e);
                }
                self.delay.delay_us(RETRY_DELAY_US);
            }
            for (w, b) in chunk.iter_mut().zip(buf.chunks_exact(2)) {
                *w = u16::from_be_bytes([b[0], b[1]]);
            }
        }
        Ok(())
    }

    fn read_word(&mut self, addr: u16) -> Result<u16, I::Error> {
        let mut w = [0u16; 1];
        self.read_words(addr, &mut w)?;
        Ok(w[0])
    }

    fn write_word(&mut self, addr: u16, value: u16) -> Result<(), I::Error> {
        let a = addr.to_be_bytes();
        let v = value.to_be_bytes();
        let mut attempt = 0;
        while let Err(e) = self.i2c.write(self.address, &[a[0], a[1], v[0], v[1]]) {
            attempt += 1;
            self.retries += 1;
            if attempt >= MAX_ATTEMPTS {
                return Err(e);
            }
            self.delay.delay_us(RETRY_DELAY_US);
        }
        Ok(())
    }

    /// Sets the sub-page refresh rate and selects chess-pattern mode (the
    /// mode the sensor is calibrated in). A full frame takes two sub-pages.
    pub fn configure(&mut self, rate: RefreshRate) -> Result<(), Error<I::Error>> {
        let ctrl = self.read_word(REG_CONTROL1)?;
        let ctrl = (ctrl & !(0x7 << 7)) | ((rate as u16) << 7) | (1 << 12);
        self.write_word(REG_CONTROL1, ctrl)?;
        Ok(())
    }

    /// Blocks until a new sub-page is available, reads it and updates the
    /// matching half of `temperatures`. Returns the sub-page number (0 or 1).
    pub fn read_subpage(&mut self, emissivity: f32) -> Result<u8, Error<I::Error>> {
        let mut polls = 0u32;
        while self.read_word(REG_STATUS)? & 0x0008 == 0 {
            polls += 1;
            if polls > 50_000 {
                return Err(Error::Timeout);
            }
            self.delay.delay_us(500);
        }

        // Re-read if the sensor started overwriting RAM while we were reading.
        let mut status;
        let mut attempts = 0;
        loop {
            self.write_word(REG_STATUS, 0x0030)?;
            let mut ram = [0u16; RAM_WORDS];
            self.read_words(RAM_START, &mut ram)?;
            self.ram = ram;
            status = self.read_word(REG_STATUS)?;
            attempts += 1;
            if status & 0x0008 == 0 || attempts >= 5 {
                break;
            }
        }
        let control = self.read_word(REG_CONTROL1)?;
        let subpage = (status & 0x0001) as u8;
        self.calculate_to(control, subpage, emissivity);
        Ok(subpage)
    }

    fn extract_params(&mut self, ee: &[u16; EEPROM_WORDS]) {
        let p = &mut self.params;

        // VDD
        p.k_vdd = signed(ee[51] >> 8, 8) * 32.0;
        p.vdd25 = (((ee[51] & 0xFF) as f32 - 256.0) * 32.0) - 8192.0;

        // PTAT
        p.kv_ptat = signed(ee[50] >> 10, 6) / 4096.0;
        p.kt_ptat = signed(ee[50] & 0x03FF, 10) / 8.0;
        p.v_ptat25 = ee[49] as i16 as f32;
        p.alpha_ptat = ((ee[16] & 0xF000) >> 12) as f32 / 4.0 + 8.0;

        // Gain, TGC, resolution, KsTa
        p.gain_ee = ee[48] as i16 as f32;
        p.tgc = signed(ee[60] & 0xFF, 8) / 32.0;
        p.resolution_ee = ((ee[56] & 0x3000) >> 12) as u8;
        p.ks_ta = signed(ee[60] >> 8, 8) / 8192.0;

        // KsTo and corner temperatures
        let step = (((ee[63] & 0x3000) >> 12) * 10) as f32;
        p.ct[0] = -40.0;
        p.ct[1] = 0.0;
        p.ct[2] = ((ee[63] & 0x00F0) >> 4) as f32 * step;
        p.ct[3] = p.ct[2] + ((ee[63] & 0x0F00) >> 8) as f32 * step;
        p.ct[4] = 400.0;
        let ks_to_scale = pow2((ee[63] & 0x000F) as i32 + 8);
        p.ks_to[0] = signed(ee[61] & 0xFF, 8) / ks_to_scale;
        p.ks_to[1] = signed(ee[61] >> 8, 8) / ks_to_scale;
        p.ks_to[2] = signed(ee[62] & 0xFF, 8) / ks_to_scale;
        p.ks_to[3] = signed(ee[62] >> 8, 8) / ks_to_scale;
        p.ks_to[4] = -0.0002;

        // Compensation pixel
        let cp_alpha_scale = ((ee[32] & 0xF000) >> 12) as i32 + 27;
        p.cp_alpha[0] = signed(ee[57] & 0x03FF, 10) / pow2(cp_alpha_scale);
        p.cp_alpha[1] = (1.0 + signed(ee[57] >> 10, 6) / 128.0) * p.cp_alpha[0];
        p.cp_offset[0] = signed(ee[58] & 0x03FF, 10);
        p.cp_offset[1] = signed(ee[58] >> 10, 6) + p.cp_offset[0];
        let kta_scale1 = ((ee[56] & 0x00F0) >> 4) as i32 + 8;
        let kta_scale2 = (ee[56] & 0x000F) as i32;
        let kv_scale = ((ee[56] & 0x0F00) >> 8) as i32;
        p.cp_kta = signed(ee[59] & 0xFF, 8) / pow2(kta_scale1);
        p.cp_kv = signed(ee[59] >> 8, 8) / pow2(kv_scale);

        // Interleaved / chess correction
        p.calibration_mode_ee = ((ee[10] & 0x0800) >> 4) ^ 0x80;
        p.il_chess_c[0] = signed(ee[53] & 0x003F, 6) / 16.0;
        p.il_chess_c[1] = signed((ee[53] & 0x07C0) >> 6, 5) / 2.0;
        p.il_chess_c[2] = signed((ee[53] & 0xF800) >> 11, 5) / 8.0;

        // Per-row / per-column tables
        let mut acc_row = [0f32; HEIGHT];
        let mut occ_row = [0f32; HEIGHT];
        for i in 0..HEIGHT / 4 {
            acc_row[i * 4..i * 4 + 4].copy_from_slice(&nibbles(ee[34 + i]));
            occ_row[i * 4..i * 4 + 4].copy_from_slice(&nibbles(ee[18 + i]));
        }
        let mut acc_col = [0f32; WIDTH];
        let mut occ_col = [0f32; WIDTH];
        for i in 0..WIDTH / 4 {
            acc_col[i * 4..i * 4 + 4].copy_from_slice(&nibbles(ee[40 + i]));
            occ_col[i * 4..i * 4 + 4].copy_from_slice(&nibbles(ee[24 + i]));
        }

        let acc_rem_scale = pow2((ee[32] & 0x000F) as i32);
        let acc_col_scale = pow2(((ee[32] & 0x00F0) >> 4) as i32);
        let acc_row_scale = pow2(((ee[32] & 0x0F00) >> 8) as i32);
        let alpha_scale = pow2(((ee[32] & 0xF000) >> 12) as i32 + 30);
        let alpha_ref = ee[33] as f32;

        let occ_rem_scale = pow2((ee[16] & 0x000F) as i32);
        let occ_col_scale = pow2(((ee[16] & 0x00F0) >> 4) as i32);
        let occ_row_scale = pow2(((ee[16] & 0x0F00) >> 8) as i32);
        let offset_ref = ee[17] as i16 as f32;

        // Kta / Kv per (row parity, column parity)
        let kta_rc = [
            signed(ee[54] >> 8, 8),   // odd row, odd column (1-based)
            signed(ee[55] >> 8, 8),   // odd row, even column
            signed(ee[54] & 0xFF, 8), // even row, odd column
            signed(ee[55] & 0xFF, 8), // even row, even column
        ];
        let kv_rc = [
            signed((ee[52] >> 12) & 0xF, 4),
            signed((ee[52] >> 4) & 0xF, 4),
            signed((ee[52] >> 8) & 0xF, 4),
            signed(ee[52] & 0xF, 4),
        ];
        let kta_scale1 = pow2(kta_scale1);
        let kta_scale2 = pow2(kta_scale2);
        let kv_scale = pow2(kv_scale);

        for row in 0..HEIGHT {
            for col in 0..WIDTH {
                let px = row * WIDTH + col;
                let word = ee[64 + px];
                let split = 2 * (row % 2) + col % 2;

                p.alpha[px] = (alpha_ref
                    + acc_row[row] * acc_row_scale
                    + acc_col[col] * acc_col_scale
                    + signed((word & 0x03F0) >> 4, 6) * acc_rem_scale)
                    / alpha_scale;

                p.offset[px] = offset_ref
                    + occ_row[row] * occ_row_scale
                    + occ_col[col] * occ_col_scale
                    + signed((word & 0xFC00) >> 10, 6) * occ_rem_scale;

                p.kta[px] =
                    (kta_rc[split] + signed((word & 0x000E) >> 1, 3) * kta_scale2) / kta_scale1;
                p.kv[px] = kv_rc[split] / kv_scale;

                p.broken[px] = word == 0;
            }
        }
    }

    fn vdd(&self, control: u16) -> f32 {
        let p = &self.params;
        let raw = self.ram[810] as i16 as f32;
        let resolution_ram = ((control & 0x0C00) >> 10) as i32;
        let correction = pow2(p.resolution_ee as i32 - resolution_ram);
        (correction * raw - p.vdd25) / p.k_vdd + 3.3
    }

    fn ta(&self, vdd: f32) -> f32 {
        let p = &self.params;
        let ptat = self.ram[800] as i16 as f32;
        let ptat_art = self.ram[768] as i16 as f32;
        let ptat_art = ptat / (ptat * p.alpha_ptat + ptat_art) * 262144.0;
        (ptat_art / (1.0 + p.kv_ptat * (vdd - 3.3)) - p.v_ptat25) / p.kt_ptat + 25.0
    }

    #[allow(clippy::needless_range_loop, reason = "px indexes several parallel arrays")]
    #[doc(hidden)]
    pub fn calculate_to(&mut self, control: u16, subpage: u8, emissivity: f32) {
        let vdd = self.vdd(control);
        let ta = self.ta(vdd);
        self.ambient = ta;
        let p = &self.params;
        let ram = &self.ram;

        // Reflected temperature: for a sensor in open air Melexis recommends
        // Ta - 8 °C.
        let tr = ta - 8.0;
        let ta4 = powf(ta + 273.15, 4.0);
        let tr4 = powf(tr + 273.15, 4.0);
        let ta_tr = tr4 - (tr4 - ta4) / emissivity;

        let alpha_corr_r = {
            let r2 = 1.0 + p.ks_to[2] * p.ct[2];
            [
                1.0 / (1.0 + p.ks_to[0] * 40.0),
                1.0,
                r2,
                r2 * (1.0 + p.ks_to[3] * (p.ct[3] - p.ct[2])),
            ]
        };

        let gain = p.gain_ee / ram[778] as i16 as f32;
        let mode = (control & 0x1000) >> 5;
        let dt = ta - 25.0;
        let dv = vdd - 3.3;

        let mut ir_cp = [ram[776] as i16 as f32 * gain, ram[808] as i16 as f32 * gain];
        let cp_comp = (1.0 + p.cp_kta * dt) * (1.0 + p.cp_kv * dv);
        ir_cp[0] -= p.cp_offset[0] * cp_comp;
        ir_cp[1] -= if mode == p.calibration_mode_ee {
            p.cp_offset[1]
        } else {
            p.cp_offset[1] + p.il_chess_c[0]
        } * cp_comp;

        let sp = subpage as usize;
        for px in 0..PIXELS {
            let il_pattern = ((px / 32) % 2) as i32;
            let chess_pattern = il_pattern ^ (px % 2) as i32;
            let pattern = if mode == 0 { il_pattern } else { chess_pattern };
            if pattern != subpage as i32 {
                continue;
            }
            let n = px as i32;
            let conversion_pattern = ((n + 2) / 4 - (n + 3) / 4 + (n + 1) / 4 - n / 4)
                * (1 - 2 * il_pattern);

            let mut ir = ram[px] as i16 as f32 * gain;
            ir -= p.offset[px] * (1.0 + p.kta[px] * dt) * (1.0 + p.kv[px] * dv);
            if mode != p.calibration_mode_ee {
                ir += p.il_chess_c[2] * (2 * il_pattern - 1) as f32
                    - p.il_chess_c[1] * conversion_pattern as f32;
            }
            ir /= emissivity;
            ir -= p.tgc * ir_cp[sp];

            let alpha = (p.alpha[px] - p.tgc * p.cp_alpha[sp]) * (1.0 + p.ks_ta * dt);

            let sx = alpha * alpha * alpha * (ir + alpha * ta_tr);
            let sx = sqrtf(sqrtf(sx)) * p.ks_to[1];
            let to = sqrtf(sqrtf(ir / (alpha * (1.0 - p.ks_to[1] * 273.15) + sx) + ta_tr)) - 273.15;

            let range = if to < p.ct[1] {
                0
            } else if to < p.ct[2] {
                1
            } else if to < p.ct[3] {
                2
            } else {
                3
            };
            self.temperatures[px] = sqrtf(sqrtf(
                ir / (alpha * alpha_corr_r[range] * (1.0 + p.ks_to[range] * (to - p.ct[range])))
                    + ta_tr,
            )) - 273.15;
        }

        // Replace pixels flagged as broken in EEPROM with the mean of their
        // valid horizontal/vertical neighbours.
        for px in 0..PIXELS {
            if !p.broken[px] {
                continue;
            }
            let (row, col) = (px / WIDTH, px % WIDTH);
            let mut sum = 0.0;
            let mut count = 0;
            let neighbours = [
                (row > 0).then(|| px - WIDTH),
                (row + 1 < HEIGHT).then(|| px + WIDTH),
                (col > 0).then(|| px - 1),
                (col + 1 < WIDTH).then(|| px + 1),
            ];
            for n in neighbours.into_iter().flatten() {
                if !p.broken[n] {
                    sum += self.temperatures[n];
                    count += 1;
                }
            }
            if count > 0 {
                self.temperatures[px] = sum / count as f32;
            }
        }
    }
}
