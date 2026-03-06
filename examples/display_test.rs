//! Display test example.
//!
//! Cycles through a series of test screens: solid colours, colour bars,
//! gradients, hue/saturation and hue/brightness selectors, a checkerboard,
//! a scrolling scanline, and an animated rainbow. Loops forever.
//!
//! Static screens are sent once and held for their duration; animated screens
//! are submitted at up to the panel's minimum refresh rate (read from the
//! device at handshake) with actual USB throughput logged periodically.
//!
//! Run with:
//! ```sh
//! cargo run --example display_test
//! ```
use log::{error, info};
use std::io;
use std::process::ExitCode;
use std::thread::sleep;
use std::time::{Duration, Instant};
use zedmd_rs::color::{hsv_to_rgb565, rgb565};
use zedmd_rs::zedmd::connect;

fn main() -> ExitCode {
    env_logger::Builder::new()
        .format_timestamp(None)
        .format_module_path(false)
        .format_target(false)
        .filter_level(log::LevelFilter::Info)
        .init();

    match run() {
        Ok(_) => ExitCode::SUCCESS,
        Err(e) => {
            error!("Error: {}", e);
            ExitCode::FAILURE
        }
    }
}

// ── Screens ───────────────────────────────────────────────────────────────────

/// Solid colour fill.
fn screen_solid(pixels: &mut [u16], color: u16) {
    pixels.fill(color);
}

/// Horizontal colour gradient between two colours.
fn screen_gradient_h(pixels: &mut [u16], width: usize, height: usize, from: u16, to: u16) {
    let fr = ((from >> 11) & 0x1F) as f32;
    let fg = ((from >> 5) & 0x3F) as f32;
    let fb = (from & 0x1F) as f32;
    let tr = ((to >> 11) & 0x1F) as f32;
    let tg = ((to >> 5) & 0x3F) as f32;
    let tb = (to & 0x1F) as f32;
    for y in 0..height {
        for x in 0..width {
            let t = x as f32 / (width - 1) as f32;
            let r = (fr + t * (tr - fr)) as u16;
            let g = (fg + t * (tg - fg)) as u16;
            let b = (fb + t * (tb - fb)) as u16;
            pixels[y * width + x] = (r << 11) | (g << 5) | b;
        }
    }
}

/// Full rainbow: hue sweeps 0..360 left to right.
fn screen_rainbow(pixels: &mut [u16], width: usize, height: usize) {
    for y in 0..height {
        for x in 0..width {
            let hue = x as f32 / width as f32 * 360.0;
            pixels[y * width + x] = hsv_to_rgb565(hue, 1.0, 1.0);
        }
    }
}

/// 2-D hue/saturation selector: X = hue (0..360), Y = saturation (1..0 top to bottom).
fn screen_hue_saturation(pixels: &mut [u16], width: usize, height: usize) {
    for y in 0..height {
        let sat = 1.0 - y as f32 / (height - 1) as f32;
        for x in 0..width {
            let hue = x as f32 / (width - 1) as f32 * 360.0;
            pixels[y * width + x] = hsv_to_rgb565(hue, sat, 1.0);
        }
    }
}

/// 2-D hue/brightness selector: X = hue (0..360), Y = brightness (1..0 top to bottom).
fn screen_hue_brightness(pixels: &mut [u16], width: usize, height: usize) {
    for y in 0..height {
        let val = 1.0 - y as f32 / (height - 1) as f32;
        for x in 0..width {
            let hue = x as f32 / (width - 1) as f32 * 360.0;
            pixels[y * width + x] = hsv_to_rgb565(hue, 1.0, val);
        }
    }
}

/// Animated rainbow: hue shifts over time.
fn screen_rainbow_animated(pixels: &mut [u16], width: usize, height: usize, t: f32) {
    for y in 0..height {
        for x in 0..width {
            let hue = (x as f32 / width as f32 * 360.0 + t * 120.0).rem_euclid(360.0);
            pixels[y * width + x] = hsv_to_rgb565(hue, 1.0, 1.0);
        }
    }
}

/// Checkerboard of two colours.
fn screen_checkerboard(
    pixels: &mut [u16],
    width: usize,
    height: usize,
    a: u16,
    b: u16,
    cell: usize,
) {
    for y in 0..height {
        for x in 0..width {
            pixels[y * width + x] = if (x / cell + y / cell).is_multiple_of(2) {
                a
            } else {
                b
            };
        }
    }
}

/// Vertical colour bars: R G B C M Y W K.
fn screen_colour_bars(pixels: &mut [u16], width: usize, height: usize) {
    let bars: &[u16] = &[
        rgb565(255, 0, 0),     // red
        rgb565(0, 255, 0),     // green
        rgb565(0, 0, 255),     // blue
        rgb565(0, 255, 255),   // cyan
        rgb565(255, 0, 255),   // magenta
        rgb565(255, 255, 0),   // yellow
        rgb565(255, 255, 255), // white
        rgb565(0, 0, 0),       // black
    ];
    let n = bars.len();
    for y in 0..height {
        for x in 0..width {
            pixels[y * width + x] = bars[x * n / width];
        }
    }
}

/// Scrolling horizontal scan-line.
fn screen_scanline(pixels: &mut [u16], width: usize, height: usize, row: usize, color: u16) {
    pixels.fill(0);
    let y = row % height;
    for x in 0..width {
        pixels[y * width + x] = color;
    }
}

// ── Main loop ─────────────────────────────────────────────────────────────────

fn run() -> io::Result<()> {
    let mut comm = connect()?;
    comm.disable_debug()?;
    comm.run()?;

    let width = comm.width() as usize;
    let height = comm.height() as usize;
    let min_refresh_rate = comm.panel_min_refresh_rate().max(1);
    let frame_budget = Duration::from_secs_f64(1.0 / min_refresh_rate as f64);
    info!(
        "Connected: {}x{}, min refresh rate: {}Hz",
        width, height, min_refresh_rate
    );

    let mut pixels = vec![0u16; width * height];

    // Each entry: (name, hold_seconds, is_animated)
    let screens: &[(&str, f32, bool)] = &[
        ("Red", 2.0, false),
        ("Green", 2.0, false),
        ("Blue", 2.0, false),
        ("White", 2.0, false),
        ("Black", 1.0, false),
        ("Colour bars", 3.0, false),
        ("Red→Blue gradient", 3.0, false),
        ("Rainbow", 3.0, false),
        ("Hue/Saturation", 4.0, false),
        ("Hue/Brightness", 4.0, false),
        ("Checkerboard", 2.0, false),
        ("Scanline", 3.0, true),
        ("Animated rainbow", 4.0, true),
    ];

    loop {
        for (name, hold_secs, animated) in screens {
            info!("Screen: {}", name);
            let start = Instant::now();
            let hold = Duration::from_secs_f32(*hold_secs);
            let mut frame = 0u32;

            loop {
                let frame_start = Instant::now();
                let elapsed = frame_start.duration_since(start);
                if elapsed >= hold {
                    break;
                }
                let t = elapsed.as_secs_f32();

                match *name {
                    "Red" => screen_solid(&mut pixels, rgb565(255, 0, 0)),
                    "Green" => screen_solid(&mut pixels, rgb565(0, 255, 0)),
                    "Blue" => screen_solid(&mut pixels, rgb565(0, 0, 255)),
                    "White" => screen_solid(&mut pixels, rgb565(255, 255, 255)),
                    "Black" => screen_solid(&mut pixels, 0),
                    "Colour bars" => screen_colour_bars(&mut pixels, width, height),
                    "Red→Blue gradient" => screen_gradient_h(
                        &mut pixels,
                        width,
                        height,
                        rgb565(255, 0, 0),
                        rgb565(0, 0, 255),
                    ),
                    "Rainbow" => screen_rainbow(&mut pixels, width, height),
                    "Hue/Saturation" => screen_hue_saturation(&mut pixels, width, height),
                    "Hue/Brightness" => screen_hue_brightness(&mut pixels, width, height),
                    "Checkerboard" => {
                        screen_checkerboard(&mut pixels, width, height, rgb565(255, 255, 255), 0, 4)
                    }
                    "Scanline" => screen_scanline(
                        &mut pixels,
                        width,
                        height,
                        frame as usize,
                        rgb565(0, 255, 0),
                    ),
                    "Animated rainbow" => screen_rainbow_animated(&mut pixels, width, height, t),
                    _ => screen_solid(&mut pixels, 0),
                }

                comm.render_rgb565_frame(&pixels);
                frame += 1;

                // Static screens only need one render; sleep the remainder.
                // Animated screens sleep one frame budget to cap the submit rate.
                if !animated {
                    sleep(hold - elapsed);
                    break;
                } else if let Some(remaining) = frame_budget.checked_sub(frame_start.elapsed()) {
                    sleep(remaining);
                }
            }

            if *animated {
                let fps = frame as f64 / *hold_secs as f64;
                info!("  {} frames in {:.1}s = {:.1} fps", frame, hold_secs, fps);
            }
        }
    }
}
