//! Plasma animation example.
//!
//! Renders a classic sine-wave colour plasma effect in an infinite loop,
//! capped at the panel's reported minimum refresh rate (read from the device
//! at handshake). Because the plasma changes every zone on every frame the
//! background USB thread runs at full device throughput (~9–10 fps on a
//! 128×32 panel with a USB package size of 512 bytes).
//!
//! If the submit rate exceeds USB throughput, pending frames are dropped
//! (latest-frame-wins) — visible as skipped animation steps.
//!
//! Run with:
//! ```sh
//! cargo run --example plasma                    # USB
//! cargo run --example plasma -- --wifi          # ZeDMD-WiFi.local
//! cargo run --example plasma -- --wifi 10.0.1.7 # explicit host
//! ```
use log::{error, info};
use std::io;
use std::process::ExitCode;
use std::thread::sleep;
use std::time::{Duration, Instant};
use zedmd_rs::color::hsv_to_rgb565;
use zedmd_rs::zedmd::{ZeDMDComm, connect, connect_wifi};

fn main() -> ExitCode {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    match run() {
        Ok(_) => ExitCode::SUCCESS,
        Err(e) => {
            error!("Error: {}", e);
            ExitCode::FAILURE
        }
    }
}

fn render_plasma(pixels: &mut [u16], width: usize, height: usize, t: f32) {
    for y in 0..height {
        for x in 0..width {
            let xf = x as f32 / width as f32;
            let yf = y as f32 / height as f32;
            let v = (xf * 6.0 + t).sin()
                + (yf * 4.0 + t * 0.7).sin()
                + ((xf + yf) * 5.0 + t * 1.3).sin()
                + ((xf * xf + yf * yf).sqrt() * 8.0 - t * 2.0).sin();
            let hue = (v * 45.0).rem_euclid(360.0);
            pixels[y * width + x] = hsv_to_rgb565(hue, 1.0, 1.0);
        }
    }
}

fn parse_wifi_arg() -> Option<String> {
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--wifi" {
            return Some(args.next().unwrap_or_else(|| "ZeDMD-WiFi.local".into()));
        }
    }
    None
}

fn run() -> io::Result<()> {
    let mut comm: ZeDMDComm = match parse_wifi_arg() {
        Some(host) => {
            info!("Connecting over WiFi to {}", host);
            connect_wifi(&host)?
        }
        None => connect()?,
    };
    comm.run()?;

    let width = comm.width() as usize;
    let height = comm.height() as usize;
    // Cap submit rate to the panel's minimum refresh rate so we don't submit
    // faster than the device expects to be driven.
    let refresh_rate = comm.panel_min_refresh_rate().max(1);
    info!(
        "Connected: {}x{}, refresh rate cap: {} fps (RGB565)",
        width, height, refresh_rate
    );

    let mut pixels = vec![0u16; width * height];
    let start = Instant::now();
    let mut frames = 0u64;
    let frame_budget = Duration::from_secs_f64(1.0 / refresh_rate as f64);

    loop {
        let frame_start = Instant::now();
        let t = start.elapsed().as_secs_f32();
        render_plasma(&mut pixels, width, height, t);
        comm.render_rgb565_frame(&pixels);
        frames += 1;

        if frames.is_multiple_of(150) {
            let elapsed = start.elapsed().as_secs_f64();
            let submit_fps = frames as f64 / elapsed;
            let usb_fps = comm.usb_fps();
            let dropped = submit_fps - usb_fps;
            info!(
                "submit {:.1} fps  |  usb {:.1} fps  |  dropped {:.1} fps",
                submit_fps,
                usb_fps,
                dropped.max(0.0)
            );
        }

        if let Some(remaining) = frame_budget.checked_sub(frame_start.elapsed()) {
            sleep(remaining);
        }
    }
}
