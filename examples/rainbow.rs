//! Animated rainbow example.
//!
//! Renders an animated rainbow where the full hue spectrum shifts
//! horizontally over time, in an infinite loop.
//!
//! Capped at the panel's minimum refresh rate (read from the device
//! at handshake). Submit and USB fps are logged every 150 frames.
//!
//! Run with:
//! ```sh
//! cargo run --example rainbow                    # USB
//! cargo run --example rainbow -- --wifi          # ZeDMD-WiFi.local
//! cargo run --example rainbow -- --wifi 10.0.1.7 # explicit host
//! ```
use log::{error, info};
use std::io;
use std::process::ExitCode;
use std::thread::sleep;
use std::time::{Duration, Instant};
use zedmd::color::hsv_to_rgb565;
use zedmd::{ZeDMDComm, connect, connect_wifi};

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

fn render_rainbow(pixels: &mut [u16], width: usize, height: usize, t: f32) {
    for y in 0..height {
        for x in 0..width {
            let hue = (x as f32 / width as f32 * 360.0 + t * 120.0).rem_euclid(360.0);
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
    let min_refresh_rate = comm.panel_min_refresh_rate().max(1);
    let frame_budget = Duration::from_secs_f64(1.0 / min_refresh_rate as f64);
    info!(
        "Connected: {}x{}, min refresh rate: {}Hz",
        width, height, min_refresh_rate
    );

    let mut pixels = vec![0u16; width * height];
    let start = Instant::now();
    let mut frames = 0u64;

    loop {
        let frame_start = Instant::now();
        let t = start.elapsed().as_secs_f32();

        render_rainbow(&mut pixels, width, height, t);
        comm.render_rgb565_frame(&pixels);
        frames += 1;

        if frames.is_multiple_of(150) {
            let elapsed_secs = start.elapsed().as_secs_f64();
            let submit_fps = frames as f64 / elapsed_secs;
            let usb_fps = comm.usb_fps();
            let dropped = (submit_fps - usb_fps).max(0.0);
            info!(
                "submit {:.1} fps  |  usb {:.1} fps  |  dropped {:.1} fps",
                submit_fps, usb_fps, dropped
            );
        }

        if let Some(remaining) = frame_budget.checked_sub(frame_start.elapsed()) {
            sleep(remaining);
        }
    }
}
