//! True RGB888 plasma animation example — command `0x04` (`RGB888ZonesStream`).
//!
//! Unlike [`plasma_rgb888`] which converts RGB888 → RGB565 before sending,
//! this example enables [`enable_true_rgb888`] so frames are sent with 3
//! bytes/pixel via command `0x04`.
//!
//! > **Warning**: this command is known to be broken in the reference libzedmd
//! > implementation and in the firmware. It will most likely fail with ACK
//! > timeouts or a miniz decompression error on the device. The example is
//! > provided purely for testing and investigation purposes.
//! > See <https://github.com/PPUC/libzedmd/issues/47>.
//!
//! Run with:
//! ```sh
//! cargo run --example plasma_rgb888_true                    # USB
//! cargo run --example plasma_rgb888_true -- --wifi          # ZeDMD-WiFi.local
//! cargo run --example plasma_rgb888_true -- --wifi 10.0.1.7 # explicit host
//! ```
use log::{error, info};
use std::io;
use std::process::ExitCode;
use std::thread::sleep;
use std::time::{Duration, Instant};
use zedmd_rs::color::hsv_to_rgb888;
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

fn render_plasma(pixels: &mut [u8], width: usize, height: usize, t: f32) {
    for y in 0..height {
        for x in 0..width {
            let xf = x as f32 / width as f32;
            let yf = y as f32 / height as f32;
            let v = (xf * 6.0 + t).sin()
                + (yf * 4.0 + t * 0.7).sin()
                + ((xf + yf) * 5.0 + t * 1.3).sin()
                + ((xf * xf + yf * yf).sqrt() * 8.0 - t * 2.0).sin();
            let hue = (v * 45.0).rem_euclid(360.0);
            let (r, g, b) = hsv_to_rgb888(hue, 1.0, 1.0);
            let i = (y * width + x) * 3;
            pixels[i] = r;
            pixels[i + 1] = g;
            pixels[i + 2] = b;
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
    comm.enable_true_rgb888();
    comm.run()?;

    let width = comm.width() as usize;
    let height = comm.height() as usize;
    let refresh_rate = comm.panel_min_refresh_rate().max(1);
    info!(
        "Connected: {}x{}, refresh rate cap: {} fps (true RGB888 — command 0x04, EXPERIMENTAL)",
        width, height, refresh_rate
    );

    let mut pixels = vec![0u8; width * height * 3];
    let start = Instant::now();
    let mut frames = 0u64;
    let frame_budget = Duration::from_secs_f64(1.0 / refresh_rate as f64);

    loop {
        let frame_start = Instant::now();
        let t = start.elapsed().as_secs_f32();
        render_plasma(&mut pixels, width, height, t);
        comm.render_rgb888_frame(&pixels);
        frames += 1;

        if frames.is_multiple_of(150) {
            let elapsed = start.elapsed().as_secs_f64();
            let submit_fps = frames as f64 / elapsed;
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
