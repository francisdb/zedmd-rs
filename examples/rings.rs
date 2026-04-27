//! Animated concentric rings example.
//!
//! Renders outward-travelling colour rings centred on the display.
//!
//! **This example is heavier on the USB connection than `plasma`.** Because
//! the hue and brightness both shift across the entire display every frame,
//! every zone is dirty on every frame and no delta compression benefit is
//! gained — the device receives the maximum possible data per frame.
//! Expect USB throughput of ~6–7 fps on a 128×32 panel with a 512-byte USB
//! package size, compared to ~9–10 fps for plasma which benefits more from
//! zone hashing.
//!
//! The animation speed is scaled down (`time_scale = 0.1`) so the rings
//! move slowly despite the limited frame rate.
//!
//! Run with:
//! ```sh
//! cargo run --example rings                    # USB
//! cargo run --example rings -- --wifi          # ZeDMD-WiFi.local
//! cargo run --example rings -- --wifi 10.0.1.7 # explicit host
//! ```
use log::{error, info};
use std::f32::consts::PI;
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

fn render_rings(pixels: &mut [u16], width: usize, height: usize, t: f32) {
    let cx = width as f32 / 2.0;
    let cy = height as f32 / 2.0;
    // Normalise both axes by the same value so rings are true circles.
    let scale = cx.min(cy);
    for y in 0..height {
        for x in 0..width {
            let dx = (x as f32 - cx) / scale;
            let dy = (y as f32 - cy) / scale;
            let dist = (dx * dx + dy * dy).sqrt();
            let hue = ((dist * 3.0 - t * 4.0) * 360.0).rem_euclid(360.0);
            let v = ((dist * 2.0 - t * 4.0) * PI).sin() * 0.5 + 0.5;
            pixels[y * width + x] = hsv_to_rgb565(hue, 1.0, v);
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
    let refresh_rate = comm.panel_min_refresh_rate().max(1);
    info!(
        "Connected: {}x{}, refresh rate cap: {} fps",
        width, height, refresh_rate
    );

    let mut pixels = vec![0u16; width * height];
    let start = Instant::now();
    let mut frames = 0u64;
    let time_scale = 0.1f32;
    let frame_budget = Duration::from_secs_f64(1.0 / refresh_rate as f64);

    loop {
        let frame_start = Instant::now();
        let t = start.elapsed().as_secs_f32() * time_scale;
        render_rings(&mut pixels, width, height, t);
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
