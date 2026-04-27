//! Pixel scan example.
//!
//! Scans a single white pixel across every position on the display from
//! (0, 0) to the last pixel, row by row. Useful for verifying that the
//! USB connection and zone-streaming protocol are working correctly and
//! that every pixel on the panel is functional.
//!
//! Each pixel is submitted via the async frame queue and then
//! [`wait_for_frame`] is called to block until the device has actually
//! received and displayed it before moving to the next position. Without
//! this wait the queue would run at thousands of frames per second,
//! dropping almost every frame and making the scan appear to jump to a
//! far-away pixel.
//!
//! Run with:
//! ```sh
//! cargo run --example pixel_scan                    # USB
//! cargo run --example pixel_scan -- --wifi          # ZeDMD-WiFi.local
//! cargo run --example pixel_scan -- --wifi 10.0.1.7 # explicit host
//! ```
use log::{error, info};
use std::io;
use std::process::ExitCode;
use std::thread::sleep;
use std::time::{Duration, Instant};
use zedmd::color::rgb565;
use zedmd::{ZeDMDComm, connect, connect_wifi};

fn main() -> ExitCode {
    // Initialize the logger with color support and debug level
    env_logger::Builder::new()
        .format_timestamp(None)
        .format_module_path(false)
        .format_target(false)
        .filter_level(log::LevelFilter::Info)
        .init();

    // Test the connection to the ZeDMD device
    match test_connect() {
        Ok(_) => ExitCode::SUCCESS,
        Err(e) => {
            error!("Error: {}", e);
            ExitCode::FAILURE
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

fn test_connect() -> io::Result<()> {
    let mut comm: ZeDMDComm = match parse_wifi_arg() {
        Some(host) => {
            info!("Connecting over WiFi to {}", host);
            connect_wifi(&host)?
        }
        None => connect()?,
    };
    comm.run()?;
    comm.disable_debug()?;
    info!("Connected to ZeDMD device");

    let width = comm.width() as usize;
    let height = comm.height() as usize;
    info!("Display dimensions: {}x{}", width, height);

    if width == 0 || height == 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Device reported zero dimensions",
        ));
    }

    let total_pixels = width * height;
    // panel_min_refresh_rate is the minimum PWM scan rate of the LED matrix
    // hardware. Sleeping for 1/min_rate gives the panel at least one full
    // refresh cycle to display the pixel before we move on.
    let min_refresh_rate = comm.panel_min_refresh_rate().max(1);
    let min_panel_frame = Duration::from_secs_f64(1.0 / min_refresh_rate as f64);
    info!(
        "Scanning white pixel from (0,0) to ({},{}) - {} pixels total, min panel refresh {}Hz",
        width - 1,
        height - 1,
        total_pixels,
        min_refresh_rate
    );

    let mut pixels = vec![0u16; total_pixels];
    let scan_start = Instant::now();
    let mut last_frame_id = 0u64;

    for i in 0..total_pixels {
        if i > 0 {
            pixels[i - 1] = 0;
        }
        pixels[i] = rgb565(255, 255, 255);

        last_frame_id = comm.render_rgb565_frame(&pixels);
        // Wait for USB send, then sleep one panel refresh cycle so the
        // hardware has time to display the pixel before we move on.
        comm.wait_for_frame(last_frame_id);
        sleep(min_panel_frame);

        let x = i % width;
        let y = i / width;
        if x == 0 {
            info!("Row {:2}", y);
        }
    }

    let total_ms = scan_start.elapsed().as_secs_f64() * 1000.0;
    let fps = total_pixels as f64 / (total_ms / 1000.0);
    info!(
        "Scan complete: {} pixels in {:.0}ms = {:.1} fps ({:.2}ms/frame)",
        total_pixels,
        total_ms,
        fps,
        total_ms / total_pixels as f64
    );
    // wait_for_frame already guarantees the last pixel is visible before we
    // clear, but be explicit here for clarity.
    comm.wait_for_frame(last_frame_id);
    comm.clear_screen()?;
    comm.stop()?;

    Ok(())
}
