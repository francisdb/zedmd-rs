//! Pixel scan example.
//!
//! Scans a single white pixel across every position on the display from
//! (0, 0) to the last pixel, row by row. Useful for verifying that the
//! USB connection and zone-streaming protocol are working correctly and
//! that every pixel on the panel is functional.
//!
//! Run with:
//! ```sh
//! cargo run --example pixel_scan
//! ```
use log::{error, info};
use std::io;
use std::process::ExitCode;
use std::thread::sleep;
use std::time::{Duration, Instant};
use zedmd_rs::color::rgb565;
use zedmd_rs::zedmd::connect;

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

fn test_connect() -> io::Result<()> {
    let mut comm = connect()?;
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

    // Start keep-alive (handled inline during render calls)
    comm.run()?;

    let total_pixels = width * height;
    info!(
        "Scanning white pixel from (0,0) to ({},{}) - {} pixels total",
        width - 1,
        height - 1,
        total_pixels
    );

    let mut pixels = vec![0u16; total_pixels];
    let scan_start = Instant::now();

    for i in 0..total_pixels {
        // Clear previous pixel, set current pixel to white (RGB565 0xFFFF)
        if i > 0 {
            pixels[i - 1] = 0;
        }
        pixels[i] = rgb565(255, 255, 255);

        let frame_start = Instant::now();
        comm.render_rgb565_frame(&pixels)?;
        let frame_ms = frame_start.elapsed().as_secs_f64() * 1000.0;

        let x = i % width;
        let y = i / width;
        // Log once per row to avoid flooding the output
        if x == 0 {
            info!("Row {:2} - frame took {:.2}ms", y, frame_ms);
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
    comm.clear_screen()?;

    sleep(Duration::from_millis(500));
    comm.stop()?;

    Ok(())
}
