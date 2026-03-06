//! Scaling example.
//!
//! Demonstrates client-side RGB888 frame scaling using `set_frame_size` and
//! `enable_upscaling`. Cycles through three modes in an infinite loop:
//!
//! - **Native** — source matches panel size exactly; no scaling.
//! - **Downscale 2×** — source is `panel_w*2 × panel_h*2`; the 2× majority-vote
//!   halve reduces it to panel size before sending.
//! - **Upscale 2×** — source is `panel_w/2 × panel_h/2`; pixel-doubled to panel
//!   size. Useful for seeing how thin-feature content (fonts, lines) degrades
//!   when the source resolution doesn't match the panel.
//!
//! All frames are rendered as true RGB888 (`enable_true_rgb888`) so the colour
//! conversion path doesn't mask scaling artefacts.
//!
//! Run with:
//! ```sh
//! cargo run --example scaling
//! ```
use log::{error, info};
use std::io;
use std::process::ExitCode;
use std::thread::sleep;
use std::time::{Duration, Instant};
use zedmd_rs::color::hsv_to_rgb888;
use zedmd_rs::zedmd::connect;

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

fn render_plasma_rgb888(pixels: &mut [u8], width: usize, height: usize, t: f32) {
    for y in 0..height {
        for x in 0..width {
            let xf = x as f32 / width as f32;
            let yf = y as f32 / height as f32;
            let v = (xf * 6.0 + t).sin()
                + (yf * 4.0 + t * 0.7).sin()
                + ((xf + yf) * 5.0 + t * 1.3).sin()
                + ((xf * xf + yf * yf).sqrt() * 8.0 - t * 1.1).sin();
            let hue = (v * 45.0 + t * 30.0).rem_euclid(360.0);
            let (r, g, b) = hsv_to_rgb888(hue, 1.0, 1.0);
            let i = (y * width + x) * 3;
            pixels[i] = r;
            pixels[i + 1] = g;
            pixels[i + 2] = b;
        }
    }
}

fn run() -> io::Result<()> {
    let mut comm = connect()?;
    comm.enable_true_rgb888();
    comm.run()?;

    let panel_w = comm.width() as usize;
    let panel_h = comm.height() as usize;
    let min_refresh_rate = comm.panel_min_refresh_rate().max(1);
    let frame_budget = Duration::from_secs_f64(1.0 / min_refresh_rate as f64);
    info!(
        "Connected: {}x{}, min refresh rate: {}Hz (true RGB888)",
        panel_w, panel_h, min_refresh_rate
    );

    // Each mode: (label, source_width, source_height, upscaling_enabled)
    let modes: &[(&str, usize, usize, bool)] = &[
        ("Native (panel size)", panel_w, panel_h, false),
        (
            "Downscale 2x (double source)",
            panel_w * 2,
            panel_h * 2,
            false,
        ),
        ("Upscale 2x (half source)", panel_w / 2, panel_h / 2, true),
    ];

    let hold = Duration::from_secs(6);

    loop {
        for (label, src_w, src_h, upscaling) in modes {
            info!(
                "Mode: {} ({}x{} → {}x{})",
                label, src_w, src_h, panel_w, panel_h
            );
            comm.set_frame_size(*src_w as u32, *src_h as u32);
            if *upscaling {
                comm.enable_upscaling();
            } else {
                comm.disable_upscaling();
            }

            let mut pixels = vec![0u8; src_w * src_h * 3];
            let start = Instant::now();
            let mut frames = 0u64;

            loop {
                let frame_start = Instant::now();
                let elapsed = frame_start.duration_since(start);
                if elapsed >= hold {
                    break;
                }
                let t = elapsed.as_secs_f32();

                render_plasma_rgb888(&mut pixels, *src_w, *src_h, t);
                comm.render_rgb888_frame(&pixels);
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
    }
}
