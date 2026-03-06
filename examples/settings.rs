//! Device settings example.
//!
//! Reads and displays all current device settings reported by the handshake,
//! then optionally applies and persists new values.
//!
//! Run with:
//! ```sh
//! cargo run --example settings
//! ```
use log::{error, info};
use std::io;
use std::process::ExitCode;
use zedmd_rs::types::RgbOrder;
use zedmd_rs::zedmd::connect;

fn main() -> ExitCode {
    env_logger::Builder::new()
        .format_timestamp(None)
        .format_module_path(false)
        .format_target(false)
        .filter_level(log::LevelFilter::Info)
        .init();

    match configure() {
        Ok(_) => ExitCode::SUCCESS,
        Err(e) => {
            error!("Error: {}", e);
            ExitCode::FAILURE
        }
    }
}

fn print_settings(comm: &zedmd_rs::zedmd::ZeDMDComm) {
    info!("  usb_package_size    : {}", comm.write_at_once());
    info!("  min_refresh_rate    : {}", comm.panel_min_refresh_rate());
    info!("  brightness          : {}", comm.brightness());
    info!(
        "  rgb_order           : {} ({})",
        comm.rgb_order(),
        comm.rgb_order() as u8
    );
    info!("  y_offset            : {}", comm.y_offset());
    info!("  panel_clock_phase   : {}", comm.panel_clock_phase());
    info!("  panel_i2s_speed     : {}", comm.panel_i2s_speed());
    info!("  panel_latch_blanking: {}", comm.panel_latch_blanking());
    info!("  panel_driver        : {}", comm.panel_driver());
}

fn print_usage() {
    eprintln!("Usage: settings [OPTIONS]");
    eprintln!();
    eprintln!("Options:");
    eprintln!("  --usb-package-size <32..1920>   USB packet size (multiple of 32)");
    eprintln!("  --min-refresh-rate <30..255>    Panel minimum refresh rate (fps)");
    eprintln!("  --brightness <0..255>           Display brightness");
    eprintln!("  --rgb-order <RGB|RBG|GRB|GBR|BRG|BGR>  RGB channel order");
    eprintln!("  --y-offset <n>                  Y offset");
    eprintln!("  --panel-clock-phase <n>         Panel clock phase");
    eprintln!("  --panel-i2s-speed <8..>         Panel I2S speed");
    eprintln!("  --panel-latch-blanking <n>      Panel latch blanking");
    eprintln!("  --panel-driver <n>              Panel driver");
    eprintln!("  --debug                         Enable debug output");
    eprintln!("  --no-debug                      Disable debug output");
    eprintln!("  --reset                         Reset all settings to factory defaults");
    eprintln!();
    eprintln!("All changes are saved to device flash automatically.");
}

fn configure() -> io::Result<()> {
    let mut comm = connect()?;
    info!("Connected to ZeDMD device");
    info!("Current settings:");
    print_settings(&comm);

    let args: Vec<String> = std::env::args().collect();
    let mut usb_package_size: Option<u16> = None;
    let mut min_refresh_rate: Option<u8> = None;
    let mut brightness: Option<u8> = None;
    let mut rgb_order: Option<RgbOrder> = None;
    let mut y_offset: Option<u8> = None;
    let mut panel_clock_phase: Option<u8> = None;
    let mut panel_i2s_speed: Option<u8> = None;
    let mut panel_latch_blanking: Option<u8> = None;
    let mut panel_driver: Option<u8> = None;
    let mut reset = false;
    let mut debug: Option<bool> = None;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--usb-package-size" => {
                i += 1;
                usb_package_size = Some(parse_u16(&args[i])?);
            }
            "--min-refresh-rate" => {
                i += 1;
                min_refresh_rate = Some(parse_u8(&args[i])?);
            }
            "--brightness" => {
                i += 1;
                brightness = Some(parse_u8(&args[i])?);
            }
            "--rgb-order" => {
                i += 1;
                rgb_order = Some(parse_rgb_order(&args[i])?);
            }
            "--y-offset" => {
                i += 1;
                y_offset = Some(parse_u8(&args[i])?);
            }
            "--panel-clock-phase" => {
                i += 1;
                panel_clock_phase = Some(parse_u8(&args[i])?);
            }
            "--panel-i2s-speed" => {
                i += 1;
                panel_i2s_speed = Some(parse_u8(&args[i])?);
            }
            "--panel-latch-blanking" => {
                i += 1;
                panel_latch_blanking = Some(parse_u8(&args[i])?);
            }
            "--panel-driver" => {
                i += 1;
                panel_driver = Some(parse_u8(&args[i])?);
            }
            "--debug" => {
                debug = Some(true);
            }
            "--no-debug" => {
                debug = Some(false);
            }
            "--reset" => {
                reset = true;
            }
            "--help" | "-h" => {
                print_usage();
                return Ok(());
            }
            other => {
                eprintln!("Unknown argument: {}", other);
                print_usage();
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("Unknown argument: {}", other),
                ));
            }
        }
        i += 1;
    }

    if reset {
        info!("Resetting to factory defaults");
        usb_package_size = Some(64);
        min_refresh_rate = Some(30);
        // brightness, rgb_order etc. are left as-is on reset (device handles it)
    }

    let any_change = usb_package_size.is_some()
        || min_refresh_rate.is_some()
        || brightness.is_some()
        || rgb_order.is_some()
        || y_offset.is_some()
        || panel_clock_phase.is_some()
        || panel_i2s_speed.is_some()
        || panel_latch_blanking.is_some()
        || panel_driver.is_some()
        || debug.is_some();

    if !any_change {
        info!("No changes requested. Use --help to see available options.");
        return Ok(());
    }

    if let Some(v) = usb_package_size {
        comm.set_usb_package_size(v)?;
    }
    if let Some(v) = min_refresh_rate {
        comm.set_min_refresh_rate(v)?;
    }
    if let Some(v) = brightness {
        comm.set_brightness(v)?;
    }
    if let Some(v) = rgb_order {
        comm.set_rgb_order(v)?;
    }
    if let Some(v) = y_offset {
        comm.set_y_offset(v)?;
    }
    if let Some(v) = panel_clock_phase {
        comm.set_panel_clock_phase(v)?;
    }
    if let Some(v) = panel_i2s_speed {
        comm.set_panel_i2s_speed(v)?;
    }
    if let Some(v) = panel_latch_blanking {
        comm.set_panel_latch_blanking(v)?;
    }
    if let Some(v) = panel_driver {
        comm.set_panel_driver(v)?;
    }
    match debug {
        Some(true) => {
            comm.enable_debug()?;
        }
        Some(false) => {
            comm.disable_debug()?;
        }
        None => {}
    }

    comm.save_settings()?;
    info!("Settings saved, reconnecting to verify...");
    comm.reconnect()?;

    info!("Verified settings:");
    print_settings(&comm);

    Ok(())
}

fn parse_u8(s: &str) -> io::Result<u8> {
    s.parse()
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, format!("Invalid value: {}", s)))
}

fn parse_u16(s: &str) -> io::Result<u16> {
    s.parse()
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, format!("Invalid value: {}", s)))
}

fn parse_rgb_order(s: &str) -> io::Result<RgbOrder> {
    match s.to_uppercase().as_str() {
        "RGB" => Ok(RgbOrder::Rgb),
        "BRG" => Ok(RgbOrder::Brg),
        "GBR" => Ok(RgbOrder::Gbr),
        "RBG" => Ok(RgbOrder::Rbg),
        "GRB" => Ok(RgbOrder::Grb),
        "BGR" => Ok(RgbOrder::Bgr),
        _ => Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!(
                "Invalid RGB order '{}'. Valid values: RGB, BRG, GBR, RBG, GRB, BGR",
                s
            ),
        )),
    }
}
