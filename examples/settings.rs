//! Device settings example.
//!
//! Reads and displays current settings reported by the handshake, then
//! optionally applies and persists new values.
//!
//! Run with:
//! ```sh
//! cargo run --example settings
//! ```
use log::{error, info, warn};
use std::io;
use std::process::ExitCode;
use zedmd::connect;
use zedmd::types::{RgbOrder, TransportMode};

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

fn print_settings(comm: &zedmd::ZeDMDComm) {
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
    eprintln!("Display/panel options:");
    eprintln!("  --usb-package-size <32..1920>   USB packet size (multiple of 32)");
    eprintln!("  --min-refresh-rate <30..255>    Panel minimum refresh rate");
    eprintln!("  --brightness <0..255>           Display brightness");
    eprintln!("  --rgb-order <RGB|RBG|GRB|GBR|BRG|BGR>  RGB channel order");
    eprintln!("  --y-offset <n>                  Y offset");
    eprintln!("  --panel-clock-phase <n>         Panel clock phase");
    eprintln!("  --panel-i2s-speed <8..>         Panel I2S speed");
    eprintln!("  --panel-latch-blanking <n>      Panel latch blanking");
    eprintln!("  --panel-driver <n>              Panel driver");
    eprintln!("  --debug                         Enable debug output");
    eprintln!("  --no-debug                      Disable debug output");
    eprintln!("  --reset                         Reset key settings to defaults");
    eprintln!();
    eprintln!("WiFi/transport options:");
    eprintln!("  --ssid <name>                   WiFi SSID (max 31 chars)");
    eprintln!("  --password <password>           WiFi password (max 63 chars)");
    eprintln!("  --port <1..65535>               WiFi UDP/TCP port");
    eprintln!("  --power <0..127>                WiFi transmit power");
    eprintln!("  --udp-delay <0..9>              WiFi UDP inter-packet delay");
    eprintln!("  --transport <usb|wifi-udp|wifi-tcp|spi>");
    eprintln!();
    eprintln!("Persistence:");
    eprintln!("  --save                          Persist changes to device flash.");
    eprintln!("                                  Without this flag changes apply to the");
    eprintln!("                                  current session only and are lost on reboot.");
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

    let mut ssid: Option<String> = None;
    let mut password: Option<String> = None;
    let mut wifi_port: Option<u16> = None;
    let mut wifi_power: Option<u8> = None;
    let mut udp_delay: Option<u8> = None;
    let mut transport_mode: Option<TransportMode> = None;
    let mut save = false;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--usb-package-size" => {
                i += 1;
                usb_package_size = Some(parse_u16(next_arg(&args, i, "--usb-package-size")?)?);
            }
            "--min-refresh-rate" => {
                i += 1;
                min_refresh_rate = Some(parse_u8(next_arg(&args, i, "--min-refresh-rate")?)?);
            }
            "--brightness" => {
                i += 1;
                brightness = Some(parse_u8(next_arg(&args, i, "--brightness")?)?);
            }
            "--rgb-order" => {
                i += 1;
                rgb_order = Some(parse_rgb_order(next_arg(&args, i, "--rgb-order")?)?);
            }
            "--y-offset" => {
                i += 1;
                y_offset = Some(parse_u8(next_arg(&args, i, "--y-offset")?)?);
            }
            "--panel-clock-phase" => {
                i += 1;
                panel_clock_phase = Some(parse_u8(next_arg(&args, i, "--panel-clock-phase")?)?);
            }
            "--panel-i2s-speed" => {
                i += 1;
                panel_i2s_speed = Some(parse_u8(next_arg(&args, i, "--panel-i2s-speed")?)?);
            }
            "--panel-latch-blanking" => {
                i += 1;
                panel_latch_blanking =
                    Some(parse_u8(next_arg(&args, i, "--panel-latch-blanking")?)?);
            }
            "--panel-driver" => {
                i += 1;
                panel_driver = Some(parse_u8(next_arg(&args, i, "--panel-driver")?)?);
            }
            "--ssid" => {
                i += 1;
                ssid = Some(next_arg(&args, i, "--ssid")?.to_owned());
            }
            "--password" => {
                i += 1;
                password = Some(next_arg(&args, i, "--password")?.to_owned());
            }
            "--port" => {
                i += 1;
                wifi_port = Some(parse_u16(next_arg(&args, i, "--port")?)?);
            }
            "--power" => {
                i += 1;
                wifi_power = Some(parse_u8(next_arg(&args, i, "--power")?)?);
            }
            "--udp-delay" => {
                i += 1;
                udp_delay = Some(parse_u8(next_arg(&args, i, "--udp-delay")?)?);
            }
            "--transport" => {
                i += 1;
                transport_mode = Some(parse_transport(next_arg(&args, i, "--transport")?)?);
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
            "--save" => {
                save = true;
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
        || debug.is_some()
        || ssid.is_some()
        || password.is_some()
        || wifi_port.is_some()
        || wifi_power.is_some()
        || udp_delay.is_some()
        || transport_mode.is_some();

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

    if let Some(v) = ssid {
        comm.set_wifi_ssid(&v)?;
    }
    if let Some(v) = password {
        comm.set_wifi_password(&v)?;
    }
    if let Some(v) = wifi_port {
        comm.set_wifi_port(v)?;
    }
    if let Some(v) = wifi_power {
        comm.set_wifi_power(v)?;
    }
    if let Some(v) = udp_delay {
        comm.set_udp_delay(v)?;
    }
    if let Some(v) = transport_mode {
        comm.set_transport_mode(v)?;
    }

    if save {
        comm.save_settings()?;

        if let Some(mode) = transport_mode
            && mode != TransportMode::Usb
        {
            warn!("Transport set to {}.", mode);
            warn!(
                "USB is expected to be unavailable after reboot until transport is switched back to USB via the device buttons/menu."
            );
            warn!(
                "If needed, use the firmware web interface or a factory reset path supported by your firmware build."
            );
            return Ok(());
        }

        info!("Settings saved, reconnecting to verify...");
        comm.reconnect()?;
        info!("Verified settings:");
        print_settings(&comm);
    } else {
        info!("Changes applied for this session only. Use --save to persist to device flash.");
    }

    Ok(())
}

fn next_arg<'a>(args: &'a [String], idx: usize, flag: &str) -> io::Result<&'a str> {
    args.get(idx).map(String::as_str).ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("Missing value for {}", flag),
        )
    })
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

fn parse_transport(s: &str) -> io::Result<TransportMode> {
    match s.to_ascii_lowercase().as_str() {
        "usb" => Ok(TransportMode::Usb),
        "wifi-udp" | "udp" => Ok(TransportMode::WifiUdp),
        "wifi-tcp" | "tcp" => Ok(TransportMode::WifiTcp),
        "spi" => Ok(TransportMode::Spi),
        _ => Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!(
                "Invalid transport '{}'. Valid values: usb, wifi-udp, wifi-tcp, spi",
                s
            ),
        )),
    }
}
