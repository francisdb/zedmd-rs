//! Look for ZeDMD devices without opening them: USB serial ports with a
//! ZeDMD bridge id, then a WiFi ZeDMD at zedmd-wifi.local or on the local
//! network.
//!
//! ```bash
//! cargo run --example discover
//! ```
use std::time::Duration;
use zedmd::discovery;

fn main() {
    env_logger::init();

    let ports = discovery::usb_ports();
    if ports.is_empty() {
        println!("No USB serial port with a ZeDMD bridge id");
    }
    for port in ports {
        println!("USB candidate: {}", port);
    }

    println!("Looking for a WiFi ZeDMD...");
    match discovery::discover_wifi(Duration::from_millis(300)) {
        Some(d) => println!(
            "WiFi ZeDMD at {} ({}): {}x{}, firmware {}, streaming over {} port {}",
            d.host,
            d.ip,
            d.width,
            d.height,
            d.firmware_version,
            if d.tcp { "TCP" } else { "UDP" },
            d.port
        ),
        None => println!("No WiFi ZeDMD found"),
    }
}
