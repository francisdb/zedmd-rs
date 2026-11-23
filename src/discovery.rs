/// Finding ZeDMD devices without opening them: USB serial ports with a
/// ZeDMD bridge id, and WiFi devices answering the HTTP handshake, at their
/// mDNS name or anywhere on the local network.
///
/// Meant for setup screens ("find my ZeDMD") and for writing the address
/// into another program's settings; [`crate::connect`] and
/// [`crate::connect_wifi`] remain the way to talk to the device.
use crate::comm::ZEDMD_USB_IDS;
use crate::wifi_handshake::fetch_handshake_with_timeout;
use log::debug;
use serialport::SerialPortType::UsbPort;
use std::io;
use std::net::{IpAddr, Ipv4Addr, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, mpsc};
use std::thread;
use std::time::Duration;

/// The mDNS name a ZeDMD announces on the network.
pub const DEFAULT_WIFI_HOST: &str = "zedmd-wifi.local";

/// How many addresses [`discover_wifi`] probes at the same time.
const SCAN_THREADS: usize = 32;

/// A ZeDMD that answered the WiFi handshake.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WifiDevice {
    /// The name or address it was reached at.
    pub host: String,
    /// The address `host` resolved to.
    pub ip: IpAddr,
    pub width: u32,
    pub height: u32,
    pub firmware_version: String,
    /// Streaming transport the device is set to: TCP, or UDP when false.
    pub tcp: bool,
    /// Streaming port.
    pub port: u16,
}

/// Serial ports whose USB vendor/product id is one of the bridges used on
/// ZeDMD boards. Nothing is written to them, so a match is a candidate,
/// not a confirmed ZeDMD.
pub fn usb_ports() -> Vec<String> {
    let Ok(ports) = serialport::available_ports() else {
        return Vec::new();
    };
    ports
        .into_iter()
        .filter(|p| matches!(&p.port_type, UsbPort(info) if ZEDMD_USB_IDS.contains(&(info.vid, info.pid))))
        .map(|p| p.port_name)
        .collect()
}

/// Ask `host` (a name or an IP address) for the ZeDMD handshake, waiting at
/// most `timeout` for the connection and for each read.
pub fn probe_wifi(host: &str, timeout: Duration) -> io::Result<WifiDevice> {
    probe_wifi_at(host, 80, timeout)
}

fn probe_wifi_at(host: &str, http_port: u16, timeout: Duration) -> io::Result<WifiDevice> {
    let (h, ip) = fetch_handshake_with_timeout(host, http_port, timeout)?;
    Ok(WifiDevice {
        host: host.to_string(),
        ip,
        width: h.width,
        height: h.height,
        firmware_version: h.firmware_version,
        tcp: h.tcp,
        port: h.port,
    })
}

/// Find a WiFi ZeDMD: first at [`DEFAULT_WIFI_HOST`], then at every address
/// of this machine's local network (its /24, private ranges only), probing
/// [`SCAN_THREADS`] addresses at a time with `timeout` each. Returns the
/// first device that answers. With a 300 ms timeout a full scan takes a few
/// seconds; call it off the UI thread.
pub fn discover_wifi(timeout: Duration) -> Option<WifiDevice> {
    // mDNS resolution can be slow, give the name a little more time
    match probe_wifi(DEFAULT_WIFI_HOST, timeout.max(Duration::from_secs(1))) {
        Ok(device) => return Some(device),
        Err(e) => debug!("{} did not answer: {}", DEFAULT_WIFI_HOST, e),
    }
    let me = local_ipv4()?;
    let [a, b, c, mine] = me.octets();
    let hosts: Vec<Ipv4Addr> = (1..=254u8)
        .filter(|&d| d != mine)
        .map(|d| Ipv4Addr::new(a, b, c, d))
        .collect();
    debug!("Scanning {}.{}.{}.0/24 for a ZeDMD", a, b, c);
    scan(hosts, 80, timeout)
}

fn scan(hosts: Vec<Ipv4Addr>, http_port: u16, timeout: Duration) -> Option<WifiDevice> {
    let hosts = Arc::new(hosts);
    let next = Arc::new(AtomicUsize::new(0));
    let found = Arc::new(AtomicBool::new(false));
    let (tx, rx) = mpsc::channel();
    let workers: Vec<_> = (0..SCAN_THREADS)
        .filter_map(|i| {
            let (hosts, next, found, tx) = (hosts.clone(), next.clone(), found.clone(), tx.clone());
            thread::Builder::new()
                .name(format!("zedmd-scan-{}", i))
                .spawn(move || {
                    while !found.load(Ordering::Relaxed) {
                        let Some(ip) = hosts.get(next.fetch_add(1, Ordering::Relaxed)) else {
                            break;
                        };
                        if let Ok(device) = probe_wifi_at(&ip.to_string(), http_port, timeout) {
                            found.store(true, Ordering::Relaxed);
                            let _ = tx.send(device);
                        }
                    }
                })
                .ok()
        })
        .collect();
    drop(tx);
    let device = rx.recv().ok();
    found.store(true, Ordering::Relaxed);
    for w in workers {
        let _ = w.join();
    }
    device
}

/// This machine's IPv4 address on a private network, if it has one.
fn local_ipv4() -> Option<Ipv4Addr> {
    // Connecting a UDP socket sends nothing, it only selects the route and
    // therefore the local address. 192.0.2.1 is a documentation address.
    let socket = UdpSocket::bind("0.0.0.0:0").ok()?;
    socket.connect("192.0.2.1:9").ok()?;
    match socket.local_addr().ok()?.ip() {
        IpAddr::V4(ip) if ip.is_private() => Some(ip),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Read, Write};
    use std::net::TcpListener;

    /// A one-shot HTTP server on 127.0.0.1 answering with `body`.
    fn serve(status: &'static str, body: &'static str) -> u16 {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        thread::spawn(move || {
            if let Ok((mut s, _)) = listener.accept() {
                let mut buf = [0u8; 512];
                let _ = s.read(&mut buf);
                let _ = write!(
                    s,
                    "HTTP/1.1 {}\r\nConnection: close\r\n\r\n{}",
                    status, body
                );
            }
        });
        port
    }

    const HANDSHAKE: &str = "128|32|5.1.8|0|UDP|3333|5|256|7|3|0|0|8|2|30|0|test-ssid|0|63531";

    #[test]
    fn probe_reads_a_zedmd_handshake() {
        let port = serve("200 OK", HANDSHAKE);
        let d = probe_wifi_at("127.0.0.1", port, Duration::from_secs(2)).unwrap();
        assert_eq!((d.width, d.height, d.port, d.tcp), (128, 32, 3333, false));
        assert_eq!(d.firmware_version, "5.1.8");
        assert_eq!(d.ip, IpAddr::V4(Ipv4Addr::LOCALHOST));
    }

    #[test]
    fn probe_rejects_other_web_servers() {
        let port = serve("200 OK", "<html>router login</html>");
        assert!(probe_wifi_at("127.0.0.1", port, Duration::from_secs(2)).is_err());
        let port = serve("404 Not Found", HANDSHAKE);
        assert!(probe_wifi_at("127.0.0.1", port, Duration::from_secs(2)).is_err());
    }

    #[test]
    fn scan_returns_the_device_that_answers() {
        let port = serve("200 OK", HANDSHAKE);
        // 127.0.0.2 has nothing listening on that port (Linux routes all of
        // 127/8 to loopback; elsewhere it fails to connect, same result)
        let hosts = vec![Ipv4Addr::new(127, 0, 0, 2), Ipv4Addr::LOCALHOST];
        let d = scan(hosts, port, Duration::from_millis(500)).unwrap();
        assert_eq!(d.host, "127.0.0.1");
    }
}
