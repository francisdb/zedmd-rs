/// WiFi handshake — `GET /handshake` over HTTP/1.1, parses the pipe-separated
/// reply the firmware (`wifi_transport.cpp`) sends back. Field order:
///
/// `width|height|version|s3|protocol|port|udp_delay|write_at_once|brightness|
///  rgbMode|panelClkphase|panelDriver|panelI2sspeed|panelLatchBlanking|
///  panelMinRefreshRate|yOffset|ssid|half|id|wifiPower|deviceType`
///
/// The `protocol` field is the literal string `"UDP"` or `"TCP"`. The trailing
/// `wifiPower` and `deviceType` fields are optional on older firmware.
use log::{debug, info};
use std::io::{self, Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

const HTTP_TIMEOUT: Duration = Duration::from_secs(5);

#[derive(Debug, Clone)]
pub(crate) struct WifiHandshake {
    pub width: u32,
    pub height: u32,
    pub firmware_version: String,
    pub s3: bool,
    pub tcp: bool,
    pub port: u16,
    pub udp_delay: u8,
    /// Negotiated USB packet size — present in the WiFi handshake but unused
    /// by the WiFi transport (kept so the handshake parser stays a 1:1 map).
    #[allow(dead_code)]
    pub write_at_once: usize,
    pub brightness: u8,
    pub rgb_mode: u8,
    pub panel_clkphase: u8,
    pub panel_driver: u8,
    pub panel_i2sspeed: u8,
    pub panel_latch_blanking: u8,
    pub panel_min_refresh_rate: u8,
    pub y_offset: u8,
    pub ssid: String,
    pub half: bool,
    pub id: u16,
}

pub(crate) fn fetch_handshake(host: &str) -> io::Result<WifiHandshake> {
    let addrs: Vec<_> = (host, 80).to_socket_addrs()?.collect();
    let addr = addrs.into_iter().next().ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::NotFound,
            format!("Could not resolve {}", host),
        )
    })?;
    info!(
        "Fetching ZeDMD handshake from http://{}/handshake (resolved {})",
        host, addr
    );

    let mut stream = TcpStream::connect_timeout(&addr, HTTP_TIMEOUT)?;
    stream.set_read_timeout(Some(HTTP_TIMEOUT))?;
    stream.set_write_timeout(Some(HTTP_TIMEOUT))?;

    let request = format!(
        "GET /handshake HTTP/1.1\r\nHost: {}\r\nConnection: close\r\nUser-Agent: zedmd-rs\r\n\r\n",
        host
    );
    stream.write_all(request.as_bytes())?;
    stream.flush()?;

    let mut response = Vec::with_capacity(512);
    stream.read_to_end(&mut response)?;

    let body = extract_body(&response)?;
    debug!("Handshake body: {}", body);
    parse(body)
}

fn extract_body(response: &[u8]) -> io::Result<&str> {
    // Find end-of-headers (\r\n\r\n)
    let sep = response
        .windows(4)
        .position(|w| w == b"\r\n\r\n")
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "No HTTP header terminator"))?;
    let head = std::str::from_utf8(&response[..sep])
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
    let status_line = head.lines().next().unwrap_or("");
    if !status_line.contains("200") {
        return Err(io::Error::other(format!(
            "Unexpected HTTP status: {}",
            status_line
        )));
    }
    let body = &response[sep + 4..];
    std::str::from_utf8(body).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))
}

fn parse(body: &str) -> io::Result<WifiHandshake> {
    let parts: Vec<&str> = body.trim_end().split('|').collect();
    let invalid =
        |msg: String| io::Error::new(io::ErrorKind::InvalidData, format!("Handshake: {}", msg));

    if parts.len() < 17 {
        return Err(invalid(format!(
            "expected at least 17 fields, got {} ({:?})",
            parts.len(),
            parts
        )));
    }
    let parse_u32 = |i: usize| -> io::Result<u32> {
        parts[i]
            .parse()
            .map_err(|e| invalid(format!("field {}: {}", i, e)))
    };
    let parse_u16 = |i: usize| -> io::Result<u16> {
        parts[i]
            .parse()
            .map_err(|e| invalid(format!("field {}: {}", i, e)))
    };
    let parse_u8 = |i: usize| -> io::Result<u8> {
        parts[i]
            .parse()
            .map_err(|e| invalid(format!("field {}: {}", i, e)))
    };
    let parse_usize = |i: usize| -> io::Result<usize> {
        parts[i]
            .parse()
            .map_err(|e| invalid(format!("field {}: {}", i, e)))
    };

    Ok(WifiHandshake {
        width: parse_u32(0)?,
        height: parse_u32(1)?,
        firmware_version: parts[2].to_string(),
        s3: parts[3] == "1",
        tcp: parts[4].eq_ignore_ascii_case("TCP"),
        port: parse_u16(5)?,
        udp_delay: parse_u8(6)?,
        write_at_once: parse_usize(7)?,
        brightness: parse_u8(8)?,
        rgb_mode: parse_u8(9)?,
        panel_clkphase: parse_u8(10)?,
        panel_driver: parse_u8(11)?,
        panel_i2sspeed: parse_u8(12)?,
        panel_latch_blanking: parse_u8(13)?,
        panel_min_refresh_rate: parse_u8(14)?,
        y_offset: parse_u8(15)?,
        ssid: parts[16].to_string(),
        half: parts.get(17).is_some_and(|s| *s == "1"),
        id: parts.get(18).and_then(|s| s.parse().ok()).unwrap_or(0),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_real_device_response() {
        let body = "128|32|5.1.8|0|UDP|3333|5|256|7|3|0|0|8|2|30|0|test-ssid|0|63531";
        let h = parse(body).unwrap();
        assert_eq!(h.width, 128);
        assert_eq!(h.height, 32);
        assert_eq!(h.firmware_version, "5.1.8");
        assert!(!h.s3);
        assert!(!h.tcp);
        assert_eq!(h.port, 3333);
        assert_eq!(h.udp_delay, 5);
        assert_eq!(h.write_at_once, 256);
        assert_eq!(h.brightness, 7);
        assert_eq!(h.rgb_mode, 3);
        assert_eq!(h.panel_clkphase, 0);
        assert_eq!(h.panel_driver, 0);
        assert_eq!(h.panel_i2sspeed, 8);
        assert_eq!(h.panel_latch_blanking, 2);
        assert_eq!(h.panel_min_refresh_rate, 30);
        assert_eq!(h.y_offset, 0);
        assert_eq!(h.ssid, "test-ssid");
        assert!(!h.half);
        assert_eq!(h.id, 63531);
    }
}
