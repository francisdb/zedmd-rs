/// Transport abstraction over USB serial and WiFi UDP.
///
/// Higher-level code (`SharedZeDMDComm`) builds the same command payload — a
/// `FRAME` header followed by one or more `ZeDMD`-prefixed commands — and hands
/// it to the transport. The transport is responsible for getting those bytes to
/// the firmware:
///
/// - **USB** chunks the payload at `write_at_once` boundaries, zero-pads the
///   tail of each chunk, and waits for a 6-byte `ZeDMDA` ACK per chunk.
/// - **WiFi UDP** sends the payload as one or more 1400-byte UDP datagrams with
///   `udp_delay` ms between them, and never reads back. The firmware's
///   `HandleData()` scans for the inner `ZeDMD` ctrl-chars marker, so it
///   tolerates the leading `FRAME` bytes (they don't match and are discarded).
use log::{debug, error};
use serialport::SerialPort;
use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::thread::sleep;
use std::time::Duration;

pub(crate) const ZEDMD_COMM_MIN_SERIAL_WRITE_AT_ONCE: usize = 32;
pub(crate) const ZEDMD_WIFI_UDP_CHUNK_SIZE: usize = 1400;

const CTRL_CHARS_HEADER: [u8; 5] = [0x5A, 0x65, 0x44, 0x4D, 0x44];

pub(crate) enum Transport {
    Usb(UsbTransport),
    WifiUdp(WifiUdpTransport),
}

pub(crate) struct UsbTransport {
    pub port: Option<Box<dyn SerialPort>>,
    pub write_at_once: usize,
}

pub(crate) struct WifiUdpTransport {
    pub socket: UdpSocket,
    pub target: SocketAddr,
    pub udp_delay: Duration,
}

impl std::fmt::Debug for Transport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Transport::Usb(u) => f
                .debug_struct("UsbTransport")
                .field("connected", &u.port.is_some())
                .field("write_at_once", &u.write_at_once)
                .finish(),
            Transport::WifiUdp(w) => f
                .debug_struct("WifiUdpTransport")
                .field("target", &w.target)
                .field("udp_delay", &w.udp_delay)
                .finish(),
        }
    }
}

impl Transport {
    /// Send a fully-built command payload starting with the `FRAME` header.
    pub fn send(&mut self, data: &[u8]) -> io::Result<()> {
        match self {
            Transport::Usb(t) => t.send(data),
            Transport::WifiUdp(t) => t.send(data),
        }
    }

    pub fn flush(&mut self) -> io::Result<()> {
        match self {
            Transport::Usb(t) => match &mut t.port {
                Some(port) => port.flush(),
                None => Ok(()),
            },
            Transport::WifiUdp(_) => Ok(()),
        }
    }

    pub fn is_usb(&self) -> bool {
        matches!(self, Transport::Usb(_))
    }

    /// USB packet size in bytes (or `0` for WiFi, where it is not applicable).
    pub fn write_at_once(&self) -> usize {
        match self {
            Transport::Usb(t) => t.write_at_once,
            Transport::WifiUdp(_) => 0,
        }
    }

    /// Update the USB packet size. No-op on WiFi.
    pub fn set_write_at_once(&mut self, size: usize) {
        if let Transport::Usb(t) = self {
            t.write_at_once = size;
        }
    }
}

impl UsbTransport {
    fn send(&mut self, data: &[u8]) -> io::Result<()> {
        let port = self
            .port
            .as_mut()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "No port connected"))?;
        send_chunks_usb(port, data, self.write_at_once)
    }
}

impl WifiUdpTransport {
    fn send(&mut self, data: &[u8]) -> io::Result<()> {
        let mut sent = 0;
        let total = data.len();
        while sent < total {
            let chunk_len = ZEDMD_WIFI_UDP_CHUNK_SIZE.min(total - sent);
            let n = self
                .socket
                .send_to(&data[sent..sent + chunk_len], self.target)?;
            if n < chunk_len {
                return Err(io::Error::other(format!(
                    "UDP short send: wrote {} of {} bytes",
                    n, chunk_len
                )));
            }
            sent += n;
            // The ESP32's AsyncUDP can crash if datagrams arrive too fast;
            // libzedmd uses the same per-datagram delay.
            if sent < total {
                sleep(self.udp_delay);
            }
        }
        debug!(
            "WiFi UDP sent {} bytes in {} datagram(s)",
            total,
            total.div_ceil(ZEDMD_WIFI_UDP_CHUNK_SIZE)
        );
        Ok(())
    }
}

/// USB chunked send with per-chunk ACK, mirrors the firmware's USB transport.
pub(crate) fn send_chunks_usb(
    port: &mut Box<dyn SerialPort>,
    data: &[u8],
    write_at_once: usize,
) -> io::Result<()> {
    let write_at_once = write_at_once.max(ZEDMD_COMM_MIN_SERIAL_WRITE_AT_ONCE);
    let mut sent = 0;
    let total = data.len();

    while sent < total {
        let chunk_len = write_at_once.min(total - sent);
        let mut packet = vec![0u8; write_at_once];
        packet[..chunk_len].copy_from_slice(&data[sent..sent + chunk_len]);
        sent += chunk_len;

        port.write_all(&packet)?;

        // Read the 6-byte ACK with retries to handle partial reads.
        let mut ack = [0u8; 6];
        let mut ack_read = 0;
        while ack_read < 6 {
            match port.read(&mut ack[ack_read..]) {
                Ok(0) => {
                    error!("ACK read returned 0 bytes (sent {}/{} bytes)", sent, total);
                    return Err(io::Error::new(
                        io::ErrorKind::UnexpectedEof,
                        "ACK read returned 0 bytes",
                    ));
                }
                Ok(n) => ack_read += n,
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {
                    if ack_read == 0 {
                        error!(
                            "ACK timeout after sending packet (sent {}/{} bytes)",
                            sent, total
                        );
                        return Err(io::Error::new(io::ErrorKind::TimedOut, "ACK timeout"));
                    }
                }
                Err(e) => {
                    error!("ACK read error: {}", e);
                    return Err(e);
                }
            }
        }
        if ack[..5] != CTRL_CHARS_HEADER {
            error!("Bad ACK header: {:02x?}", &ack[..6]);
            return Err(io::Error::new(io::ErrorKind::InvalidData, "Bad ACK header"));
        }
        match ack[5] {
            b'A' => {}
            b'F' => {
                error!("Device sent NACK ('F')");
                return Err(io::Error::other("Device NACK"));
            }
            other => {
                error!("Unexpected ACK byte: 0x{:02x} ({:?})", other, other as char);
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("Unexpected ACK byte 0x{:02x}", other),
                ));
            }
        }
    }
    Ok(())
}
