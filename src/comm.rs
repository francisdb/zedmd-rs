use log::{debug, error, info, warn};
use serialport::SerialPortType::UsbPort;
use serialport::{DataBits, Parity, SerialPort, StopBits};
/// Serial transport layer — mirrors ZeDMDComm.cpp/.h from libzedmd.
/// Handles connection, handshake, chunked writes and ACK protocol.
use std::io::{self, Write};
use std::thread::sleep;
use std::time::{Duration, Instant};

use crate::types::ZedmdCommCommand;
use miniz_oxide::deflate::compress_to_vec_zlib;

// The maximum baud rate supported by CP210x.
pub(crate) const ZEDMD_COMM_BAUD_RATE: u32 = 921600;
pub(crate) const ZEDMD_COMM_MIN_SERIAL_WRITE_AT_ONCE: usize = 32;
pub(crate) const ZEDMD_COMM_MAX_SERIAL_WRITE_AT_ONCE: usize = 1920;
pub(crate) const ZEDMD_COMM_DEFAULT_SERIAL_WRITE_AT_ONCE: usize = 64;
pub(crate) const ZEDMD_COMM_KEEP_ALIVE_INTERVAL: u128 = 3000; // ms
// ACK read timeout in ms. The C++ uses 16ms per sp_blocking_read call, but
// sp_blocking_read retries internally until the full buffer is filled or the
// timeout expires. Our read_exact on Linux gives up after one timeout interval,
// so we use a larger value to give the device enough time to respond.
pub(crate) const ZEDMD_COMM_SERIAL_READ_TIMEOUT_MS: u64 = 200;

// "FRAME" in ascii
pub(crate) const FRAME_HEADER: [u8; 5] = [0x46, 0x52, 0x41, 0x4d, 0x45];
// "ZeDMD" in ascii (first 5 of the 6-byte CtrlChars)
pub(crate) const CTRL_CHARS_HEADER: [u8; 5] = [0x5A, 0x65, 0x44, 0x4D, 0x44];

#[derive(Debug)]
pub(crate) struct SharedZeDMDComm {
    pub width: u32,
    pub height: u32,
    pub zone_width: u32,
    pub zone_height: u32,
    pub firmware_version: String,
    pub write_at_once: usize,
    pub brightness: u8,
    pub rgb_mode: u8,
    pub y_offset: u8,
    pub panel_clkphase: u8,
    pub panel_driver: u8,
    pub panel_i2sspeed: u8,
    pub panel_latch_blanking: u8,
    pub panel_min_refresh_rate: u8,
    pub udp_delay: u8,
    pub half: bool,
    pub s3: bool,
    pub id: u16,
    pub device_name: String,
    pub port: Option<Box<dyn SerialPort>>,
    pub keep_alive: bool,
    pub last_keep_alive: Instant,
    /// Hashes of the last-sent zone contents, used to skip unchanged zones.
    /// 0 = unknown / never sent (force send). 1 = known black.
    pub zone_hashes: Vec<u64>,
}

impl SharedZeDMDComm {
    /// Send a simple command (no payload) using the full framing protocol.
    pub fn send_simple_command(&mut self, cmd: ZedmdCommCommand) -> io::Result<()> {
        let write_at_once = self.write_at_once;
        let payload = build_payload_simple(cmd);
        let port = self
            .port
            .as_mut()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "No port connected"))?;
        let result = send_chunks(port, &payload, write_at_once);
        if result.is_ok() {
            self.last_keep_alive = Instant::now();
        }
        result
    }

    /// Send a command with a single byte of payload.
    pub fn send_command_with_byte(&mut self, cmd: ZedmdCommCommand, value: u8) -> io::Result<()> {
        let write_at_once = self.write_at_once;
        let mut payload = Vec::with_capacity(FRAME_HEADER.len() + CTRL_CHARS_HEADER.len() + 5);
        payload.extend_from_slice(&FRAME_HEADER);
        payload.extend_from_slice(&CTRL_CHARS_HEADER);
        payload.push(cmd as u8);
        payload.push(0); // size high byte (payload = 1 byte)
        payload.push(1); // size low byte
        payload.push(0); // compression flag
        payload.push(value);
        let port = self
            .port
            .as_mut()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "No port connected"))?;
        let result = send_chunks(port, &payload, write_at_once);
        if result.is_ok() {
            self.last_keep_alive = Instant::now();
        }
        result
    }

    /// Render a full RGB565 frame using the zone-streaming protocol.
    /// Only zones whose content has changed since the last call are transmitted.
    /// Zone data is split into sub-frames when it exceeds the device buffer limit,
    /// matching the C++ behaviour (ZEDMD_ZONES_BYTE_LIMIT_RGB565 = 128*4*2+16 = 1040).
    pub fn render_rgb565_frame(&mut self, pixels: &[u16]) -> io::Result<()> {
        // Send keep-alive inline if the device has been idle long enough.
        self.keep_alive_tick()?;

        let width = self.width as usize;
        let height = self.height as usize;
        let zone_width = self.zone_width as usize;
        let zone_height = self.zone_height as usize;
        if width == 0 || height == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Device dimensions not set",
            ));
        }
        if pixels.len() != width * height {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "Expected {} pixels ({}x{}), got {}",
                    width * height,
                    width,
                    height,
                    pixels.len()
                ),
            ));
        }

        let zone_bytes = zone_width * zone_height * 2; // bytes per zone (RGB565)
        let zones_x = width / zone_width;
        let zones_y = height / zone_height;

        let mut all_zone_data: Vec<u8> = Vec::new();
        let mut changed_zones = 0usize;
        let mut idx: u8 = 0;

        let write_at_once = self.write_at_once;
        let port = self
            .port
            .as_mut()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "No port connected"))?;

        for zy in 0..zones_y {
            for zx in 0..zones_x {
                let mut zone_buf = vec![0u8; zone_bytes];
                let mut all_black = true;
                for row in 0..zone_height {
                    for col in 0..zone_width {
                        let px = pixels[(zy * zone_height + row) * width + (zx * zone_width + col)];
                        let off = (row * zone_width + col) * 2;
                        zone_buf[off] = (px & 0xFF) as u8;
                        zone_buf[off + 1] = (px >> 8) as u8;
                        if px != 0 {
                            all_black = false;
                        }
                    }
                }

                let hash: u64 = if all_black { 1 } else { fnv1a_hash(&zone_buf) };
                if hash != self.zone_hashes[idx as usize] {
                    self.zone_hashes[idx as usize] = hash;
                    changed_zones += 1;
                    if all_black {
                        all_zone_data.push(idx.wrapping_add(128));
                    } else {
                        all_zone_data.push(idx);
                        all_zone_data.extend_from_slice(&zone_buf);
                    }
                }
                idx = idx.wrapping_add(1);
            }
        }

        if all_zone_data.is_empty() {
            debug!("No zones changed, skipping frame");
            return Ok(());
        }

        // Match libzedmd: split raw zone data at ZEDMD_ZONES_BYTE_LIMIT_RGB565 = zone_w*zone_h*2*16+16
        // then compress each chunk independently and send in reverse order (last sub-frame first).
        let zones_byte_limit = zone_width * zone_height * 2 * 16 + 16; // 1040 for 8x4 zones
        let zone_bytes_total = zone_bytes + 1;
        let buffer_size_threshold = zones_byte_limit - zone_bytes_total;

        // Split into sub-frames
        let mut sub_frames: Vec<Vec<u8>> = Vec::new();
        let mut pos = 0;
        let mut buf: Vec<u8> = Vec::with_capacity(zones_byte_limit);
        while pos < all_zone_data.len() {
            let entry_len = if all_zone_data[pos] >= 128 {
                1
            } else {
                zone_bytes_total
            };
            let end = (pos + entry_len).min(all_zone_data.len());
            buf.extend_from_slice(&all_zone_data[pos..end]);
            pos = end;
            if buf.len() > buffer_size_threshold {
                sub_frames.push(buf.clone());
                buf.clear();
            }
        }
        if !buf.is_empty() {
            sub_frames.push(buf);
        }

        // Send in reverse order (matching libzedmd's rbegin iteration)
        let n_sub_frames = sub_frames.len();
        for chunk in sub_frames.into_iter() {
            let compressed = compress_to_vec_zlib(&chunk, 6);
            let (data, compressed_flag) = if compressed.len() < chunk.len() {
                (compressed.as_slice(), 1u8)
            } else {
                (chunk.as_slice(), 0u8)
            };
            let data_size = data.len();
            let mut payload: Vec<u8> = Vec::with_capacity(
                FRAME_HEADER.len() + CTRL_CHARS_HEADER.len() * 2 + 8 + data_size,
            );
            payload.extend_from_slice(&FRAME_HEADER);
            payload.extend_from_slice(&CTRL_CHARS_HEADER);
            payload.push(ZedmdCommCommand::RGB565ZonesStream as u8);
            payload.push((data_size >> 8) as u8);
            payload.push((data_size & 0xFF) as u8);
            payload.push(compressed_flag);
            payload.extend_from_slice(data);
            payload.extend_from_slice(&CTRL_CHARS_HEADER);
            payload.push(ZedmdCommCommand::RenderFrame as u8);
            payload.push(0);
            payload.push(0);
            payload.push(0);
            send_chunks(port, &payload, write_at_once)?;
        }

        debug!(
            "Sent RGB565 delta frame: {} changed zones, {} raw bytes, {} sub-frame(s)",
            changed_zones,
            all_zone_data.len(),
            n_sub_frames
        );
        self.last_keep_alive = Instant::now();
        Ok(())
    }

    pub fn keep_alive_tick(&mut self) -> io::Result<()> {
        if !self.keep_alive {
            return Ok(());
        }
        let now = Instant::now();
        if now.duration_since(self.last_keep_alive).as_millis() < ZEDMD_COMM_KEEP_ALIVE_INTERVAL {
            return Ok(());
        }
        self.last_keep_alive = now;
        let payload = build_payload_simple(ZedmdCommCommand::KeepAlive);
        let write_at_once = self.write_at_once;
        let port = self
            .port
            .as_mut()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "No port connected"))?;
        debug!("Sending keep-alive");
        send_chunks(port, &payload, write_at_once)
    }
}

/// Scan all USB serial ports and return a connected `SharedZeDMDComm` for the first ZeDMD found.
pub(crate) fn connect_internal() -> io::Result<SharedZeDMDComm> {
    let ports = serialport::available_ports().expect("No ports found!");
    for p in ports {
        if let UsbPort(info) = p.port_type {
            let port_name = p.port_name.clone();
            debug!(
                "ZeDMD candidate: device={}, vid={:04x}, pid={:04x}",
                port_name, info.vid, info.pid
            );
            let port: Box<dyn SerialPort> = serialport::new(p.port_name, ZEDMD_COMM_BAUD_RATE)
                .parity(Parity::None)
                .data_bits(DataBits::Eight)
                .stop_bits(StopBits::One)
                .flow_control(serialport::FlowControl::None)
                .timeout(Duration::from_millis(ZEDMD_COMM_SERIAL_READ_TIMEOUT_MS))
                .open()
                .expect("Failed to open port");

            if let Some(internal) = try_handshake(port_name, port) {
                return Ok(internal);
            }
        }
    }
    Err(io::Error::new(
        io::ErrorKind::NotFound,
        "No ZeDMD device found",
    ))
}

/// Try the handshake on an already-opened port. Returns `Some(SharedZeDMDComm)` on success.
pub(crate) fn try_handshake(
    port_name: String,
    mut port: Box<dyn SerialPort>,
) -> Option<SharedZeDMDComm> {
    let mut handshake_command = [0u8; ZEDMD_COMM_DEFAULT_SERIAL_WRITE_AT_ONCE];
    handshake_command[..5].copy_from_slice(&FRAME_HEADER);
    handshake_command[5..10].copy_from_slice(&CTRL_CHARS_HEADER);
    handshake_command[10] = ZedmdCommCommand::Handshake as u8;
    log_bytes("Handshake buffer", &handshake_command);

    for attempt in 0..2 {
        debug!("Attempt {} to send handshake command", attempt + 1);
        sleep(Duration::from_millis(200));
        drain_until_silent(&mut port, Duration::from_millis(200));

        log_bytes("Sending handshake", &handshake_command);
        if port.write_all(&handshake_command).is_err() {
            continue;
        }

        let zeroes = [0u8; ZEDMD_COMM_DEFAULT_SERIAL_WRITE_AT_ONCE];
        for _ in 1..(ZEDMD_COMM_MAX_SERIAL_WRITE_AT_ONCE / ZEDMD_COMM_DEFAULT_SERIAL_WRITE_AT_ONCE)
        {
            let _ = port.write_all(&zeroes);
            sleep(Duration::from_millis(10));
        }
        let _ = port.flush();
        sleep(Duration::from_millis(200));

        let _ = port.set_timeout(Duration::from_millis(500));
        let mut data = [0u8; 64];
        let n = match port.read(&mut data) {
            Ok(n) => n,
            Err(e) => {
                debug!(
                    "Failed to read handshake response (attempt {}): {}",
                    attempt + 1,
                    e
                );
                continue;
            }
        };
        debug!(
            "Read {} bytes from handshake response (attempt {})",
            n,
            attempt + 1
        );
        log_bytes("Handshake response", &data[..n]);

        if n < 58
            || !data[..4].starts_with(&CTRL_CHARS_HEADER[..4])
            || data[57] != b'R'
            || data[8] == 0
        {
            debug!(
                "Handshake response check failed (attempt {}, n={}, data[57]={:?}, data[8]={})",
                attempt + 1,
                n,
                data[57] as char,
                data[8]
            );
            continue;
        }

        let width = data[4] as u32 + data[5] as u32 * 256;
        let height = data[6] as u32 + data[7] as u32 * 256;
        let zone_width = if width > 0 { width / 16 } else { 0 };
        let zone_height = if height > 0 { height / 8 } else { 0 };
        let write_at_once_raw = (data[11] as usize) + (data[12] as usize) * 256;
        let write_at_once = if write_at_once_raw == 0 {
            ZEDMD_COMM_DEFAULT_SERIAL_WRITE_AT_ONCE
        } else {
            write_at_once_raw.clamp(
                ZEDMD_COMM_MIN_SERIAL_WRITE_AT_ONCE,
                ZEDMD_COMM_MAX_SERIAL_WRITE_AT_ONCE,
            )
        };

        let mut internal = SharedZeDMDComm {
            width,
            height,
            zone_width,
            zone_height,
            firmware_version: format!("{}.{}.{}", data[8], data[9], data[10]),
            write_at_once,
            brightness: data[13],
            rgb_mode: data[14],
            y_offset: data[15],
            panel_clkphase: data[16],
            panel_driver: data[17],
            panel_i2sspeed: if data[18] < 8 { 8 } else { data[18] },
            panel_latch_blanking: data[19],
            panel_min_refresh_rate: if data[20] < 30 { 30 } else { data[20] },
            udp_delay: data[21],
            half: (data[22] & 0b00000001) != 0,
            s3: (data[22] & 0b00000010) != 0,
            id: data[23] as u16 + data[24] as u16 * 256,
            device_name: port_name,
            port: Some(port),
            keep_alive: false,
            last_keep_alive: Instant::now(),
            zone_hashes: vec![
                1u64;
                ((width / zone_width.max(1)) * (height / zone_height.max(1))) as usize
            ],
        };

        let prefix = if internal.s3 { "S3 " } else { "" };
        info!(
            "ZeDMD {} found: {}device={}, width={}, height={}, write_at_once={}",
            internal.firmware_version,
            prefix,
            internal.device_name,
            internal.width,
            internal.height,
            internal.write_at_once
        );
        if internal.write_at_once <= 64 {
            warn!(
                "The ZeDMD USB package size of {} is very low. Try to increase it for smoother animations.",
                internal.write_at_once
            );
        }
        if internal.panel_min_refresh_rate <= 30 {
            warn!(
                "The ZeDMD panel minimal refresh rate of {} is very low. Try to increase it for smoother animations.",
                internal.panel_min_refresh_rate
            );
        }

        flush_and_discard(internal.port.as_mut().unwrap());
        if let Some(port) = internal.port.as_mut() {
            let _ = port.set_timeout(Duration::from_millis(ZEDMD_COMM_SERIAL_READ_TIMEOUT_MS));
        }
        // Clear the display so the device state matches our zone_hashes (all black).
        let _ = internal.send_simple_command(ZedmdCommCommand::ClearScreen);
        return Some(internal);
    }
    None
}

/// Build a simple control frame payload for a command with no payload data.
pub(crate) fn build_payload_simple(cmd: ZedmdCommCommand) -> Vec<u8> {
    let mut payload = Vec::with_capacity(FRAME_HEADER.len() + CTRL_CHARS_HEADER.len() + 4);
    payload.extend_from_slice(&FRAME_HEADER);
    payload.extend_from_slice(&CTRL_CHARS_HEADER);
    payload.push(cmd as u8);
    payload.push(0); // size high byte
    payload.push(0); // size low byte
    payload.push(0); // compression flag
    payload
}

/// Send `data` to the device using the firmware's USB transport protocol.
///
/// `data` must start with the FRAME header (5 bytes). The firmware scans for
/// FRAME byte-by-byte, then reads the remainder of the `write_at_once`-byte
/// packet. The data is split at `write_at_once` boundaries; each chunk is
/// zero-padded. Device sends one ACK (`ZeDMDA`, 6 bytes) per chunk.
pub(crate) fn send_chunks(
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

        log_bytes(
            &format!("Sending packet ({}/{} bytes)", sent, total),
            &packet,
        );
        port.write_all(&packet)?;
        // No flush — kernel transmits immediately at 921600 baud.
        // Flushing before reading the ACK adds unnecessary latency.

        // Read the 6-byte ACK with a manual retry loop to handle partial reads.
        // Using read() instead of read_exact() avoids losing bytes on a timeout
        // mid-read which would permanently desync the ACK stream.
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
                Ok(n) => {
                    ack_read += n;
                }
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {
                    if ack_read == 0 {
                        error!(
                            "ACK timeout after sending packet (sent {}/{} bytes)",
                            sent, total
                        );
                        return Err(io::Error::new(io::ErrorKind::TimedOut, "ACK timeout"));
                    }
                    // Partial read — keep waiting for the rest
                }
                Err(e) => {
                    error!("ACK read error: {}", e);
                    return Err(e);
                }
            }
        }
        log_bytes("ACK received", &ack);
        if ack[..5] != CTRL_CHARS_HEADER {
            error!("Bad ACK header: {:02x?}", &ack[..6]);
            return Err(io::Error::new(io::ErrorKind::InvalidData, "Bad ACK header"));
        }
        match ack[5] {
            b'A' => debug!("Packet ACK OK"),
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

pub(crate) fn drain_until_silent(port: &mut Box<dyn SerialPort>, silence: Duration) {
    let _ = port.flush();
    // Use a short timeout so reads don't block when the port is empty
    let _ = port.set_timeout(Duration::from_millis(10));
    let mut buf = [0u8; 64];
    loop {
        match port.read(&mut buf) {
            Ok(n) if n > 0 => {
                debug!("Drained {} stale bytes", n);
                // Keep draining immediately — more data may follow
            }
            _ => {
                // Nothing came in within 10ms — sleep the full silence window
                // then do one final check to confirm the port is truly quiet
                sleep(silence);
                match port.read(&mut buf) {
                    Ok(n) if n > 0 => {
                        debug!("Drained {} more stale bytes after silence wait", n);
                        // Still data coming in, loop again
                    }
                    _ => break,
                }
            }
        }
    }
    // Restore the normal ACK timeout
    let _ = port.set_timeout(Duration::from_millis(ZEDMD_COMM_SERIAL_READ_TIMEOUT_MS));
}

pub(crate) fn flush_and_discard(port: &mut Box<dyn SerialPort>) {
    debug!("Flushing and discarding data in the port");
    let _ = port.flush();
    let _ = port.set_timeout(Duration::from_millis(10));
    let mut buffer = [0u8; 64];
    loop {
        match port.read(&mut buffer) {
            Ok(n) if n > 0 => debug!("Flushed {} bytes: {:02x?}", n, &buffer[..n]),
            _ => break,
        }
    }
    let _ = port.set_timeout(Duration::from_millis(ZEDMD_COMM_SERIAL_READ_TIMEOUT_MS));
}

fn log_bytes(message: &str, data: &[u8]) {
    let s: String = data
        .iter()
        .map(|&b| {
            if b.is_ascii_graphic() {
                format!("{}  ", b as char)
            } else {
                format!("{:02x} ", b)
            }
        })
        .collect();
    debug!("{}: [{}] {}", message, data.len(), s);
}

/// Simple FNV-1a 64-bit hash — fast, no deps, good enough for zone change detection.
pub(crate) fn fnv1a_hash(data: &[u8]) -> u64 {
    const FNV_OFFSET: u64 = 0xcbf29ce484222325;
    const FNV_PRIME: u64 = 0x100000001b3;
    let mut hash = FNV_OFFSET;
    for &byte in data {
        hash ^= byte as u64;
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    hash
}
