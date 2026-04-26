use log::{debug, info, warn};
use serialport::SerialPortType::UsbPort;
use serialport::{DataBits, Parity, SerialPort, StopBits};
/// Serial transport layer — mirrors ZeDMDComm.cpp/.h from libzedmd.
/// Handles connection, handshake, chunked writes and ACK protocol.
use std::io::{self, Read, Write};
use std::thread::sleep;
use std::time::{Duration, Instant};

use crate::transport::{Transport, UsbTransport};
use crate::types::ZedmdCommCommand;
use miniz_oxide::deflate::compress_to_vec_zlib;

// The maximum baud rate supported by CP210x.
pub(crate) const ZEDMD_COMM_BAUD_RATE: u32 = 921600;
pub(crate) const ZEDMD_COMM_MIN_SERIAL_WRITE_AT_ONCE: usize = 32;
pub(crate) const ZEDMD_COMM_MAX_SERIAL_WRITE_AT_ONCE: usize = 1920;
pub(crate) const ZEDMD_COMM_DEFAULT_SERIAL_WRITE_AT_ONCE: usize = 64;
pub(crate) const ZEDMD_COMM_KEEP_ALIVE_INTERVAL: u128 = 3000; // ms
pub(crate) const ZEDMD_COMM_SERIAL_READ_TIMEOUT_MS: u64 = 200;
pub(crate) const ZEDMD_WIFI_UDP_KEEP_ALIVE_INTERVAL: u128 = 3000; // ms — matches firmware's UDP timeout
pub(crate) const ZEDMD_WIFI_TCP_KEEP_ALIVE_INTERVAL: u128 = 100; // ms — matches libzedmd's TCP setting

/// ZEDMD_ZONES_BYTE_LIMIT_RGB565 = 128*4*2+16 = 1040
pub(crate) const ZEDMD_ZONES_BYTE_LIMIT_RGB565: usize = 128 * 4 * 2 + 16;
/// ZEDMD_ZONES_BYTE_LIMIT_RGB888: the theoretical value (128*4*3+16 = 1552) exceeds the
/// firmware's BUFFER_SIZE of 1152, causing the device to display "payloadSize > BUFFER_SIZE".
/// We cap at 1152 to match the firmware buffer so sub-frames stay within bounds.
pub(crate) const ZEDMD_ZONES_BYTE_LIMIT_RGB888: usize = 1152;

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
    pub transport: Transport,
    pub keep_alive: bool,
    pub last_keep_alive: Instant,
    /// Hashes of the last-sent zone contents, used to skip unchanged zones.
    /// 0 = unknown / never sent (force send). 1 = known black.
    pub zone_hashes: Vec<u64>,
}

impl SharedZeDMDComm {
    /// Send a simple command (no payload) using the full framing protocol.
    pub fn send_simple_command(&mut self, cmd: ZedmdCommCommand) -> io::Result<()> {
        let payload = build_payload_simple(cmd);
        let result = self.transport.send(&payload);
        if result.is_ok() {
            self.last_keep_alive = Instant::now();
        }
        result
    }

    /// Send a command with a single byte of payload.
    pub fn send_command_with_byte(&mut self, cmd: ZedmdCommCommand, value: u8) -> io::Result<()> {
        let mut payload = Vec::with_capacity(FRAME_HEADER.len() + CTRL_CHARS_HEADER.len() + 5);
        payload.extend_from_slice(&FRAME_HEADER);
        payload.extend_from_slice(&CTRL_CHARS_HEADER);
        payload.push(cmd as u8);
        payload.push(0); // size high byte (payload = 1 byte)
        payload.push(1); // size low byte
        payload.push(0); // compression flag
        payload.push(value);
        let result = self.transport.send(&payload);
        if result.is_ok() {
            self.last_keep_alive = Instant::now();
        }
        result
    }

    /// Send a command with arbitrary byte payload (used for strings like WiFi credentials).
    pub fn send_command_with_buffer(
        &mut self,
        cmd: ZedmdCommCommand,
        buffer: &[u8],
    ) -> io::Result<()> {
        let mut payload =
            Vec::with_capacity(FRAME_HEADER.len() + CTRL_CHARS_HEADER.len() + 5 + buffer.len());
        payload.extend_from_slice(&FRAME_HEADER);
        payload.extend_from_slice(&CTRL_CHARS_HEADER);
        payload.push(cmd as u8);
        let size = buffer.len() as u16;
        payload.push((size >> 8) as u8); // size high byte
        payload.push((size & 0xFF) as u8); // size low byte
        payload.push(0); // compression flag
        payload.extend_from_slice(buffer);
        let result = self.transport.send(&payload);
        if result.is_ok() {
            self.last_keep_alive = Instant::now();
        }
        result
    }

    /// Send a command with an integer (u16) payload.
    pub fn send_command_with_u16(&mut self, cmd: ZedmdCommCommand, value: u16) -> io::Result<()> {
        let mut payload = Vec::with_capacity(FRAME_HEADER.len() + CTRL_CHARS_HEADER.len() + 7);
        payload.extend_from_slice(&FRAME_HEADER);
        payload.extend_from_slice(&CTRL_CHARS_HEADER);
        payload.push(cmd as u8);
        payload.push(0); // size high byte (payload = 2 bytes)
        payload.push(2); // size low byte
        payload.push(0); // compression flag
        payload.push((value & 0xFF) as u8);
        payload.push((value >> 8) as u8);
        let result = self.transport.send(&payload);
        if result.is_ok() {
            self.last_keep_alive = Instant::now();
        }
        result
    }

    /// Render a full RGB565 frame using the zone-streaming protocol.
    /// Only zones whose content has changed since the last call are transmitted.
    pub fn render_rgb565_frame(&mut self, pixels: &[u16]) -> io::Result<()> {
        self.keep_alive_tick()?;
        let width = self.width as usize;
        let height = self.height as usize;
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
        // Convert u16 pixels to little-endian bytes
        let bytes: Vec<u8> = pixels
            .iter()
            .flat_map(|&p| [(p & 0xFF) as u8, (p >> 8) as u8])
            .collect();
        self.render_zones(
            &bytes,
            2,
            ZedmdCommCommand::RGB565ZonesStream,
            ZEDMD_ZONES_BYTE_LIMIT_RGB565,
        )
    }

    /// Render a full RGB888 frame using the zone-streaming protocol.
    /// Only zones whose content has changed since the last call are transmitted.
    /// Pixels are packed `[r, g, b, r, g, b, …]`.
    pub fn render_rgb888_frame(&mut self, pixels: &[u8]) -> io::Result<()> {
        self.keep_alive_tick()?;
        let width = self.width as usize;
        let height = self.height as usize;
        if width == 0 || height == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Device dimensions not set",
            ));
        }
        if pixels.len() != width * height * 3 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "Expected {} bytes ({}x{}x3), got {}",
                    width * height * 3,
                    width,
                    height,
                    pixels.len()
                ),
            ));
        }
        self.render_zones(
            pixels,
            3,
            ZedmdCommCommand::RGB888ZonesStream,
            ZEDMD_ZONES_BYTE_LIMIT_RGB888,
        )
    }

    /// Generic zone-streaming implementation shared by RGB565 and RGB888.
    /// `bytes_per_pixel` is 2 (RGB565) or 3 (RGB888).
    fn render_zones(
        &mut self,
        pixels: &[u8],
        bytes_per_pixel: usize,
        command: ZedmdCommCommand,
        zones_byte_limit: usize,
    ) -> io::Result<()> {
        let width = self.width as usize;
        let height = self.height as usize;
        let zone_width = self.zone_width as usize;
        let zone_height = self.zone_height as usize;
        let zone_bytes = zone_width * zone_height * bytes_per_pixel;
        let zones_x = width / zone_width;
        let zones_y = height / zone_height;

        let mut all_zone_data: Vec<u8> = Vec::new();
        let mut changed_zones = 0usize;
        let mut idx: u8 = 0;

        for zy in 0..zones_y {
            for zx in 0..zones_x {
                let mut zone_buf = vec![0u8; zone_bytes];
                let mut all_black = true;
                for row in 0..zone_height {
                    for col in 0..zone_width {
                        let px_offset = ((zy * zone_height + row) * width
                            + (zx * zone_width + col))
                            * bytes_per_pixel;
                        let zone_offset = (row * zone_width + col) * bytes_per_pixel;
                        zone_buf[zone_offset..zone_offset + bytes_per_pixel]
                            .copy_from_slice(&pixels[px_offset..px_offset + bytes_per_pixel]);
                        if pixels[px_offset..px_offset + bytes_per_pixel]
                            .iter()
                            .any(|&b| b != 0)
                        {
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

        let zone_bytes_total = zone_bytes + 1;
        let buffer_size_threshold = zones_byte_limit - zone_bytes_total;

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

        let n_sub_frames = sub_frames.len();

        // Each sub-frame is an independent firmware transaction: FRAME header,
        // zone data command, then RenderFrame. RenderFrame returns result=1
        // which causes the firmware to break out and wait for a new FRAME header,
        // so each sub-frame must be its own send_chunks call.
        // Sub-frames are sent in reverse order matching C++ StreamBytes (rbegin/rend).
        for (sf_idx, chunk) in sub_frames.into_iter().rev().enumerate() {
            let raw_size = chunk.len();
            let compressed = compress_to_vec_zlib(&chunk, 6);
            let (data, compressed_flag) = if compressed.len() < chunk.len() {
                (compressed.as_slice(), 1u8)
            } else {
                (chunk.as_slice(), 0u8)
            };
            let data_size = data.len();
            debug!(
                "Sub-frame {}/{}: raw={} compressed={} flag={} command={:?}",
                sf_idx + 1,
                n_sub_frames,
                raw_size,
                data_size,
                compressed_flag,
                command
            );
            let mut payload: Vec<u8> = Vec::with_capacity(
                FRAME_HEADER.len() + CTRL_CHARS_HEADER.len() * 2 + 8 + data_size,
            );
            payload.extend_from_slice(&FRAME_HEADER);
            payload.extend_from_slice(&CTRL_CHARS_HEADER);
            payload.push(command as u8);
            payload.push((data_size >> 8) as u8);
            payload.push((data_size & 0xFF) as u8);
            payload.push(compressed_flag);
            payload.extend_from_slice(data);
            payload.extend_from_slice(&CTRL_CHARS_HEADER);
            payload.push(ZedmdCommCommand::RenderFrame as u8);
            payload.push(0);
            payload.push(0);
            payload.push(0);
            self.transport.send(&payload)?;
        }

        debug!(
            "Sent {:?} delta frame: {} changed zones, {} raw bytes, {} sub-frame(s)",
            command,
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
        let interval = match &self.transport {
            Transport::Usb(_) => ZEDMD_COMM_KEEP_ALIVE_INTERVAL,
            Transport::WifiUdp(_) => ZEDMD_WIFI_UDP_KEEP_ALIVE_INTERVAL,
            Transport::WifiTcp(_) => ZEDMD_WIFI_TCP_KEEP_ALIVE_INTERVAL,
        };
        if now.duration_since(self.last_keep_alive).as_millis() < interval {
            return Ok(());
        }
        self.last_keep_alive = now;
        let payload = build_payload_simple(ZedmdCommCommand::KeepAlive);
        debug!("Sending keep-alive");
        self.transport.send(&payload)
    }
}

/// Scan all USB serial ports and return a connected `SharedZeDMDComm` for the first ZeDMD found.
/// Retries for up to 10 seconds to handle USB re-enumeration after sleep/wake.
pub(crate) fn connect_internal() -> io::Result<SharedZeDMDComm> {
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut last_err = io::Error::new(io::ErrorKind::NotFound, "No ZeDMD device found");

    loop {
        match try_connect_once() {
            Some(comm) => return Ok(comm),
            None => {
                if Instant::now() >= deadline {
                    return Err(last_err);
                }
                info!("No ZeDMD found yet, retrying in 1s...");
                sleep(Duration::from_secs(1));
                last_err = io::Error::new(io::ErrorKind::NotFound, "No ZeDMD device found");
            }
        }
    }
}

fn try_connect_once() -> Option<SharedZeDMDComm> {
    let ports = match serialport::available_ports() {
        Ok(p) => p,
        Err(e) => {
            debug!("Failed to enumerate ports: {}", e);
            return None;
        }
    };
    for p in ports {
        if let UsbPort(info) = p.port_type {
            let port_name = p.port_name.clone();
            debug!(
                "ZeDMD candidate: device={}, vid={:04x}, pid={:04x}",
                port_name, info.vid, info.pid
            );
            let port = match serialport::new(&p.port_name, ZEDMD_COMM_BAUD_RATE)
                .parity(Parity::None)
                .data_bits(DataBits::Eight)
                .stop_bits(StopBits::One)
                .flow_control(serialport::FlowControl::None)
                .timeout(Duration::from_millis(ZEDMD_COMM_SERIAL_READ_TIMEOUT_MS))
                .open()
            {
                Ok(p) => p,
                Err(e) => {
                    debug!("Failed to open {}: {}", port_name, e);
                    continue;
                }
            };

            if let Some(internal) = try_handshake(port_name, port) {
                return Some(internal);
            }
        }
    }
    None
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

        let transport = Transport::Usb(UsbTransport {
            port: Some(port),
            write_at_once,
        });

        let mut internal = SharedZeDMDComm {
            width,
            height,
            zone_width,
            zone_height,
            firmware_version: format!("{}.{}.{}", data[8], data[9], data[10]),
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
            transport,
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
            internal.transport.write_at_once()
        );
        if internal.transport.write_at_once() <= 64 {
            warn!(
                "The ZeDMD USB package size of {} is very low. Try to increase it for smoother animations.",
                internal.transport.write_at_once()
            );
        }
        if internal.panel_min_refresh_rate <= 30 {
            warn!(
                "The ZeDMD panel minimal refresh rate of {} is very low. Try to increase it for smoother animations.",
                internal.panel_min_refresh_rate
            );
        }

        if let Transport::Usb(UsbTransport { port: Some(p), .. }) = &mut internal.transport {
            flush_and_discard(p);
            let _ = p.set_timeout(Duration::from_millis(ZEDMD_COMM_SERIAL_READ_TIMEOUT_MS));
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
