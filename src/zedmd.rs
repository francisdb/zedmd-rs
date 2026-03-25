use crate::comm::{
    SharedZeDMDComm, ZEDMD_COMM_BAUD_RATE, ZEDMD_COMM_SERIAL_READ_TIMEOUT_MS, connect_internal,
    try_handshake,
};
use crate::queue::{FramePixels, FrameQueue};
use crate::scale::{scale_rgb565, scale_rgb888};
use crate::types::{RgbOrder, TransportMode, ZedmdCommCommand};
use log::{error, info};
use serialport::{DataBits, Parity, StopBits};
/// Public ZeDMD API — mirrors ZeDMD.cpp/.h from libzedmd.
use std::io;
use std::sync::{Arc, Mutex};
use std::thread;
use std::thread::sleep;
use std::time::{Duration, Instant};

/// Handle to the ZeDMD device.
///
/// Frames submitted via [`render_rgb565_frame`] or [`render_rgb888_frame`] are
/// queued and forwarded to the device by a dedicated background thread (matching
/// libzedmd's async model). The caller returns immediately after queuing; stale
/// frames are dropped if the device can't keep up.
pub struct ZeDMDComm {
    pub(crate) comm: Arc<Mutex<SharedZeDMDComm>>,
    queue: FrameQueue,
    thread: Option<thread::JoinHandle<()>>,
    /// For usb_fps() measurement
    last_fps_instant: Instant,
    last_fps_sent: u64,
    /// Source (ROM) resolution set via set_frame_size(). Defaults to panel size.
    rom_width: u32,
    rom_height: u32,
    /// When true, small sources are pixel-doubled instead of centred.
    upscaling: bool,
    /// When true, RGB888 frames are sent as RGB888ZonesStream (cmd 0x04).
    /// When false (default), they are converted to RGB565 before sending.
    true_rgb888: bool,
}

pub fn connect() -> io::Result<ZeDMDComm> {
    let internal = connect_internal()?;
    let panel_width = internal.width;
    let panel_height = internal.height;
    let comm = Arc::new(Mutex::new(internal));
    let queue = FrameQueue::new();

    // Spawn the background streaming thread
    let comm_thread = Arc::clone(&comm);
    let queue_thread = queue.clone();
    let handle = thread::Builder::new()
        .name("zedmd-stream".into())
        .spawn(move || {
            while let Some(frame) = queue_thread.pop() {
                let mut c = comm_thread.lock().unwrap();
                let result = match &frame.pixels {
                    FramePixels::Rgb565(pixels) => c.render_rgb565_frame(pixels),
                    FramePixels::Rgb888(pixels) => c.render_rgb888_frame(pixels),
                };
                if let Err(e) = result {
                    error!("Stream thread render error: {}", e);
                } else {
                    queue_thread.mark_sent(frame.id);
                }
            }
            info!("ZeDMD stream thread exiting");
        })
        .expect("Failed to spawn zedmd-stream thread");

    Ok(ZeDMDComm {
        comm,
        queue,
        thread: Some(handle),
        last_fps_instant: Instant::now(),
        last_fps_sent: 0,
        rom_width: panel_width,
        rom_height: panel_height,
        upscaling: false,
        true_rgb888: false,
    })
}

impl Drop for ZeDMDComm {
    fn drop(&mut self) {
        self.queue.stop();
        if let Some(handle) = self.thread.take() {
            let _ = handle.join();
        }
    }
}

impl ZeDMDComm {
    /// Start the keep-alive ticker. Call once after connecting.
    pub fn run(&mut self) -> io::Result<()> {
        let mut comm = self.comm.lock().unwrap();
        comm.keep_alive = true;
        comm.last_keep_alive = Instant::now();
        Ok(())
    }

    /// Stop sending keep-alives and flush the port.
    pub fn stop(&mut self) -> io::Result<()> {
        let mut comm = self.comm.lock().unwrap();
        comm.keep_alive = false;
        if let Some(port) = &mut comm.port {
            let _ = port.flush();
        }
        info!("ZeDMDComm stopped");
        Ok(())
    }

    pub fn reconnect(&mut self) -> io::Result<()> {
        let device_name = {
            let mut comm = self.comm.lock().unwrap();
            comm.port = None;
            comm.device_name.clone()
        };
        info!("Reconnecting to {}...", device_name);
        sleep(Duration::from_millis(2000));
        let port = serialport::new(&device_name, ZEDMD_COMM_BAUD_RATE)
            .parity(Parity::None)
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .timeout(Duration::from_millis(ZEDMD_COMM_SERIAL_READ_TIMEOUT_MS))
            .open()
            .map_err(|e| {
                io::Error::new(
                    io::ErrorKind::NotFound,
                    format!("Failed to reopen {}: {}", device_name, e),
                )
            })?;
        let internal = try_handshake(device_name, port).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::NotFound,
                "Re-handshake failed after reconnect",
            )
        })?;
        let mut comm = self.comm.lock().unwrap();
        comm.width = internal.width;
        comm.height = internal.height;
        comm.zone_width = internal.zone_width;
        comm.zone_height = internal.zone_height;
        comm.firmware_version = internal.firmware_version;
        comm.write_at_once = internal.write_at_once;
        comm.brightness = internal.brightness;
        comm.rgb_mode = internal.rgb_mode;
        comm.y_offset = internal.y_offset;
        comm.panel_clkphase = internal.panel_clkphase;
        comm.panel_driver = internal.panel_driver;
        comm.panel_i2sspeed = internal.panel_i2sspeed;
        comm.panel_latch_blanking = internal.panel_latch_blanking;
        comm.panel_min_refresh_rate = internal.panel_min_refresh_rate;
        comm.udp_delay = internal.udp_delay;
        comm.half = internal.half;
        comm.s3 = internal.s3;
        comm.id = internal.id;
        comm.port = internal.port;
        comm.zone_hashes = internal.zone_hashes;
        comm.last_keep_alive = Instant::now();
        Ok(())
    }

    /// Queue an RGB565 frame for async rendering. Returns a frame ID that can
    /// be passed to [`wait_for_frame`] to block until the device has received it.
    /// If the source resolution was set via [`set_frame_size`], the frame is
    /// scaled to the panel size before queuing. If a frame is already pending
    /// it is replaced (latest-frame-wins).
    pub fn render_rgb565_frame(&self, pixels: &[u16]) -> u64 {
        let comm = self.comm.lock().unwrap();
        let panel_w = comm.width;
        let panel_h = comm.height;
        drop(comm);

        let scaled = if self.rom_width != panel_w || self.rom_height != panel_h {
            scale_rgb565(
                pixels,
                self.rom_width,
                self.rom_height,
                panel_w,
                panel_h,
                self.upscaling,
            )
        } else {
            pixels.to_vec()
        };
        self.queue.push_rgb565(scaled)
    }

    /// Queue an RGB888 frame for async rendering. Pixels are packed
    /// `[r, g, b, r, g, b, …]`. If [`enable_true_rgb888`] has been called the
    /// frame is sent as RGB888ZonesStream (command 0x04); otherwise it is
    /// converted to RGB565 first (matching libzedmd's default behaviour).
    /// Scaling via [`set_frame_size`] is applied before conversion.
    pub fn render_rgb888_frame(&self, pixels: &[u8]) -> u64 {
        let comm = self.comm.lock().unwrap();
        let panel_w = comm.width;
        let panel_h = comm.height;
        drop(comm);

        let scaled = if self.rom_width != panel_w || self.rom_height != panel_h {
            scale_rgb888(
                pixels,
                self.rom_width,
                self.rom_height,
                panel_w,
                panel_h,
                self.upscaling,
            )
        } else {
            pixels.to_vec()
        };

        if self.true_rgb888 {
            self.queue.push_rgb888(scaled)
        } else {
            // Convert RGB888 → RGB565 (matching libzedmd default)
            let rgb565: Vec<u16> = scaled
                .chunks_exact(3)
                .map(|c| {
                    let r = c[0] as u16;
                    let g = c[1] as u16;
                    let b = c[2] as u16;
                    ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
                })
                .collect();
            self.queue.push_rgb565(rgb565)
        }
    }

    /// Set the source (ROM) resolution of frames you will submit. When this
    /// differs from the panel size the client will automatically scale frames
    /// before sending them to the device, matching libzedmd's `SetFrameSize`
    /// behaviour. Call before the first [`render_rgb565_frame`] or
    /// [`render_rgb888_frame`].
    pub fn set_frame_size(&mut self, width: u32, height: u32) {
        info!("Setting frame size to {}x{}", width, height);
        self.rom_width = width;
        self.rom_height = height;
    }

    /// Enable upscaling: when the source resolution is smaller than the panel,
    /// frames are pixel-doubled (2×) instead of centred with black borders.
    /// Matches libzedmd's `EnableUpscaling`. No device command is sent.
    pub fn enable_upscaling(&mut self) {
        info!("Upscaling enabled");
        self.upscaling = true;
    }

    /// Disable upscaling: small sources are centred with black borders (default).
    /// Matches libzedmd's `DisableUpscaling`. No device command is sent.
    pub fn disable_upscaling(&mut self) {
        info!("Upscaling disabled");
        self.upscaling = false;
    }

    /// Enable true RGB888 transport: [`render_rgb888_frame`] will send
    /// RGB888ZonesStream (command 0x04) instead of converting to RGB565 first.
    /// Use this for maximum colour fidelity at the cost of ~50% more USB bandwidth.
    /// Matches libzedmd's `EnableTrueRgb888(true)`. No device command is sent.
    pub fn enable_true_rgb888(&mut self) {
        info!("True RGB888 enabled");
        self.true_rgb888 = true;
    }

    /// Disable true RGB888: [`render_rgb888_frame`] converts to RGB565 before
    /// sending (default). Matches libzedmd's `EnableTrueRgb888(false)`.
    pub fn disable_true_rgb888(&mut self) {
        info!("True RGB888 disabled");
        self.true_rgb888 = false;
    }

    /// Non-blocking check: returns `true` if the frame with the given ID has
    /// already been sent to the device. Use this in animation loops to skip
    /// rendering a new frame when the device hasn't finished the previous one,
    /// without blocking the caller.
    pub fn is_frame_sent(&self, id: u64) -> bool {
        self.queue.is_frame_sent(id)
    }

    /// Block until the frame with the given ID has been sent to the device.
    /// Use the ID returned by [`render_rgb565_frame`]. Prefer this for
    /// sequential use cases (e.g. pixel scan) where you must not advance
    /// until the device has received a specific frame.
    pub fn wait_for_frame(&self, id: u64) {
        self.queue.wait_for_frame(id);
    }

    /// Returns the actual USB send rate (frames/sec) since the last call to
    /// this method. Use this alongside the submit fps to see how many frames
    /// the device is actually keeping up with.
    pub fn usb_fps(&mut self) -> f64 {
        let now = Instant::now();
        let elapsed = now.duration_since(self.last_fps_instant).as_secs_f64();
        let sent = self.queue.sent_count();
        let delta = sent.saturating_sub(self.last_fps_sent);
        self.last_fps_instant = now;
        self.last_fps_sent = sent;
        if elapsed > 0.0 {
            delta as f64 / elapsed
        } else {
            0.0
        }
    }

    /// Clear the display synchronously (bypasses the frame queue).
    pub fn clear_screen(&self) -> io::Result<()> {
        let mut comm = self.comm.lock().unwrap();
        let result = comm.send_simple_command(ZedmdCommCommand::ClearScreen);
        if result.is_ok() {
            comm.zone_hashes.fill(1);
        }
        result
    }

    pub fn led_test(&self) -> io::Result<()> {
        self.comm
            .lock()
            .unwrap()
            .send_simple_command(ZedmdCommCommand::LEDTest)
    }

    pub fn reset(&self) -> io::Result<()> {
        self.comm
            .lock()
            .unwrap()
            .send_simple_command(ZedmdCommCommand::Reset)
    }

    /// Width of the display in pixels, as reported by the device at handshake.
    pub fn width(&self) -> u32 {
        self.comm.lock().unwrap().width
    }

    /// Height of the display in pixels, as reported by the device at handshake.
    pub fn height(&self) -> u32 {
        self.comm.lock().unwrap().height
    }

    /// USB packet size in bytes. The device sends this at handshake.
    /// Each USB packet is padded/split to this size. Valid range: 32–1920.
    pub fn write_at_once(&self) -> usize {
        self.comm.lock().unwrap().write_at_once
    }

    /// Display brightness, 0 (off) to 255 (maximum).
    pub fn brightness(&self) -> u8 {
        self.comm.lock().unwrap().brightness
    }

    /// Vertical display offset in pixels. Used on split/half panels.
    pub fn y_offset(&self) -> u8 {
        self.comm.lock().unwrap().y_offset
    }

    /// Minimum panel refresh rate in Hz (typically 30–200).
    /// This is the internal PWM refresh rate of the LED matrix hardware —
    /// higher values reduce flicker and improve colour depth, at the cost of
    /// more ESP32 CPU/DMA load.
    pub fn panel_min_refresh_rate(&self) -> u8 {
        self.comm.lock().unwrap().panel_min_refresh_rate
    }

    /// Panel clock phase (0 or 1). Affects signal timing on some panels.
    /// Try toggling if you see ghosting or colour artefacts.
    pub fn panel_clock_phase(&self) -> u8 {
        self.comm.lock().unwrap().panel_clkphase
    }

    /// Panel I2S clock speed index (minimum 8). Higher values drive the panel
    /// shift registers faster; may need lowering on long ribbon cables.
    pub fn panel_i2s_speed(&self) -> u8 {
        self.comm.lock().unwrap().panel_i2sspeed
    }

    /// Panel latch blanking cycles. Controls how long the row latch is held
    /// before clocking in the next row. Increase if you see row bleed-through.
    pub fn panel_latch_blanking(&self) -> u8 {
        self.comm.lock().unwrap().panel_latch_blanking
    }

    /// Panel driver chip index. Selects the shift-register protocol used by
    /// the HUB75 driver IC (e.g. standard, FM6126, FM6127, ICN2038S …).
    pub fn panel_driver(&self) -> u8 {
        self.comm.lock().unwrap().panel_driver
    }

    /// RGB channel order of the LED matrix.
    pub fn rgb_order(&self) -> RgbOrder {
        RgbOrder::from_u8(self.comm.lock().unwrap().rgb_mode).unwrap_or(RgbOrder::Rgb)
    }

    /// Set the USB packet size in bytes. Valid range: 32–1920; rounded down to
    /// the nearest multiple of 32. Larger values reduce per-packet overhead and
    /// improve throughput. The new size takes effect immediately for subsequent
    /// frames but is **not** persisted — call [`save_settings`] to store it.
    pub fn set_usb_package_size(&self, size: u16) -> io::Result<()> {
        let size = (size.clamp(32, 1920) / 32) * 32;
        let multiplier = (size / 32) as u8;
        info!(
            "Setting USB package size to {} (multiplier={})",
            size, multiplier
        );
        let mut comm = self.comm.lock().unwrap();
        comm.send_command_with_byte(ZedmdCommCommand::SetUsbPackageSizeMultiplier, multiplier)?;
        comm.write_at_once = size as usize;
        Ok(())
    }

    /// Set the minimum internal PWM refresh rate of the LED matrix panel in Hz.
    /// Recommended minimum is 60 Hz to avoid visible flicker. The change takes
    /// effect after the next device restart — call [`save_settings`] to persist.
    pub fn set_min_refresh_rate(&self, rate: u8) -> io::Result<()> {
        info!("Setting panel min refresh rate to {}", rate);
        self.comm
            .lock()
            .unwrap()
            .send_command_with_byte(ZedmdCommCommand::SetMinRefreshRate, rate)
    }

    /// Set display brightness (0 = off, 255 = maximum). Takes effect immediately.
    /// Call [`save_settings`] to persist across reboots.
    pub fn set_brightness(&self, brightness: u8) -> io::Result<()> {
        info!("Setting brightness to {}", brightness);
        self.comm
            .lock()
            .unwrap()
            .send_command_with_byte(ZedmdCommCommand::Brightness, brightness)
    }

    /// Set the RGB channel order of the LED matrix. Use this when colours appear
    /// swapped (e.g. red shows as blue). Takes effect immediately.
    /// Call [`save_settings`] to persist across reboots.
    pub fn set_rgb_order(&self, order: RgbOrder) -> io::Result<()> {
        info!("Setting RGB order to {}", order);
        self.comm
            .lock()
            .unwrap()
            .send_command_with_byte(ZedmdCommCommand::RGBOrder, order as u8)
    }

    /// Set the vertical pixel offset. Used on half-height or split panels to
    /// align the image correctly. Call [`save_settings`] to persist.
    pub fn set_y_offset(&self, offset: u8) -> io::Result<()> {
        info!("Setting Y offset to {}", offset);
        self.comm
            .lock()
            .unwrap()
            .send_command_with_byte(ZedmdCommCommand::SetYOffset, offset)
    }

    /// Set the panel clock phase (0 or 1). Try toggling if you see ghosting or
    /// colour artefacts at higher I2S speeds. Call [`save_settings`] to persist.
    pub fn set_panel_clock_phase(&self, phase: u8) -> io::Result<()> {
        info!("Setting panel clock phase to {}", phase);
        self.comm
            .lock()
            .unwrap()
            .send_command_with_byte(ZedmdCommCommand::SetClkphase, phase)
    }

    /// Set the panel I2S clock speed index (minimum 8). Lower values are safer
    /// on long cables; higher values allow faster row scanning. Call
    /// [`save_settings`] to persist.
    pub fn set_panel_i2s_speed(&self, speed: u8) -> io::Result<()> {
        let speed = speed.max(8);
        info!("Setting panel I2S speed to {}", speed);
        self.comm
            .lock()
            .unwrap()
            .send_command_with_byte(ZedmdCommCommand::SetI2sspeed, speed)
    }

    /// Set the transport mode (`USB`, `WiFi UDP`, `WiFi TCP`, `SPI`).
    ///
    /// The mode flag is updated immediately in RAM, but becomes active only
    /// after reboot. Call [`save_settings`] to persist it.
    pub fn set_transport_mode(&self, mode: TransportMode) -> io::Result<()> {
        info!("Setting transport mode to {}", mode);
        self.comm
            .lock()
            .unwrap()
            .send_command_with_byte(ZedmdCommCommand::SetTransport, mode as u8)
    }

    /// Set the WiFi UDP inter-packet delay (0-9). Only relevant in WiFi UDP mode.
    /// Call [`save_settings`] to persist.
    pub fn set_udp_delay(&self, delay: u8) -> io::Result<()> {
        let delay = delay.min(9);
        info!("Setting UDP delay to {}", delay);
        self.comm
            .lock()
            .unwrap()
            .send_command_with_byte(ZedmdCommCommand::SetUdpDelay, delay)
    }

    /// Set the panel latch blanking cycles. Increase if you see row bleed-through
    /// or ghosting between rows. Call [`save_settings`] to persist.
    pub fn set_panel_latch_blanking(&self, blanking: u8) -> io::Result<()> {
        info!("Setting panel latch blanking to {}", blanking);
        self.comm
            .lock()
            .unwrap()
            .send_command_with_byte(ZedmdCommCommand::SetLatchBlanking, blanking)
    }

    /// Set the panel driver chip protocol index. Selects the HUB75 driver IC
    /// variant (0 = standard, 1 = FM6126A, 2 = FM6127, 3 = ICN2038S, 4 = MBI5124,
    /// 5 = SM5266P, 6 = DP3246_BP3446). Call [`save_settings`] to persist.
    pub fn set_panel_driver(&self, driver: u8) -> io::Result<()> {
        info!("Setting panel driver to {}", driver);
        self.comm
            .lock()
            .unwrap()
            .send_command_with_byte(ZedmdCommCommand::SetDriver, driver)
    }

    /// Persist all current settings to the device's flash storage.
    ///
    /// Note: the device sends its ACK *before* the flash write completes, so
    /// this call returns quickly even though the write takes a moment.
    pub fn save_settings(&self) -> io::Result<()> {
        info!("Saving settings to device flash");
        self.comm
            .lock()
            .unwrap()
            .send_simple_command(ZedmdCommCommand::SaveSettings)
    }

    /// Enable firmware debug output (session only; use `save_settings` to persist).
    pub fn enable_debug(&self) -> io::Result<()> {
        info!("Enabling debug output on device");
        self.comm
            .lock()
            .unwrap()
            .send_simple_command(ZedmdCommCommand::EnableDebug)
    }

    /// Disable firmware debug output (session only; use `save_settings` to persist).
    pub fn disable_debug(&self) -> io::Result<()> {
        info!("Disabling debug output on device");
        self.comm
            .lock()
            .unwrap()
            .send_simple_command(ZedmdCommCommand::DisableDebug)
    }

    /// Set the WiFi SSID (network name). The device must have WiFi capability enabled.
    /// Call [`save_settings`] to persist across reboots. Maximum 31 characters.
    pub fn set_wifi_ssid(&self, ssid: &str) -> io::Result<()> {
        if ssid.len() > 31 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "SSID must be 31 characters or less",
            ));
        }
        info!("Setting WiFi SSID to '{}'", ssid);
        self.comm
            .lock()
            .unwrap()
            .send_command_with_buffer(ZedmdCommCommand::SetWiFiSSID, ssid.as_bytes())
    }

    /// Set the WiFi password. The device must have WiFi capability enabled.
    /// Call [`save_settings`] to persist across reboots. Maximum 63 characters.
    pub fn set_wifi_password(&self, password: &str) -> io::Result<()> {
        if password.len() > 63 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "Password must be 63 characters or less",
            ));
        }
        info!("Setting WiFi password");
        self.comm
            .lock()
            .unwrap()
            .send_command_with_buffer(ZedmdCommCommand::SetWiFiPassword, password.as_bytes())
    }

    /// Set the WiFi port number (TCP/UDP port the device listens on).
    /// Valid range is 1-65535. Call [`save_settings`] to persist across reboots.
    pub fn set_wifi_port(&self, port: u16) -> io::Result<()> {
        if port == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "Port must be between 1 and 65535",
            ));
        }
        info!("Setting WiFi port to {}", port);
        self.comm
            .lock()
            .unwrap()
            .send_command_with_u16(ZedmdCommCommand::SetWiFiPort, port)
    }

    /// Set the WiFi transmit power level (0-127, where 127 is maximum power).
    /// Lower values reduce power consumption and interference. Default is usually 80.
    /// Call [`save_settings`] to persist across reboots.
    pub fn set_wifi_power(&self, power: u8) -> io::Result<()> {
        info!("Setting WiFi power to {}", power);
        self.comm
            .lock()
            .unwrap()
            .send_command_with_byte(ZedmdCommCommand::SetWiFiPower, power)
    }
}
