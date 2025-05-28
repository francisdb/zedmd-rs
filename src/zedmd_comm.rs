use std::fmt::Display;
use std::io::Write;
use serialport::{DataBits, Parity, SerialPort, StopBits};
use serialport::SerialPortType::UsbPort;
use std::io;
use std::sync::{Arc, Mutex};
use log::{debug, error, info, warn};
use std::thread::sleep;
use std::time::{Duration, Instant};

// The maximum baud rate supported by CP210x. CH340 and others might be able to run higher baudrates, but on the ESP32
// we don't know which USB-to-serial converter is in use.
const ZEDMD_COMM_BAUD_RATE: u32 = 921600;
const ZEDMD_COMM_MIN_SERIAL_WRITE_AT_ONCE: usize = 32;
const ZEDMD_COMM_MAX_SERIAL_WRITE_AT_ONCE: usize = 1920;
const ZEDMD_COMM_DEFAULT_SERIAL_WRITE_AT_ONCE: usize = 64;

const ZEDMD_COMM_KEEP_ALIVE_INTERVAL: usize = 3000;

#[derive(Debug)]
pub struct ZeDMDComm {
    comm: Arc<Mutex<SharedZeDMDComm>>
}

#[derive(Debug)]
struct SharedZeDMDComm {
    width: u32,
    height: u32,
    zone_width: u32,
    zone_height: u32,
    firmware_version: String,
    write_at_once: u32,
    brightness: u8,
    rgb_mode: u8,
    y_offset: u8,
    panel_clkphase: u8,
    panel_driver: u8,
    panel_i2sspeed: u8,
    panel_latch_blanking: u8,
    panel_min_refresh_rate: u8,
    udp_delay: u8,
    half: bool,
    s3: bool, // S3 model
    id: u16, // device ID
    device_name: String, // port name
    port: Option<Box<dyn SerialPort>>,
    keep_alive: bool,
    last_keep_alive: Instant,
    running: bool
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZedmdCommCommand {
    FrameSize = 0x02,
    Handshake = 0x0c,
    LEDTest = 0x10,
    EnableUpscaling = 0x15,
    DisableUpscaling = 0x14,
    Brightness = 0x16,
    RGBOrder = 0x17,
    SetWiFiPower = 0x1a,
    SetWiFiSSID = 0x1b,
    SetWiFiPassword = 0x1c,
    SetWiFiPort = 0x1d,
    SaveSettings = 0x1e,
    Reset = 0x1f,

    SetClkphase = 0x28,
    SetI2sspeed = 0x29,
    SetLatchBlanking = 0x2a,
    SetMinRefreshRate = 0x2b,
    SetDriver = 0x2c,
    SetTransport = 0x2d,
    SetUdpDelay = 0x2e,
    SetUsbPackageSizeMultiplier = 0x2f,
    SetYOffset = 0x30,

    RGB565ZonesStream = 0x05,
    RenderRGB565Frame = 0x06,

    ClearScreen = 0x0a,

    KeepAlive = 0x0b, // 16

    DisableDebug = 0x62,
    EnableDebug = 0x63,
}

impl Display for ZedmdCommCommand {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let name = match self {
            ZedmdCommCommand::FrameSize => "FrameSize",
            ZedmdCommCommand::Handshake => "Handshake",
            ZedmdCommCommand::LEDTest => "LEDTest",
            ZedmdCommCommand::EnableUpscaling => "EnableUpscaling",
            ZedmdCommCommand::DisableUpscaling => "DisableUpscaling",
            ZedmdCommCommand::Brightness => "Brightness",
            ZedmdCommCommand::RGBOrder => "RGBOrder",
            ZedmdCommCommand::SetWiFiPower => "SetWiFiPower",
            ZedmdCommCommand::SetWiFiSSID => "SetWiFiSSID",
            ZedmdCommCommand::SetWiFiPassword => "SetWiFiPassword",
            ZedmdCommCommand::SetWiFiPort => "SetWiFiPort",
            ZedmdCommCommand::SaveSettings => "SaveSettings",
            ZedmdCommCommand::Reset => "Reset",
            ZedmdCommCommand::SetClkphase => "SetClkphase",
            ZedmdCommCommand::SetI2sspeed => "SetI2sspeed",
            ZedmdCommCommand::SetLatchBlanking => "SetLatchBlanking",
            ZedmdCommCommand::SetMinRefreshRate => "SetMinRefreshRate",
            ZedmdCommCommand::SetDriver => "SetDriver",
            ZedmdCommCommand::SetTransport => "SetTransport",
            ZedmdCommCommand::SetUdpDelay => "SetUdpDelay",
            ZedmdCommCommand::SetUsbPackageSizeMultiplier => "SetUsbPackageSizeMultiplier",
            ZedmdCommCommand::SetYOffset => "SetYOffset",
            ZedmdCommCommand::RGB565ZonesStream => "RGB565ZonesStream",
            ZedmdCommCommand::RenderRGB565Frame => "RenderRGB565Frame",
            ZedmdCommCommand::ClearScreen => "ClearScreen",
            ZedmdCommCommand::KeepAlive => "KeepAlive",
            ZedmdCommCommand::DisableDebug => "DisableDebug",
            ZedmdCommCommand::EnableDebug => "EnableDebug",
        };
        write!(f, "{} {}", *self as u8, name)
    }
}

// FRAME in ascii
const FRAME_HEADER: [u8;5] =  [0x46, 0x52, 0x41, 0x4d, 0x45];

// "ZeDMD" in ascii
const CTRL_CHARS_HEADER: [u8;5] = [0x5A, 0x65, 0x44, 0x4D, 0x44];


pub fn connect() -> io::Result<ZeDMDComm> {
    let ports = serialport::available_ports().expect("No ports found!");
    for p in ports {
        // skip non-usb ports

        if let UsbPort(info) = p.port_type {
            let port_name = p.port_name.clone();
            debug!("ZeDMD candidate: device={}, vid={:04x}, pid={:04x}", port_name, info.vid, info.pid);

            // connect rw

            let mut port = serialport::new(p.port_name, ZEDMD_COMM_BAUD_RATE)
                .parity(Parity::None)
                .data_bits(DataBits::Eight)
                .stop_bits(StopBits::One)
                .flow_control(serialport::FlowControl::None)
                .timeout(std::time::Duration::from_secs(1))
                .open()
                .expect("Failed to open port");

            let size_high_byte = 0;
            let size_low_byte = 0;
            let compression_flag = 0;

            // write the handshake command by combining the headers and other parameters
            let mut handshake_command = vec![];
            handshake_command.extend_from_slice(&FRAME_HEADER);
            handshake_command.extend_from_slice(&CTRL_CHARS_HEADER);
            handshake_command.push(ZedmdCommCommand::Handshake as u8);
            handshake_command.push(size_high_byte);
            handshake_command.push(size_low_byte);
            handshake_command.push(compression_flag);

            // fill rest of the buffer with 0x00
            let remaining_size = ZEDMD_COMM_DEFAULT_SERIAL_WRITE_AT_ONCE - handshake_command.len();
            if remaining_size > 0 {
                handshake_command.extend(vec![0x00; remaining_size]);
            }

            println_ascii("Handshake buffer", &handshake_command);

            // sleep 200ms to allow the device to be ready
            sleep(Duration::from_millis(200));

            // try the handshake twice
            for i in 0..2 {
                debug!("Attempt {} to send handshake command", i + 1);
                flush_and_discard(&mut port);
                println_ascii("Sending handshake", &handshake_command);
                port.write_all(&handshake_command).expect("Failed to write to port");

                // depending on the device configured block size, we might need to write more bytes
                // clear the buffer
                let zeroes = vec![0; ZEDMD_COMM_DEFAULT_SERIAL_WRITE_AT_ONCE];
                for _ in 1..(ZEDMD_COMM_MAX_SERIAL_WRITE_AT_ONCE / ZEDMD_COMM_DEFAULT_SERIAL_WRITE_AT_ONCE) {
                    println_ascii("Sending zeroes", &zeroes);
                    port.write_all(&zeroes).expect("Failed to write to port");
                    // sleep 10 ms to allow the device to process the command
                    sleep(Duration::from_millis(10));
                }
                //we should ignore results above

                port.flush().expect("Failed to flush port");


                // sleep 200 ms to allow the device to process the command
                sleep(std::time::Duration::from_millis(200));

                // read the response
                debug!("Reading response from port");
                let mut data = vec![0; 64];
                let read_result = port.read(&mut data);
                let n = match read_result {
                    Ok(n) => n,
                    Err(e) => {
                        error!("Failed to read from port: {}", e);
                        continue;
                    }
                };

                info!("Read {} bytes from response", n);

                if n > 0 {
                    // Print the response as hex
                    debug!("Response: {:?}", &data[..n]);
                    // Check if the response starts with the expected header
                    if data.starts_with(&CTRL_CHARS_HEADER[..4]) {
                        if data[57] == 'R' as u8 && data[8] != 0 {
                            let internal = SharedZeDMDComm {
                                width: 0,
                                height: 0,
                                zone_width: 0,
                                zone_height: 0,
                                firmware_version: format!("{}.{}.{}", data[8], data[9], data[10]),
                                write_at_once: (data[11] as u32 + data[12] as u32 * 256) as u32,
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
                                id: (data[23] as u16 + data[24] as u16 * 256),
                                device_name: port_name,
                                port: Some(port),
                                keep_alive: false, // set to true on run
                                last_keep_alive: std::time::Instant::now(),
                                running: false,
                            };



                            let prefix = if internal.s3 { "S3 " } else { "" };
                            info!("ZeDMD {} found: {}device={}, width={}, height={}", internal.firmware_version, prefix, internal.device_name, internal.width, internal.height);
                            //
                            //     // Next streaming needs to be complete.
                            //     memset(m_zoneHashes, 0, sizeof(m_zoneHashes));
                            //
                            //     while (sp_input_waiting(m_pSerialPort) > 0)
                            //     {
                            //         sp_nonblocking_read(m_pSerialPort, data, ZEDMD_COMM_MAX_SERIAL_WRITE_AT_ONCE);
                            //     }

                            if internal.write_at_once <= 64
                            {
                                warn!("The ZeDMD USB package size of {} is a very low value. Try to increase it to get smoother animations.", internal.write_at_once);
                            }
                            if internal.panel_min_refresh_rate <= 30
                            {
                                warn!("The ZeDMD panel minimal refresh rate of {} is a very low value. Try to increase it to get smoother animations.",
                                    internal.panel_min_refresh_rate);
                            }

                            let comm = ZeDMDComm {
                                comm: Arc::new(Mutex::new(internal)),
                            };
                            return Ok(comm)
                        } else {
                            error!("Handshake response error, first 8 bytes of response: {:?}", &data[..8]);
                            continue;
                        }
                    } else {
                        error!("ZeDMD handshake response error - expected header not found, first 8 bytes of response: {:?}", &data[..8]);
                        continue;
                    }
                } else {
                    info!("No response received, retrying...");
                    continue;
                }
            }
        } else {
            // Skip non-USB ports
            continue;
        }
    }
    Err(io::Error::new(io::ErrorKind::NotFound, "No ZeDMD device found"))
}

impl ZeDMDComm {

    pub fn run(&mut self) -> io::Result<()> {
        {
            let mut comm = self.comm.lock().unwrap();
            if comm.running {
                return Err(io::Error::new(io::ErrorKind::Other, "Already running"));
            }
            comm.running = true;
            comm.last_keep_alive = Instant::now();
            comm.keep_alive = true;
        }
        let comm_clone = Arc::clone(&self.comm);

        // Create a thread to handle the communication
        std::thread::spawn(move || {
            debug!("ZeDMDComm run thread starting");
            loop {
                {
                    let mut comm = comm_clone.lock().unwrap();
                    comm.keep_alive().ok();
                    if !comm.running {
                        break;
                    }
                }
                sleep(Duration::from_millis(1));
            }
            debug!("ZeDMDComm run thread finished");
        });

        Ok(())
    }

    pub fn stop(&mut self) -> io::Result<()> {
        let mut comm = self.comm.lock().unwrap();
        if !comm.running {
            return Err(io::Error::new(io::ErrorKind::Other, "Already stopped"));
        }
        comm.running = false;
        if let Some(port) = &mut comm.port {
            port.flush().expect("Failed to flush port");
            //comm.port = None; // clear the port
        }
        info!("ZeDMDComm stopped and port closed");
        Ok(())
    }

    pub(crate) fn reset(&self) -> io::Result<()> {
        let mut comm = self.comm.lock().unwrap();
        comm.reset()
    }

}

impl SharedZeDMDComm {

    pub fn reset(&mut self) -> io::Result<()> {
        if let Some(port) = &mut self.port {
            let command = vec![
                FRAME_HEADER[0], FRAME_HEADER[1], FRAME_HEADER[2], FRAME_HEADER[3], FRAME_HEADER[4],
                CTRL_CHARS_HEADER[0], CTRL_CHARS_HEADER[1], CTRL_CHARS_HEADER[2], CTRL_CHARS_HEADER[3], CTRL_CHARS_HEADER[4],
                ZedmdCommCommand::Reset as u8
            ];
            println_ascii("Reset command", &command);
            port.write_all(&command).expect("Failed to write reset command to port");
            port.flush().expect("Failed to flush port");
            // wait for the device to reset
            sleep(std::time::Duration::from_secs(2));
            // after reset, we need to reconnect
            self.port = None; // clear the port to force reconnection
            info!("Device reset, port cleared for reconnection");
            Ok(())
        } else {
            Err(io::Error::new(io::ErrorKind::NotConnected, "No port connected"))
        }
    }

    pub fn led_test(&mut self) -> io::Result<()> {
        if let Some(port) = &mut self.port {
            let command = vec![
                FRAME_HEADER[0], FRAME_HEADER[1], FRAME_HEADER[2], FRAME_HEADER[3], FRAME_HEADER[4],
                CTRL_CHARS_HEADER[0], CTRL_CHARS_HEADER[1], CTRL_CHARS_HEADER[2], CTRL_CHARS_HEADER[3], CTRL_CHARS_HEADER[4],
                ZedmdCommCommand::LEDTest as u8
            ];
            println_ascii("LED Test command", &command);
            port.write_all(&command).expect("Failed to write LED test command to port");
            port.flush().expect("Failed to flush port");
            Ok(())
        } else {
            Err(io::Error::new(io::ErrorKind::NotConnected, "No port connected"))
        }
    }

    pub fn keep_alive(&mut self) -> io::Result<()> {
        let port = self.port.as_mut().ok_or_else(|| {
            io::Error::new(io::ErrorKind::NotConnected, "No port connected")
        })?;
        let now = Instant::now();
        if !self.keep_alive {
            self.last_keep_alive = now;
            return Ok(());
        }
        if now.duration_since(self.last_keep_alive).as_millis() as usize > ZEDMD_COMM_KEEP_ALIVE_INTERVAL {
            self.last_keep_alive = now;

            let command = FrameBuilder::new_control_frame()
                .command(ZedmdCommCommand::KeepAlive as u8)
                .params(&[0, 0, 0])
                .build();

            println_ascii(&format!("Keep Alive command {}", ZedmdCommCommand::KeepAlive), &command);

            port.write_all(&command).expect("Failed to write keep alive command to port");
            port.flush().expect("Failed to flush port");
        }
        Ok(())
    }

    pub fn enable_keep_alive(&mut self) {
        self.keep_alive = true;
    }

    pub fn disable_keep_alive(&mut self) {
        self.keep_alive = false;
    }
}

fn flush_and_discard(port: &mut Box<dyn SerialPort>) -> Vec<u8> {
    debug!("Flushing and discarding data in the port");
    // flush any existing data in the port
    port.flush().expect("Failed to flush port");
    // read any existing data in the port
    let mut buffer = vec![0; 64];
    while port.bytes_to_read().expect("Failed check available bytes") > 0 {
        match port.read(&mut buffer) {
            Ok(n) => {
                if n > 0 {
                    debug!("Flushing {} bytes: {:?}", n, &buffer[..n]);
                }
            },
            Err(e) => {
                error!("Failed to read from port: {}", e);
                break;
            }
        }
    }
    buffer
}

fn println_ascii(message: &str, data: &Vec<u8>) {
    // print the handshake command as ascii or hex in case of non-ascii characters
    let handshake_command_str: String = data.iter()
        .map(|&b| if b.is_ascii_graphic() {
            format!("{}  ", b as char)
        } else {
            format!("{:02x} ", b)
        })
        .collect();
    debug!("{}: [{}] {}", message, data.len(), handshake_command_str);
}


struct FrameBuilder {
    buffer: Vec<u8>,
}

impl FrameBuilder {
    pub fn new() -> Self {
        Self { buffer: vec![] }
    }
    
    pub fn new_control_frame() -> Self {
        let mut buffer = FRAME_HEADER.to_vec();
        buffer.extend_from_slice(&CTRL_CHARS_HEADER);
        Self { buffer }
    }

    pub fn header(mut self, header: &[u8]) -> Self {
        self.buffer.extend_from_slice(header);
        self
    }

    pub fn command(mut self, cmd: u8) -> Self {
        self.buffer.push(cmd);
        self
    }

    pub fn params(mut self, params: &[u8]) -> Self {
        self.buffer.extend_from_slice(params);
        self
    }

    pub fn pad_to(mut self, size: usize, pad: u8) -> Self {
        if self.buffer.len() < size {
            self.buffer.extend(std::iter::repeat(pad).take(size - self.buffer.len()));
        }
        self
    }

    pub fn build(self) -> Vec<u8> {
        self.buffer
    }
}