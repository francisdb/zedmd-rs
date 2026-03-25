# zedmd-rs

A Rust library for controlling ZeDMD dot matrix displays over USB.

Original C++ library:
https://github.com/PPUC/libzedmd

For a detailed description of the USB serial protocol, handshake, command
opcodes, zone streaming and ACK flow see [serial_protocol.md](serial_protocol.md).

## Usage

Use cargo to add the dependency to your project:

```bash
cargo add zedmd
```

## Examples

| Example                                      | Description                                                                                                                              |
|----------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------|
| [`pixel_scan`](examples/pixel_scan.rs)       | Scans a single white pixel across every position — useful for verifying the connection and panel                                        |
| [`display_test`](examples/display_test.rs)   | Cycles through solid colours, gradients, colour bars, checkerboard and static rainbow                                                   |
| [`plasma`](examples/plasma.rs)               | Classic sine-wave colour plasma animation (~9–10 fps USB throughput)                                                                    |
| [`plasma_rgb888`](examples/plasma_rgb888.rs) | Same plasma using true RGB888 (command 0x04) — use this to verify the RGB888 path works on your device                                 |
| [`rings`](examples/rings.rs)                 | Animated concentric colour rings — heavier on the connection than plasma (~6–7 fps USB throughput)                                     |
| [`rainbow`](examples/rainbow.rs)             | Animated rainbow                                                                                                                         |
| [`scaling`](examples/scaling.rs)             | RGB888 plasma at native, 2x downscaled, and 2x upscaled source resolutions — evaluates scaling quality                                  |
| [`settings`](examples/settings.rs)           | Unified settings tool: panel/display settings, WiFi credentials, WiFi UDP delay, and transport mode (USB / WiFi UDP / WiFi TCP / SPI) |

Run any example with:

```bash
cargo run --example plasma
```

## WiFi

ZeDMD devices with WiFi capability can connect to your local network so you can
stream frames over the network instead of USB.

There are two ways to configure WiFi credentials:

### Option 1: Over USB (recommended)

Connect the device via USB and use the unified `settings` example to push SSID,
password, port, power and (optionally) transport mode, then save to flash:

```bash
cargo run --example settings -- \
  --ssid MyNetwork --password MyPassword --port 5000 --save
```

Switch transport to WiFi UDP from the same command:

```bash
cargo run --example settings -- --transport wifi-udp --save
```

Or from code:

```rust
let comm = zedmd_rs::zedmd::connect()?;
comm.set_wifi_ssid("MyNetwork")?;
comm.set_wifi_password("MyPassword")?;
comm.set_wifi_port(5000)?;
comm.set_transport_mode(zedmd_rs::types::TransportMode::WifiUdp)?;
comm.save_settings()?;
```

Reboot the device after saving for new settings to take effect.

> **Important transport behavior**
>
> ZeDMD runs one active transport at a time. After switching to a WiFi transport,
> USB serial access is expected to be unavailable until you switch transport back
> to USB from the device's physical settings controls (buttons/menu). Depending on
> firmware build/version, there may also be a factory-reset or fallback path, but
> that is firmware-specific and not guaranteed by this Rust crate.

### Option 2: Via the device's setup network

When no WiFi is configured (or the configured network is unavailable), the
device broadcasts its own setup network:

| | |
|---|---|
| **SSID** | `ZeDMD-WiFi` |
| **Password** | `zedmd1234` |

Connect to that network and open <http://zedmd-wifi.local> in a browser to
configure target WiFi credentials through the web interface.

## Allowing access to the USB device on Linux

### Option 1: creating a udev rule

You can create an udev rule to allow access to the device. Create a file in `/etc/udev/rules.d/99-zedmd.rules` with the
following content:

```bash
sudo nano /etc/udev/rules.d/99-zedmd.rules
```

```
# ZeDMD dot matrix USB device rules

ACTION!="add|change", GOTO="zedmd_rules_end"

SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="zedmd", MODE="0666" 

LABEL="zedmd_rules_end"
```

*Make sure the vendor and product id match what is listed on `lsusb`*

```bash
lsusb
...
Bus 003 Device 034: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
```

Then reload the udev rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
``` 

*This will also mount the device on /dev/zedmd*

### Option 2: Add your user to the `dialout` group

Alternatively you can give your user access to any serial port. You can do this by adding your user to the `dialout`
group:

```bash
sudo usermod -aG dialout $USER
```

After running the above command, you will need to log out and log back in for the group change to take effect.
