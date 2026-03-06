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

| Example                                      | Description                                                                                            |
|----------------------------------------------|--------------------------------------------------------------------------------------------------------|
| [`pixel_scan`](examples/pixel_scan.rs)       | Scans a single white pixel across every position — useful for verifying the connection and panel       |
| [`display_test`](examples/display_test.rs)   | Cycles through solid colours, gradients, colour bars, checkerboard and static rainbow                  |
| [`plasma`](examples/plasma.rs)               | Classic sine-wave colour plasma animation (~9–10 fps USB throughput)                                   |
| [`plasma_rgb888`](examples/plasma_rgb888.rs) | Same plasma using true RGB888 (command 0x04) — use this to verify the RGB888 path works on your device |
| [`rings`](examples/rings.rs)                 | Animated concentric colour rings — heavier on the connection than plasma (~6–7 fps USB throughput)     |
| [`rainbow`](examples/rainbow.rs)             | Animated rainbow                                                                                       |
| [`scaling`](examples/scaling.rs)             | RGB888 plasma at native, 2× downscaled, and 2× upscaled source resolutions — evaluates scaling quality |
| [`settings`](examples/settings.rs)           | Read and optionally update device settings (brightness, RGB order, USB package size, panel parameters) |

Run any example with:

```bash
cargo run --example plasma
```

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


