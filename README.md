# zedmd-rs

A Rust library for controlling ZeDMD dot matrix displays over usb.

## Usage

Use cargo to add the dependency to your project:

```bash
cargo add zedmd
```


## Allowing access to the USB device on Linux

### Option 1: creating a udev rule

You can create an udev rule to allow access to the device. Create a file in `/etc/udev/rules.d/99-zedmd.rules` with the following content:

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

Alternatively you can give your user access to any serial port. You can do this by adding your user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
```

After running the above command, you will need to log out and log back in for the group change to take effect.


