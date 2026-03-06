# ZeDMD Serial Protocol

Derived from reading `ZeDMD/src/transports/usb_transport.cpp`, `ZeDMD/src/main.cpp`,
`ZeDMD/src/main.h`, and the libzedmd C++ client.

---

## Constants

| Name                       | Value               | Description                                                  |
|----------------------------|---------------------|--------------------------------------------------------------|
| `FRAME`                    | `46 52 41 4D 45`    | Packet framing header (`"FRAME"` in ASCII)                   |
| `CtrlChars`                | `5A 65 44 4D 44 41` | Command/ACK prefix (`"ZeDMDA"` in ASCII)                     |
| `N_FRAME_CHARS`            | 5                   | Length of `FRAME` header                                     |
| `N_CTRL_CHARS`             | 5                   | Length of the `ZeDMD` part of `CtrlChars`                    |
| `N_ACK_CHARS`              | 6                   | Length of a full ACK (`N_CTRL_CHARS + 1`)                    |
| `N_INTERMEDIATE_CTR_CHARS` | 4                   | Bytes of `CtrlChars` copied into handshake response (`ZeDM`) |
| `SERIAL_BAUD`              | 921600              | Baud rate for UART devices (CP210x); CDC ignores baud rate   |
| `usbPackageSize`           | 32–1920             | Negotiated packet size (multiple of 32; default 64)          |
| `BUFFER_SIZE`              | 1152                | Firmware receive buffer per command (compile-time constant)  |

---

## USB Transport — Packet Framing

The firmware's USB transport loop (`usb_transport.cpp`) works as follows:

1. **Wait for `FRAME`** — the firmware scans incoming bytes byte-by-byte until it
   sees the 5-byte sequence `46 52 41 4D 45`. Everything before it is discarded.

2. **Read one packet** — after detecting `FRAME`, the firmware reads exactly
   `usbPackageSize - 5` more bytes (so the total consumed is `usbPackageSize`).

3. **Process** — the payload bytes are passed to `HandleData()`.

4. **Send ACK** — after processing, the firmware writes `CtrlChars` (6 bytes,
   `ZeDMDA`) back to the host.

5. **Loop** — if the command returned `1` (complete), go back to step 1 and wait
   for the next `FRAME`. If it returned `0` (more data expected for a multi-packet
   payload), stay in the inner loop and read another `usbPackageSize` bytes without
   requiring a new `FRAME` header, then ACK again.

### Client packet layout

The client builds a contiguous byte buffer starting with `FRAME`, followed by
the full command payload, and passes it to `send_chunks`. The buffer is split
into `usbPackageSize`-byte chunks, each zero-padded:

```
Chunk 1  (usbPackageSize bytes):
  [ FRAME (5) ][ payload bytes … ][ zero padding ]

Chunk N  (usbPackageSize bytes, continuation):
  [ payload bytes … ][ zero padding ]
```

`FRAME` appears only once — at the very start of chunk 1 — because it is the
first 5 bytes of the data buffer. Continuation chunks contain raw payload data
with no prefix. Every chunk receives exactly one `ZeDMDA` ACK.

### ACK reading

The host reads the 6-byte ACK using a manual `read()` loop rather than
`read_exact()`. This is important: if a `read()` call times out mid-ACK (having
received e.g. 4 of 6 bytes), those bytes are **not** discarded — the loop
retries until all 6 arrive. Using `read_exact()` would silently drop partial
reads on timeout, permanently desyncing the ACK stream.

---

## Command Framing (inside the packet payload)

Inside the payload the client embeds one or more commands. Each command is
framed with the first 5 bytes of `CtrlChars` (`ZeDMD`):

```
[ ZeDMD (5) ][ cmd (1) ][ size_hi (1) ][ size_lo (1) ][ compressed (1) ][ data (size) ]
```

| Field        | Size   | Description                                  |
|--------------|--------|----------------------------------------------|
| `ZeDMD`      | 5      | `5A 65 44 4D 44` — command marker            |
| `cmd`        | 1      | Command opcode (see table below)             |
| `size_hi`    | 1      | High byte of payload data length             |
| `size_lo`    | 1      | Low byte of payload data length              |
| `compressed` | 1      | `1` = zlib-compressed payload, `0` = raw     |
| `data`       | `size` | Command-specific payload (compressed or raw) |

The declared `size` is the length of `data` as transmitted (i.e. the compressed
size when `compressed=1`). The firmware checks `size > BUFFER_SIZE` and sends a
NACK if it would overflow its receive buffer.

The firmware scans for the 5-byte `ZeDMD` marker in `HandleData()`, then
dispatches on `cmd`.

---

## Handshake

The handshake uses a fixed 64-byte (`ZEDMD_COMM_DEFAULT_SERIAL_WRITE_AT_ONCE`)
buffer written directly to the port — **not** via `send_chunks`.

After sending the handshake the client sends 29 additional zero-filled 64-byte
buffers (total 1920 bytes = `ZEDMD_COMM_MAX_SERIAL_WRITE_AT_ONCE`) to flush any
stale data from the device's receive buffer. The device ignores zero buffers as
they contain no `FRAME` header.

Up to 2 attempts are made. The first attempt commonly fails because stale bytes
from a previous session are still in the OS serial buffer; this is expected and
logged at `DEBUG` level only. The second attempt succeeds after the buffer is
drained.

### Client → Device

```
Offset  Bytes   Content
0       5       FRAME  (46 52 41 4D 45)
5       5       ZeDMD  (5A 65 44 4D 44)
10      1       cmd = 0x0C  (Handshake)
11      1       size_hi = 0
12      1       size_lo = 0
13      1       compressed = 0
14..63  50      zero padding
```

### Device → Client

The device responds with a 58-byte binary buffer (no ACK is sent for the
handshake):

```
Offset  Bytes   Content
0       4       "ZeDM"  (first 4 bytes of CtrlChars)
4       2       TOTAL_WIDTH  (little-endian u16)
6       2       TOTAL_HEIGHT (little-endian u16)
8       1       ZEDMD_VERSION_MAJOR  (non-zero; used as validity check)
9       1       ZEDMD_VERSION_MINOR
10      1       ZEDMD_VERSION_PATCH
11      2       usbPackageSize = usbPackageSizeMultiplier × 32  (little-endian u16)
13      1       brightness  (0–255)
14      1       rgbMode  (0–5, see RGB Order section)
15      1       yOffset
16      1       panelClkphase
17      1       panelDriver
18      1       panelI2sspeed  (clamped to min 8 by client)
19      1       panelLatchBlanking
20      1       panelMinRefreshRate  (clamped to min 30 by client)
21      1       udpDelay
22      1       flags  (bit 0 = half panel, bit 1 = S3 or Pico)
23      2       shortId  (little-endian u16)
25      1       mcuType  (0=ESP32, 1=ESP32-S3, 2=S3+RM67162, 3=RP2350, 4=RP2040)
26..56  31      reserved (zeros)
57      1       'R'  (0x52, response marker)
```

Validation: `n >= 58`, `data[0..4] == "ZeDM"`, `data[57] == 'R'`, `data[8] != 0`.

---

## ACK

After each `usbPackageSize` chunk the device sends exactly 6 bytes:

```
5A 65 44 4D 44 41   →   "ZeDMDA"   (ACK)
5A 65 44 4D 44 46   →   "ZeDMDF"   (NACK — device also resets its serial port)
```

The handshake does **not** produce an ACK — its binary response buffer serves as
the reply.

`SaveSettings` (cmd `0x1E`) is a special case: the device sends its ACK
**before** the flash write completes (result code `3` in the firmware), so the
client is not blocked by flash latency.

---

## Typical Command Flow

### Simple command (e.g. ClearScreen, KeepAlive)

```
Client                                     Device
  │                                          │
  │─ FRAME + ZeDMD + cmd + 0 + 0 + 0 ──────▶│  (padded to usbPackageSize)
  │◀──────────────── ZeDMDA ─────────────────│
```

### Single-byte setting (e.g. SetBrightness, SetRGBOrder)

```
Client                                     Device
  │                                          │
  │─ FRAME + ZeDMD + cmd + 0 + 1 + 0 + val ▶│  (padded)
  │◀──────────────── ZeDMDA ─────────────────│
```

### Zone-streaming frame (RGB565)

Changed zones are collected, raw zone data is split into sub-frames at
`ZEDMD_ZONES_BYTE_LIMIT_RGB565` (= `zone_w × zone_h × 2 × 16 + 16`, typically
**1040 bytes** for an 8×4-pixel zone). Each sub-frame is zlib-compressed
independently; the compressed size must not exceed `BUFFER_SIZE` (1152 bytes).
If compression does not help, the chunk is sent uncompressed.

Each sub-frame is sent as a single `send_chunks` call containing both the
`RGB565ZonesStream` command and the `RenderFrame` command back-to-back:

```
Client                                     Device
  │                                          │
  │─ FRAME + ZeDMD + RGB565ZonesStream       │
  │    + size_hi + size_lo + compressed      │
  │    + zone_data (compressed or raw)       │
  │  + ZeDMD + RenderFrame + 0 + 0 + 0 ────▶│  chunk 1
  │◀──────────────── ZeDMDA ─────────────────│
  │─ (continuation chunks if payload > usbPackageSize) ▶│
  │◀──────────────── ZeDMDA ─────────────────│  (one per chunk)
  │                                          │  (device renders on RenderFrame)
  │─ (repeat for each sub-frame) ───────────▶│
```

---

## Command Opcodes

| Opcode                        | Hex    | Description                                                               |
|-------------------------------|--------|---------------------------------------------------------------------------|
| `FrameSize`                   | `0x02` | Set frame dimensions                                                      |
| `RGB888ZonesStream`           | `0x04` | Stream changed zones (RGB888)                                             |
| `RGB565ZonesStream`           | `0x05` | Stream changed zones (RGB565)                                             |
| `RenderFrame`                 | `0x06` | Commit buffered zones to display                                          |
| `RGB888Stream`                | `0x07` | Full-frame RGB888 stream                                                  |
| `RGB565Stream`                | `0x08` | Full-frame RGB565 stream                                                  |
| `ClearScreen`                 | `0x0A` | Clear the display                                                         |
| `KeepAlive`                   | `0x0B` | Keep-alive ping                                                           |
| `Handshake`                   | `0x0C` | Initiate handshake                                                        |
| `LEDTest`                     | `0x10` | Run built-in LED test, then restart                                       |
| `DisableUpscaling`            | `0x14` | Disable content upscaling                                                 |
| `EnableUpscaling`             | `0x15` | Enable content upscaling                                                  |
| `Brightness`                  | `0x16` | Set brightness (0–255)                                                    |
| `RGBOrder`                    | `0x17` | Set RGB channel order (0–5)                                               |
| `SetWiFiPower`                | `0x1A` | Set WiFi TX power                                                         |
| `SetWiFiSSID`                 | `0x1B` | Set WiFi SSID                                                             |
| `SetWiFiPassword`             | `0x1C` | Set WiFi password                                                         |
| `SetWiFiPort`                 | `0x1D` | Set WiFi UDP/TCP port                                                     |
| `SaveSettings`                | `0x1E` | Save all settings to flash (ACK sent before write completes)              |
| `Reset`                       | `0x1F` | Soft reset / restart                                                      |
| `SetClkphase`                 | `0x28` | Set panel clock phase                                                     |
| `SetI2sspeed`                 | `0x29` | Set panel I2S speed (min 8)                                               |
| `SetLatchBlanking`            | `0x2A` | Set panel latch blanking                                                  |
| `SetMinRefreshRate`           | `0x2B` | Set panel PWM refresh rate (min 30 Hz; see note below)                    |
| `SetDriver`                   | `0x2C` | Set panel driver chip                                                     |
| `SetTransport`                | `0x2D` | Switch transport (USB / WiFi)                                             |
| `SetUdpDelay`                 | `0x2E` | Set UDP inter-packet delay                                                |
| `SetUsbPackageSizeMultiplier` | `0x2F` | Set USB package size = multiplier × 32                                    |
| `SetYOffset`                  | `0x30` | Set vertical display offset                                               |
| `DisableDebug`                | `0x62` | Disable firmware debug output (session only, use SaveSettings to persist) |
| `EnableDebug`                 | `0x63` | Enable firmware debug output (session only)                               |

> **`SetMinRefreshRate` note** — this sets `mxconfig.min_refresh_rate` in the
> ESP32-HUB75-MatrixPanel-DMA library, which controls the **internal PWM scan
> rate of the LED matrix hardware**. It has no effect on how fast the host sends
> frames. Higher values reduce flicker and improve colour depth at the cost of
> more ESP32 CPU/DMA load. Recommended minimum: 60 Hz.

---

## RGB Order

The `rgbMode` value (0–5) selects which physical LED matrix pin receives each
colour component:

| Mode | Name    | R pin | G pin | B pin |
|------|---------|-------|-------|-------|
| 0    | **RGB** | RC    | GC    | BC    |
| 1    | **BRG** | BC    | RC    | GC    |
| 2    | **GBR** | GC    | BC    | RC    |
| 3    | **RBG** | RC    | BC    | GC    |
| 4    | **GRB** | GC    | RC    | BC    |
| 5    | **BGR** | BC    | GC    | RC    |

---

## Zone Layout

Frames are divided into fixed-size zones for delta encoding. Only zones whose
content has changed since the last frame are transmitted.

| Parameter                       | Formula                         | Value (128×32 panel) |
|---------------------------------|---------------------------------|----------------------|
| Zone width (pixels)             | `width / 16`                    | 8                    |
| Zone height (pixels)            | `height / 8`                    | 4                    |
| Zones X                         | `width / zone_w`                | 16                   |
| Zones Y                         | `height / zone_h`               | 8                    |
| Total zones                     | `zones_x × zones_y`             | 128                  |
| Zone bytes (RGB565)             | `zone_w × zone_h × 2`           | 64                   |
| `ZEDMD_ZONES_BYTE_LIMIT_RGB565` | `zone_w × zone_h × 2 × 16 + 16` | 1040                 |

> **Zone count correction**: a 128×32 panel has `(128/8) × (32/4)` = 16 × 8 =
> **128 zones**, not 32. The earlier version of this document was wrong.

### Zone encoding

Each changed zone in the sub-frame payload is encoded as:

- **Non-black zone**: `[ zone_index (1 byte) ][ RGB565 pixels (zone_bytes) ]`
- **All-black zone**: `[ zone_index + 128 (1 byte) ]` — no pixel data

Zone indices are assigned left-to-right, top-to-bottom.

### Zone hashing

To avoid retransmitting unchanged zones the client maintains a hash per zone:

- Hash `1` — canonical value for an all-black zone
- Hash `0` — unknown / never sent (forces transmission on next frame)
- Any other value — FNV-1a 64-bit hash of the raw zone pixel bytes

libzedmd uses `komihash`; this Rust implementation uses FNV-1a 64-bit, which is
functionally equivalent for this purpose.

### Sub-frame splitting and compression

1. Raw zone data is accumulated into a buffer.
2. When the buffer exceeds `bufferSizeThreshold` (= `ZEDMD_ZONES_BYTE_LIMIT_RGB565
   − zone_bytes_total`), the buffer is flushed as a sub-frame and a new one started.
3. Each sub-frame is zlib-compressed (level 6). If compression produces a larger
   result, it is sent uncompressed.
4. The compressed (or raw) size must be ≤ `BUFFER_SIZE` (1152 bytes) — the
   firmware sends a NACK and resets the serial port if this is exceeded.
5. Each sub-frame payload is wrapped: `ZeDMD + RGB565ZonesStream + size + compressed_flag + data + ZeDMD + RenderFrame`.

---

## WiFi Protocol

The WiFi transport is fundamentally different from USB:

- **No FRAME/packet framing** — data is delivered directly to `HandleData()`
  without the `FRAME`-header scanning loop.
- **No ACKs** — the firmware does not write `ZeDMDA` back for WiFi commands;
  `isWifiAndActive()` causes the ACK path to be skipped.
- **Two modes**: UDP (default) and TCP.
    - **UDP**: each datagram is passed directly to `HandleData()`. No ordering
      guarantees; dropped packets are silently skipped.
    - **TCP**: data arrives via `AsyncClient::onData`; the TCP stack handles
      ordering and retransmission.
- **Handshake**: HTTP GET `/handshake` on the built-in web server (port 80),
  returning a plain-text `\t`-separated string rather than the binary 58-byte
  USB response.
- **Configuration**: WiFi credentials, port, and power are set via the serial
  commands `SetWiFiSSID`, `SetWiFiPassword`, `SetWiFiPort`, `SetWiFiPower`, or
  through the built-in web UI at `http://zedmd-wifi.local`.

The same `HandleData()` / command opcode table is shared between USB and WiFi,
so all commands work identically once data reaches the parser.
