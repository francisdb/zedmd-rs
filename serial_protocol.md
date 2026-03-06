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

After each chunk the host must read exactly 6 bytes from the device before
sending the next chunk. Partial reads must be retried — the full 6-byte ACK
must be received before proceeding.

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
`ZEDMD_ZONES_BYTE_LIMIT_RGB565` (= `zone_w × zone_h × 2 × 16 + 16` = **1040 bytes**
for an 8×4-pixel zone). Each sub-frame is zlib-compressed independently; the
compressed size must not exceed `BUFFER_SIZE` (1152 bytes). If compression does
not help, the chunk is sent uncompressed.

Each sub-frame is an **independent firmware transaction** sent via its own
`send_chunks` call. `RenderFrame` (cmd `0x06`) returns result `1` which causes
the firmware to break out of the inner read loop and wait for a new `FRAME`
header — so each sub-frame must have its own `FRAME` prefix and `RenderFrame`
suffix. Sub-frames are sent in **reverse order** (matching C++ `rbegin/rend`
iteration in `StreamBytes`):

```
Client                                     Device
  │                                          │
  │─ FRAME + ZeDMD + RGB565ZonesStream       │  sub-frame N (last, reversed)
  │    + size_hi + size_lo + compressed      │
  │    + zone_data                           │
  │  + ZeDMD + RenderFrame + 0 + 0 + 0 ────▶│  (padded to usbPackageSize)
  │◀──────────────── ZeDMDA ─────────────────│
  │─ (continuation chunks if payload > usbPackageSize) ▶│
  │◀──────────────── ZeDMDA ─────────────────│  (one ACK per chunk)
  │                                          │
  │─ FRAME + ZeDMD + RGB565ZonesStream       │  sub-frame N-1 …
  │    + … + RenderFrame ───────────────────▶│
  │◀──────────────── ZeDMDA ─────────────────│
  │                                          │  (device renders on each RenderFrame)
```

### Zone-streaming frame (RGB888)

Identical to RGB565 zone streaming but uses command opcode `RGB888ZonesStream`
(`0x04`) and 3 bytes per pixel instead of 2. The theoretical sub-frame byte
limit would be `zone_w × zone_h × 3 × 16 + 16` = **1552 bytes**, but the
firmware's `BUFFER_SIZE` is only **1152 bytes**. Payloads exceeding this trigger
an on-screen "payloadSize > BUFFER_SIZE" error and a serial reset. The Rust
implementation therefore caps `ZEDMD_ZONES_BYTE_LIMIT_RGB888` at **1152** so
sub-frames never exceed the firmware buffer.

The firmware sets an `rgb888ZoneStream` flag when it sees command `0x04`, then
falls through to the same case `0x05` handler. It dispatches to `FillZoneRaw`
(RGB888) vs `FillZoneRaw565` (RGB565) based on that flag.

```
Client                                     Device
  │                                          │
  │─ FRAME + ZeDMD + RGB888ZonesStream       │  sub-frame N (reversed)
  │    + size_hi + size_lo + compressed      │
  │    + zone_data                           │
  │  + ZeDMD + RenderFrame + 0 + 0 + 0 ────▶│
  │◀──────────────── ZeDMDA ─────────────────│
  │─ (more sub-frames, each with FRAME … RenderFrame) ▶│
  │◀──────────────── ZeDMDA ─────────────────│
```

---

## Scaling and Frame Size

Scaling is handled entirely **client-side** in libzedmd — the firmware always
receives a full-panel-sized frame.

`SetFrameSize(width, height)` tells the client the source resolution (e.g. the
ROM's native resolution). On `RenderRgb888` / `RenderRgb565` the client calls
`GetScaleMode` to decide how to transform the source frame to the panel size:

| Scale mode | Condition                                       | Action                                          |
|------------|-------------------------------------------------|-------------------------------------------------|
| `255`      | Source == panel size                            | `memcpy` — no transformation                    |
| `0`        | Source fits inside panel (with optional offset) | **Center** the source in the panel              |
| `1`        | Source is larger than panel                     | **Scale down** (nearest-neighbour)              |
| `2`        | Source is smaller and `upscaling` is enabled    | **Scale up 2×**, then center if panel is larger |

### Upscaling flag

`EnableUpscaling()` / `DisableUpscaling()` (opcodes `0x15` / `0x14`) set an
`m_upscaling` flag in libzedmd. When `false` (default), small sources are
**centred** with black borders. When `true`, they are **pixel-doubled** to fill
the panel.

These opcodes are sent to the device so it can store the preference, but the
actual scaling arithmetic always runs on the host.

### TrueRGB888 flag

`EnableTrueRgb888(true)` sets `m_rgb888 = true` in libzedmd. When `false`
(default), `RenderRgb888` converts the scaled RGB888 frame to RGB565 before
sending (using command `0x05`). When `true`, the raw RGB888 data is sent using
command `0x04`. There is no device-side opcode for this flag — it only changes
which zone-stream command the client emits.

> **Note**: libzedmd's own test suite (`test.cpp`) never calls
> `EnableTrueRgb888` — all `RenderRgb888` calls go through the RGB888 → RGB565
> conversion path (command `0x05`). Command `0x04` (`RGB888ZonesStream`) appears
> to be untested in the reference implementation and does not work reliably in
> practice. The recommended approach is to always use the RGB565 conversion path.
> See also: <https://github.com/PPUC/libzedmd/issues/47>

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

| Parameter                       | Formula                          | Value (128×32 panel) |
|---------------------------------|----------------------------------|----------------------|
| Zone width (pixels)             | `width / 16`                     | 8                    |
| Zone height (pixels)            | `height / 8`                     | 4                    |
| Zones X                         | `width / zone_w`                 | 16                   |
| Zones Y                         | `height / zone_h`                | 8                    |
| Total zones                     | `zones_x × zones_y`              | 128                  |
| Zone bytes (RGB565)             | `zone_w × zone_h × 2`            | 64                   |
| Zone bytes (RGB888)             | `zone_w × zone_h × 3`            | 96                   |
| `ZEDMD_ZONES_BYTE_LIMIT_RGB565` | `zone_w × zone_h × 2 × 16 + 16`  | 1040                 |
| `ZEDMD_ZONES_BYTE_LIMIT_RGB888` | capped at firmware `BUFFER_SIZE` | 1152                 |

> **Zone count correction**: a 128×32 panel has `(128/8) × (32/4)` = 16 × 8 =
> **128 zones**, not 32. The earlier version of this document was wrong.

### Zone encoding

Each changed zone in the sub-frame payload is encoded as:

- **Non-black zone**: `[ zone_index (1 byte) ][ pixels (zone_bytes) ]`
  — `zone_bytes` is 64 for RGB565 (2 bytes/pixel) or 96 for RGB888 (3 bytes/pixel)
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
2. When the buffer exceeds `bufferSizeThreshold` (= `ZEDMD_ZONES_BYTE_LIMIT
   − zone_bytes_total`, where `zone_bytes_total = zone_bytes + 1`), the buffer
   is flushed as a sub-frame and a new one started.
    - RGB565: limit = 1040, threshold = 1040 − 65 = 975
    - RGB888: limit = 1152 (firmware cap), threshold = 1152 − 97 = 1055
3. Each sub-frame is zlib-compressed (level 6). If compression produces a larger
   result, it is sent uncompressed.
4. The compressed (or raw) size must be ≤ `BUFFER_SIZE` (1152 bytes) — the
   firmware sends a NACK and resets the serial port if this is exceeded.
5. Each sub-frame is sent as an independent `send_chunks` call, wrapped as
   `FRAME + ZeDMD + {RGB565|RGB888}ZonesStream + size + compressed_flag + data + ZeDMD + RenderFrame`.
   Sub-frames are sent in **reverse order** (matching C++ `StreamBytes` `rbegin/rend`).

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
