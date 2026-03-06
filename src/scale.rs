//! Client-side scaling — mirrors `ZeDMD::GetScaleMode` / `Scale888` / `Scale565` from libzedmd.
//!
//! The firmware always receives a full-panel-sized frame. All scaling is done
//! here on the host before the pixels reach the zone-streaming layer.
//!
//! # Scaling quality
//!
//! The current downscale is a fixed 2× majority-vote halve (matching libzedmd).
//! This works reasonably well for solid shapes but breaks for thin features such
//! as single-pixel-wide line fonts — a 1px horizontal or vertical line can
//! disappear entirely when it falls on an odd row/column and loses the
//! majority vote against three black neighbours.
//!
//! TODO: evaluate alternative downscaling strategies for these cases:
//! - **Area-average (box filter)** — averages the 2×2 block; preserves thin
//!   lines better by blending rather than discarding, at the cost of blurriness.
//! - **Max-value** — picks the brightest pixel in each 2×2 block; keeps thin
//!   bright features visible but can cause blooming on dense content.
//! - **Lanczos / bilinear** — higher quality for photographic content but much
//!   more expensive and unlikely to help for 1px font strokes.
//!
//! Note that none of these can fully recover a 1px font after a 2× downscale
//! without information loss — the real fix is to use a font sized for the
//! target panel resolution.

/// Scale mode returned by [`get_scale_mode`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ScaleMode {
    /// Source size matches panel — copy as-is.
    None,
    /// Source fits inside panel — centre with black borders.
    Center,
    /// Source is larger than panel — scale down (nearest-neighbour).
    ScaleDown,
    /// Source is smaller and upscaling is enabled — pixel-double then centre.
    ScaleUp,
}

/// Decide how to map a `rom_w × rom_h` source frame onto a `panel_w × panel_h` panel.
/// Matches `ZeDMD::GetScaleMode` exactly.
pub(crate) fn get_scale_mode(
    rom_w: u32,
    rom_h: u32,
    panel_w: u32,
    panel_h: u32,
    upscaling: bool,
) -> ScaleMode {
    if rom_w == panel_w && rom_h == panel_h {
        return ScaleMode::None;
    }
    if rom_w == 192 && panel_w == 256 {
        // 192-wide centred in 256-wide panel (xOffset=32 in libzedmd, handled inside ScaleDown)
        return ScaleMode::Center;
    }
    if rom_w == 192 && panel_w == 128 {
        // 192-wide halved to 96-wide, then centred in 128-wide panel
        // libzedmd returns mode 1 (ScaleDown) here, not Center
        return ScaleMode::ScaleDown;
    }
    if rom_h == 16 && panel_h == 32 {
        return ScaleMode::Center;
    }
    if rom_h == 16 && panel_h == 64 {
        return if upscaling {
            ScaleMode::ScaleUp
        } else {
            ScaleMode::Center
        };
    }
    if rom_w == 256 && panel_w == 128 {
        return ScaleMode::ScaleDown;
    }
    if rom_w == 128 && panel_w == 256 {
        return if upscaling {
            ScaleMode::ScaleUp
        } else {
            ScaleMode::Center
        };
    }
    // Fallback: copy as-is (mismatched sizes not covered above)
    ScaleMode::None
}

/// Scale an RGB565 source frame (u16 pixels, host byte order) to panel size.
/// Returns a new vec of `panel_w * panel_h` pixels.
pub(crate) fn scale_rgb565(
    src: &[u16],
    rom_w: u32,
    rom_h: u32,
    panel_w: u32,
    panel_h: u32,
    upscaling: bool,
) -> Vec<u16> {
    let mode = get_scale_mode(rom_w, rom_h, panel_w, panel_h, upscaling);
    let panel_pixels = (panel_w * panel_h) as usize;

    match mode {
        ScaleMode::None => src.to_vec(),
        ScaleMode::Center => {
            let mut dst = vec![0u16; panel_pixels];
            center_rgb565(&mut dst, panel_w, panel_h, src, rom_w, rom_h);
            dst
        }
        ScaleMode::ScaleDown => {
            let mut dst = vec![0u16; panel_pixels];
            scale_down_rgb565(&mut dst, panel_w, panel_h, src, rom_w, rom_h);
            dst
        }
        ScaleMode::ScaleUp => {
            // Pixel-double first
            let up_w = rom_w * 2;
            let up_h = rom_h * 2;
            let mut up = vec![0u16; (up_w * up_h) as usize];
            scale_up_rgb565(&mut up, src, rom_w, rom_h);
            // Then centre if the panel is larger than the doubled size
            if panel_w > up_w || panel_h > up_h {
                let mut dst = vec![0u16; panel_pixels];
                center_rgb565(&mut dst, panel_w, panel_h, &up, up_w, up_h);
                dst
            } else {
                up
            }
        }
    }
}

/// Scale an RGB888 source frame (packed `[r, g, b, r, g, b, …]` bytes) to panel size.
/// Returns a new `Vec<u8>` of `panel_w * panel_h * 3` bytes.
pub(crate) fn scale_rgb888(
    src: &[u8],
    rom_w: u32,
    rom_h: u32,
    panel_w: u32,
    panel_h: u32,
    upscaling: bool,
) -> Vec<u8> {
    let mode = get_scale_mode(rom_w, rom_h, panel_w, panel_h, upscaling);
    let panel_bytes = (panel_w * panel_h * 3) as usize;

    match mode {
        ScaleMode::None => src.to_vec(),
        ScaleMode::Center => {
            let mut dst = vec![0u8; panel_bytes];
            center_rgb888(&mut dst, panel_w, panel_h, src, rom_w, rom_h);
            dst
        }
        ScaleMode::ScaleDown => {
            let mut dst = vec![0u8; panel_bytes];
            scale_down_rgb888(&mut dst, panel_w, panel_h, src, rom_w, rom_h);
            dst
        }
        ScaleMode::ScaleUp => {
            let up_w = rom_w * 2;
            let up_h = rom_h * 2;
            let mut up = vec![0u8; (up_w * up_h * 3) as usize];
            scale_up_rgb888(&mut up, src, rom_w, rom_h);
            if panel_w > up_w || panel_h > up_h {
                let mut dst = vec![0u8; panel_bytes];
                center_rgb888(&mut dst, panel_w, panel_h, &up, up_w, up_h);
                dst
            } else {
                up
            }
        }
    }
}

// ── RGB565 primitives ────────────────────────────────────────────────────────

fn center_rgb565(dst: &mut [u16], dw: u32, dh: u32, src: &[u16], sw: u32, sh: u32) {
    let x_off = ((dw.saturating_sub(sw)) / 2) as usize;
    let y_off = ((dh.saturating_sub(sh)) / 2) as usize;
    let copy_w = sw.min(dw) as usize;
    let copy_h = sh.min(dh) as usize;
    for y in 0..copy_h {
        let src_row = &src[y * sw as usize..y * sw as usize + copy_w];
        let dst_start = (y + y_off) * dw as usize + x_off;
        dst[dst_start..dst_start + copy_w].copy_from_slice(src_row);
    }
}

fn scale_down_rgb565(dst: &mut [u16], dw: u32, dh: u32, src: &[u16], sw: u32, sh: u32) {
    // Matches libzedmd FrameUtil::Helper::ScaleDown — always a fixed 2× halve.
    // The source is assumed to be exactly 2× the destination in each dimension.
    // Each 2×2 block of source pixels is reduced to one destination pixel using
    // a majority-vote: pick the pixel that appears most often in the 2×2 block,
    // falling back to upper-left on a four-way tie. The result is centred in the
    // destination with black borders.
    //
    // TODO: for non-2× ratios (e.g. 192→128) this is still nearest-neighbour.
    // libzedmd only ever calls ScaleDown for exact 2× cases; for 192→128 the
    // GetScaleMode logic halves to 96 and then centres, so the majority-vote
    // path is still correct there.
    let half_w = sw / 2;
    let half_h = sh / 2;
    let x_off = dw.saturating_sub(half_w) / 2;
    let y_off = dh.saturating_sub(half_h) / 2;

    for sy in (0..sh as usize).step_by(2) {
        for sx in (0..sw as usize).step_by(2) {
            let ul = src[sy * sw as usize + sx];
            let ur = src[sy * sw as usize + sx + 1];
            let ll = src[(sy + 1) * sw as usize + sx];
            let lr = src[(sy + 1) * sw as usize + sx + 1];

            let px = majority_vote_u16(ul, ur, ll, lr);

            let dx = sx / 2 + x_off as usize;
            let dy = sy / 2 + y_off as usize;
            if dx < dw as usize && dy < dh as usize {
                dst[dy * dw as usize + dx] = px;
            }
        }
    }
}

/// Pick the most common of four values; on tie favour `a` (upper-left).
#[inline]
fn majority_vote_u16(a: u16, b: u16, c: u16, d: u16) -> u16 {
    if a == b || a == c || a == d {
        return a;
    }
    if b == c || b == d {
        return b;
    }
    if c == d {
        return c;
    }
    a // four-way tie — upper-left
}

fn scale_down_rgb888(dst: &mut [u8], dw: u32, dh: u32, src: &[u8], sw: u32, sh: u32) {
    // Same fixed 2× halving as scale_down_rgb565, but for RGB888.
    // TODO: non-2× ratios still fall back to nearest-neighbour (see scale_down_rgb565).
    let half_w = sw / 2;
    let half_h = sh / 2;
    let x_off = dw.saturating_sub(half_w) / 2;
    let y_off = dh.saturating_sub(half_h) / 2;

    for sy in (0..sh as usize).step_by(2) {
        for sx in (0..sw as usize).step_by(2) {
            let ul = &src[(sy * sw as usize + sx) * 3..(sy * sw as usize + sx) * 3 + 3];
            let ur = &src[(sy * sw as usize + sx + 1) * 3..(sy * sw as usize + sx + 1) * 3 + 3];
            let ll = &src[((sy + 1) * sw as usize + sx) * 3..((sy + 1) * sw as usize + sx) * 3 + 3];
            let lr = &src
                [((sy + 1) * sw as usize + sx + 1) * 3..((sy + 1) * sw as usize + sx + 1) * 3 + 3];

            let px = majority_vote_rgb888(ul, ur, ll, lr);

            let dx = sx / 2 + x_off as usize;
            let dy = sy / 2 + y_off as usize;
            if dx < dw as usize && dy < dh as usize {
                let di = (dy * dw as usize + dx) * 3;
                dst[di..di + 3].copy_from_slice(px);
            }
        }
    }
}

#[inline]
fn majority_vote_rgb888<'a>(a: &'a [u8], b: &'a [u8], c: &'a [u8], d: &'a [u8]) -> &'a [u8] {
    if a == b || a == c || a == d {
        return a;
    }
    if b == c || b == d {
        return b;
    }
    if c == d {
        return c;
    }
    a
}

fn scale_up_rgb565(dst: &mut [u16], src: &[u16], sw: u32, sh: u32) {
    // Pixel-double: each source pixel becomes a 2×2 block.
    let dw = sw * 2;
    for sy in 0..sh as usize {
        for sx in 0..sw as usize {
            let px = src[sy * sw as usize + sx];
            let dy = sy * 2;
            let dx = sx * 2;
            dst[dy * dw as usize + dx] = px;
            dst[dy * dw as usize + dx + 1] = px;
            dst[(dy + 1) * dw as usize + dx] = px;
            dst[(dy + 1) * dw as usize + dx + 1] = px;
        }
    }
}

// ── RGB888 primitives ────────────────────────────────────────────────────────

fn center_rgb888(dst: &mut [u8], dw: u32, dh: u32, src: &[u8], sw: u32, sh: u32) {
    let x_off = ((dw.saturating_sub(sw)) / 2) as usize;
    let y_off = ((dh.saturating_sub(sh)) / 2) as usize;
    let copy_w = sw.min(dw) as usize;
    let copy_h = sh.min(dh) as usize;
    for y in 0..copy_h {
        let src_start = y * sw as usize * 3;
        let dst_start = ((y + y_off) * dw as usize + x_off) * 3;
        dst[dst_start..dst_start + copy_w * 3]
            .copy_from_slice(&src[src_start..src_start + copy_w * 3]);
    }
}

fn scale_up_rgb888(dst: &mut [u8], src: &[u8], sw: u32, sh: u32) {
    let dw = sw * 2;
    for sy in 0..sh as usize {
        for sx in 0..sw as usize {
            let si = (sy * sw as usize + sx) * 3;
            let pixel = [src[si], src[si + 1], src[si + 2]];
            let dy = sy * 2;
            let dx = sx * 2;
            for (oy, ox) in [(0, 0), (0, 1), (1, 0), (1, 1)] {
                let di = ((dy + oy) * dw as usize + (dx + ox)) * 3;
                dst[di..di + 3].copy_from_slice(&pixel);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scale_mode_identity() {
        assert_eq!(get_scale_mode(128, 32, 128, 32, false), ScaleMode::None);
        assert_eq!(get_scale_mode(256, 64, 256, 64, false), ScaleMode::None);
    }

    #[test]
    fn test_scale_mode_upscale() {
        assert_eq!(get_scale_mode(128, 32, 256, 64, false), ScaleMode::Center);
        assert_eq!(get_scale_mode(128, 32, 256, 64, true), ScaleMode::ScaleUp);
    }

    #[test]
    fn test_scale_mode_downscale() {
        assert_eq!(
            get_scale_mode(256, 64, 128, 32, false),
            ScaleMode::ScaleDown
        );
        // 192x64 on a 128x32 panel also uses ScaleDown (halve to 96x32, centre)
        assert_eq!(
            get_scale_mode(192, 64, 128, 32, false),
            ScaleMode::ScaleDown
        );
    }

    #[test]
    fn test_scale_mode_192_on_256_panel() {
        // 192-wide on 256-wide panel → centre
        assert_eq!(get_scale_mode(192, 64, 256, 64, false), ScaleMode::Center);
    }

    #[test]
    fn test_downscale_192x64_to_128x32() {
        // 192x64 all-white source scaled to 128x32 panel.
        // The 2x halve produces a 96x32 image, centred in the 128x32 destination
        // with (128-96)/2 = 16px black borders on each side.
        let src = vec![0xFFFFu16; 192 * 64];
        let result = scale_rgb565(&src, 192, 64, 128, 32, false);
        assert_eq!(result.len(), 128 * 32);
        // Left border (col 0..15) should be black
        assert_eq!(result[0], 0, "left border should be black");
        assert_eq!(result[15], 0, "left border should be black");
        // Active area starts at col 16
        assert_eq!(result[16], 0xFFFF, "active area should be white");
        // Right border (col 112..127) should be black
        assert_eq!(result[127], 0, "right border should be black");
        assert_eq!(result[112], 0, "right border should be black");
        // Active area ends at col 111
        assert_eq!(result[111], 0xFFFF, "active area should be white");
    }

    #[test]
    fn test_downscale_256x64_to_128x32() {
        // 256x64 all-white source scaled to 128x32 — no black border expected
        let src = vec![0xFFFFu16; 256 * 64];
        let result = scale_rgb565(&src, 256, 64, 128, 32, false);
        assert_eq!(result.len(), 128 * 32);
        assert!(result.iter().all(|&p| p == 0xFFFF));
    }

    #[test]
    fn test_center_128x32_into_256x64() {
        // 128x32 source centred in a 256x64 panel (no upscaling)
        let src = vec![0xFFFFu16; 128 * 32];
        let result = scale_rgb565(&src, 128, 32, 256, 64, false);
        assert_eq!(result.len(), 256 * 64);
        // Top-left corner should be black (border)
        assert_eq!(result[0], 0);
        // Centre pixel (row 16, col 64) should be white
        assert_eq!(result[16 * 256 + 64], 0xFFFF);
    }

    #[test]
    fn test_upscale_128x32_into_256x64() {
        // 128x32 source pixel-doubled into 256x64
        let src = vec![0xFFFFu16; 128 * 32];
        let result = scale_rgb565(&src, 128, 32, 256, 64, true);
        assert_eq!(result.len(), 256 * 64);
        // All pixels should be white after 2× scale
        assert!(result.iter().all(|&p| p == 0xFFFF));
    }
}
