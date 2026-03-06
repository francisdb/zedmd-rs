/// Convert 8-bit RGB components to RGB565.
pub fn rgb565(r: u8, g: u8, b: u8) -> u16 {
    ((r as u16 & 0xF8) << 8) | ((g as u16 & 0xFC) << 3) | (b as u16 >> 3)
}

/// Convert HSV (h: 0..360, s: 0..1, v: 0..1) to RGB565.
pub fn hsv_to_rgb565(h: f32, s: f32, v: f32) -> u16 {
    let (r, g, b) = hsv_to_rgb888_components(h, s, v);
    rgb565(r, g, b)
}

/// Convert HSV (h: 0..360, s: 0..1, v: 0..1) to packed RGB888 `(r, g, b)`.
pub fn hsv_to_rgb888(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
    hsv_to_rgb888_components(h, s, v)
}

fn hsv_to_rgb888_components(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
    let h = h.rem_euclid(360.0);
    let c = v * s;
    let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
    let m = v - c;
    let (r1, g1, b1) = match h as u32 {
        0..=59 => (c, x, 0.0),
        60..=119 => (x, c, 0.0),
        120..=179 => (0.0, c, x),
        180..=239 => (0.0, x, c),
        240..=299 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    (
        ((r1 + m) * 255.0) as u8,
        ((g1 + m) * 255.0) as u8,
        ((b1 + m) * 255.0) as u8,
    )
}
