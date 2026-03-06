use std::fmt::Display;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
pub enum ZedmdCommCommand {
    FrameSize = 0x02,
    RGB888ZonesStream = 0x04,
    RGB565ZonesStream = 0x05,
    RenderFrame = 0x06,
    RGB888Stream = 0x07,
    RGB565Stream = 0x08,
    ClearScreen = 0x0a,
    KeepAlive = 0x0b,
    Handshake = 0x0c,
    LEDTest = 0x10,
    DisableUpscaling = 0x14,
    EnableUpscaling = 0x15,
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
    DisableDebug = 0x62,
    EnableDebug = 0x63,
}

impl Display for ZedmdCommCommand {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let name = match self {
            ZedmdCommCommand::FrameSize => "FrameSize",
            ZedmdCommCommand::RGB888ZonesStream => "RGB888ZonesStream",
            ZedmdCommCommand::RGB565ZonesStream => "RGB565ZonesStream",
            ZedmdCommCommand::RenderFrame => "RenderFrame",
            ZedmdCommCommand::RGB888Stream => "RGB888Stream",
            ZedmdCommCommand::RGB565Stream => "RGB565Stream",
            ZedmdCommCommand::ClearScreen => "ClearScreen",
            ZedmdCommCommand::KeepAlive => "KeepAlive",
            ZedmdCommCommand::Handshake => "Handshake",
            ZedmdCommCommand::LEDTest => "LEDTest",
            ZedmdCommCommand::DisableUpscaling => "DisableUpscaling",
            ZedmdCommCommand::EnableUpscaling => "EnableUpscaling",
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
            ZedmdCommCommand::DisableDebug => "DisableDebug",
            ZedmdCommCommand::EnableDebug => "EnableDebug",
        };
        write!(f, "0x{:02x} {}", *self as u8, name)
    }
}

/// The 6 possible RGB channel orderings as defined by the ZeDMD firmware.
/// Firmware array (main.cpp):
///   0: RC,GC,BC → RGB
///   1: BC,RC,GC → BRG
///   2: GC,BC,RC → GBR
///   3: RC,BC,GC → RBG
///   4: GC,RC,BC → GRB
///   5: BC,GC,RC → BGR
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RgbOrder {
    Rgb = 0,
    Brg = 1,
    Gbr = 2,
    Rbg = 3,
    Grb = 4,
    Bgr = 5,
}

impl RgbOrder {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Rgb),
            1 => Some(Self::Brg),
            2 => Some(Self::Gbr),
            3 => Some(Self::Rbg),
            4 => Some(Self::Grb),
            5 => Some(Self::Bgr),
            _ => None,
        }
    }
}

impl Display for RgbOrder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            Self::Rgb => "RGB",
            Self::Brg => "BRG",
            Self::Gbr => "GBR",
            Self::Rbg => "RBG",
            Self::Grb => "GRB",
            Self::Bgr => "BGR",
        };
        write!(f, "{}", s)
    }
}
