pub mod pwm;
// pub mod ws2812;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use esp_hal::gpio::Output;
use palette::{FromColor, Hsv, Srgb};
use sticky_signal::StickySignal;

/// LED PWM state representing the current/target values for all channels
#[derive(Clone, Copy, Debug, defmt::Format, PartialEq)]
pub struct LedPwmState {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub cw: u8,
    pub ww: u8,
}

/// Semantic device state - tracks high-level user intent
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct DeviceState {
    /// Whether the light is on or off
    pub on: bool,
    /// Brightness level 0-254 (Matter Level Control range, 1-254 = on, 0 = previous level)
    pub brightness: u8,
    /// Color temperature in mireds (153-500, lower = cooler)
    pub color_temp_mireds: u16,
    /// Hue value 0-254
    pub hue: u8,
    /// Saturation value 0-254
    pub saturation: u8,
    /// Color mode (true = using color temp, false = using HSV)
    pub use_color_temp: bool,
    /// Max brightness override - when true, all 5 channels drive at current brightness
    pub max_brightness_override: bool,
}

impl Default for DeviceState {
    fn default() -> Self {
        Self {
            on: false,
            brightness: 254,        // Default to full brightness
            color_temp_mireds: 250, // ~4000K (neutral white)
            hue: 0,
            saturation: 0,
            use_color_temp: true,           // Default to white light mode
            max_brightness_override: false, // Default to normal mode
        }
    }
}

/// Target state for LED animation, includes both the target PWM values and transition duration
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct TargetState {
    /// Target PWM state to reach
    pub target: LedPwmState,
    /// Transition duration in milliseconds (0 = instant)
    pub transition_duration_ms: u32,
}

impl Default for LedPwmState {
    fn default() -> Self {
        Self::new()
    }
}

impl LedPwmState {
    pub const fn new() -> Self {
        Self {
            r: 0,
            g: 0,
            b: 0,
            cw: 0,
            ww: 0,
        }
    }

    /// Check if any channel is on
    pub fn is_on(&self) -> bool {
        self.r != 0 || self.g != 0 || self.b != 0 || self.cw != 0 || self.ww != 0
    }
}

impl DeviceState {
    /// Convert semantic device state to PWM values
    pub fn to_pwm_state(&self) -> LedPwmState {
        if !self.on {
            // Light is off, all channels zero
            return LedPwmState::new();
        }

        let brightness_scale = self.brightness as u32;

        // Max brightness override mode - all channels at current brightness
        if self.max_brightness_override {
            let max_value = scale_brightness(255, brightness_scale);
            return LedPwmState {
                r: max_value,
                g: max_value,
                b: max_value,
                cw: max_value,
                ww: max_value,
            };
        }

        if self.use_color_temp {
            // Color temperature mode - use CW/WW channels
            let (cw, ww) = mireds_to_cwww(self.color_temp_mireds);
            LedPwmState {
                r: 0,
                g: 0,
                b: 0,
                cw: scale_brightness(cw, brightness_scale),
                ww: scale_brightness(ww, brightness_scale),
            }
        } else {
            // HSV color mode - use RGB channels
            let (r, g, b) = hsv_to_rgb(self.hue, self.saturation, 255);
            LedPwmState {
                r: scale_brightness(r, brightness_scale),
                g: scale_brightness(g, brightness_scale),
                b: scale_brightness(b, brightness_scale),
                cw: 0,
                ww: 0,
            }
        }
    }
}

/// Scale a PWM value by brightness (0-254 Matter range)
fn scale_brightness(value: u8, brightness: u32) -> u8 {
    ((value as u32 * brightness) / 254) as u8
}

/// Convert color temperature in mireds to CW/WW duty cycles
pub fn mireds_to_cwww(mireds: u16) -> (u8, u8) {
    // Matter range: 153 mireds (~6500K cool) to 500 mireds (~2000K warm)
    const MIN_MIREDS: u16 = 153;
    const MAX_MIREDS: u16 = 500;

    // Our LED color temperatures
    const CW_KELVIN: f32 = 6500.0; // Cold white
    const WW_KELVIN: f32 = 2000.0; // Warm white

    let m = mireds.clamp(MIN_MIREDS, MAX_MIREDS);

    // Convert mireds to Kelvin (K = 1,000,000 / mireds)
    let target_kelvin = 1_000_000.0 / m as f32;

    // Clamp to LED range and calculate linear interpolation in Kelvin space
    // (more perceptually uniform than interpolating in mireds)
    let target_k = target_kelvin.clamp(WW_KELVIN, CW_KELVIN);
    let t = (target_k - WW_KELVIN) / (CW_KELVIN - WW_KELVIN);

    let cw = (t * 255.0) as u8;
    let ww = 255 - cw;

    (cw, ww)
}

/// Convert HSV to RGB using the palette crate
pub fn hsv_to_rgb(h: u8, s: u8, v: u8) -> (u8, u8, u8) {
    // Convert Matter ranges (h: 0-254, s: 0-254, v: 0-255) to palette's expected ranges
    let h_deg = (h as f32 * 360.0) / 254.0;
    let s_norm = s as f32 / 254.0;
    let v_norm = v as f32 / 255.0;

    // Create HSV color and convert to sRGB
    let hsv = Hsv::new(h_deg, s_norm, v_norm);
    let rgb = Srgb::from_color(hsv);

    // Convert to u8 values (0-255)
    (
        (rgb.red * 255.0) as u8,
        (rgb.green * 255.0) as u8,
        (rgb.blue * 255.0) as u8,
    )
}

/// Global LED state shared between Matter handlers and PWM task (deprecated, use TARGET_STATE)
pub static LED_STATE: Mutex<CriticalSectionRawMutex, LedPwmState> = Mutex::new(LedPwmState::new());

/// Global semantic device state - tracks user intent (on/off, brightness, color temp, etc.)
pub static DEVICE_STATE: Mutex<CriticalSectionRawMutex, DeviceState> = Mutex::new(DeviceState {
    on: false,
    brightness: 254,
    color_temp_mireds: 250,
    hue: 0,
    saturation: 0,
    use_color_temp: true,
    max_brightness_override: false,
});

/// Sticky signal for target LED state - PWM animation task waits on this
/// Supports up to 5 concurrent waiters (persistence task, PWM task, etc.)
pub static TARGET_STATE: StickySignal<CriticalSectionRawMutex, TargetState, 5> =
    StickySignal::new_with_name("led_target");

/// Global pointer to the LED Output pin.
/// This allows `device.rs` (logic) to control the LED directly.
/// It is wrapped in a Mutex for safe mutable access.
pub static GLOBAL_LED_PIN: Mutex<CriticalSectionRawMutex, Option<Output<'static>>> =
    Mutex::new(None);
