use crate::clusters::*;
use crate::led::GLOBAL_LED_PIN;
use crate::led::{LED_STATE, LedPwmState};
use rs_matter::dm::Cluster;
use rs_matter::error::Error as MatterError;
use rs_matter::tlv::Nullable;

/// Device logic implementation for LED control via Matter
#[derive(Default)]
pub struct LedDeviceLogic;

impl LedDeviceLogic {
    /// Get current on/off state by checking if any LED channel is active
    fn get_current_on_off() -> bool {
        // We need to check this synchronously, so we use try_lock
        if let Ok(state_guard) = LED_STATE.try_lock() {
            state_guard.is_on()
        } else {
            false // Default to off if we can't get the lock
        }
    }
}

impl OnOffHooks for LedDeviceLogic {
    const CLUSTER: Cluster<'static> = ON_OFF_FULL_CLUSTER;

    fn on_off(&self) -> bool {
        Self::get_current_on_off()
    }

    fn set_on_off(&self, on: bool) {
        defmt::info!("OnOff command received: {}", on);
        defmt::info!("Heap Stats: {}", esp_alloc::HEAP.stats());

        let new_state = if on {
            // Define ON state (RED at 100% to match example)
            LedPwmState {
                r: 255,
                g: 0,
                b: 0,
                cw: 0,
                ww: 0,
            }
        } else {
            LedPwmState::default()
        };

        // Update the LED state
        if let Ok(mut state_guard) = LED_STATE.try_lock() {
            *state_guard = new_state;
            defmt::info!("Set LED target state to: {:?}", new_state);
        } else {
            defmt::warn!("Could not acquire LED_STATE lock to set on/off");
        }

        // Direct write to GPIO
        if let Ok(mut pin_guard) = GLOBAL_LED_PIN.try_lock() {
            if let Some(pin) = pin_guard.as_mut() {
                if on {
                    pin.set_high();
                } else {
                    pin.set_low();
                }
            } else {
                defmt::warn!("Could not acquire GLOBAL_LED_PIN lock");
            }
        } else {
            defmt::warn!("Could not acquire GLOBAL_LED_PIN lock");
        }
    }

    fn start_up_on_off(&self) -> Nullable<on_off::StartUpOnOffEnum> {
        // Return null to indicate we don't support startup on/off configuration
        Nullable::none()
    }

    fn set_start_up_on_off(
        &self,
        _value: Nullable<on_off::StartUpOnOffEnum>,
    ) -> Result<(), MatterError> {
        // We don't support configuring startup behavior, but return Ok to not cause errors
        Ok(())
    }

    async fn handle_off_with_effect(
        &self,
        _effect: rs_matter::dm::clusters::on_off::EffectVariantEnum,
    ) {
        // For now, just turn off immediately
        // You could implement fade effects here based on the effect parameter
        self.set_on_off(false);
    }
}

use crate::clusters::identify::IdentifyHooks;

impl IdentifyHooks for LedDeviceLogic {
    fn on_identify(&self, time: u16) {
        defmt::info!("Identify command received: {} seconds", time);
        if time > 0 {
            // Blink the LED for identification
            // Since we don't have a background task for this yet, we just log it
            // Real implementation would likely start a blinking task or similar
            // For now, let's just toggle it once if it's off
            if !Self::get_current_on_off() {
                self.set_on_off(true);
            }
        }
    }
}

impl LevelControlHooks for LedDeviceLogic {
    const MIN_LEVEL: u8 = 1;

    const MAX_LEVEL: u8 = 254;

    const FASTEST_RATE: u8 = 50;

    const CLUSTER: Cluster<'static> = LEVEL_CONTROL_FULL_CLUSTER;

    fn set_device_level(&self, level: u8) -> Result<Option<u8>, ()> {
        defmt::info!("device level set to {}", level);
        Ok(Some(level))
    }

    fn current_level(&self) -> Option<u8> {
        return Some(0);
    }

    fn set_current_level(&self, level: Option<u8>) {
        defmt::info!("level set to {}", level.unwrap());
    }
}

use crate::clusters::color_control::ColorControlHooks;

impl ColorControlHooks for LedDeviceLogic {
    fn on_color_control(&self, hue: u8, saturation: u8, temp: u16) {
        defmt::info!("Color Control: H={}, S={}, T={}", hue, saturation, temp);

        // Convert HSV to RGB
        let (r, g, b) = hsv_to_rgb(hue, saturation, 255);

        // Convert Color Temp to CW/WW
        let (cw, ww) = if temp > 0 {
            mireds_to_cwww(temp)
        } else {
            (0, 0)
        };

        let new_state = LedPwmState { r, g, b, cw, ww };

        // Update the LED state
        if let Ok(mut state_guard) = LED_STATE.try_lock() {
            *state_guard = new_state;
            defmt::info!("Set LED color state to: {:?}", new_state);
        }
    }
}

fn hsv_to_rgb(h: u8, s: u8, v: u8) -> (u8, u8, u8) {
    let h_deg = (h as u16 * 360) / 254;
    let s_float = s as f32 / 254.0;
    let v_float = v as f32 / 255.0;

    let c = v_float * s_float;
    let x = c * (1.0 - ((h_deg as f32 / 60.0) % 2.0 - 1.0).abs());
    let m = v_float - c;

    let (r_prime, g_prime, b_prime) = if h_deg < 60 {
        (c, x, 0.0)
    } else if h_deg < 120 {
        (x, c, 0.0)
    } else if h_deg < 180 {
        (0.0, c, x)
    } else if h_deg < 240 {
        (0.0, x, c)
    } else if h_deg < 300 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };

    (
        ((r_prime + m) * 255.0) as u8,
        ((g_prime + m) * 255.0) as u8,
        ((b_prime + m) * 255.0) as u8,
    )
}

fn mireds_to_cwww(mireds: u16) -> (u8, u8) {
    // Range 153 (Cool) to 500 (Warm)
    const MIN_MIREDS: u16 = 153;
    const MAX_MIREDS: u16 = 500;

    let m = mireds.clamp(MIN_MIREDS, MAX_MIREDS);
    let range = MAX_MIREDS - MIN_MIREDS;
    let val = m - MIN_MIREDS;

    // WW increases with mireds (warmer)
    // Map 0..range to 0..255
    let ww = ((val as u32 * 255) / range as u32) as u8;
    // CW decreases with mireds
    let cw = 255 - ww;

    (cw, ww)
}
