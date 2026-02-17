use crate::clusters::*;
use crate::led::GLOBAL_LED_PIN;
use crate::led::{LED_STATE, LedPwmState};
use rs_matter::dm::Cluster;
use rs_matter::dm::clusters::on_off::StartUpOnOffEnum;
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

    fn start_up_on_off(&self) -> Nullable<StartUpOnOffEnum> {
        // Return null to indicate we don't support startup on/off configuration
        Nullable::none()
    }

    fn set_start_up_on_off(&self, _value: Nullable<StartUpOnOffEnum>) -> Result<(), MatterError> {
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
