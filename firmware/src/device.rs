use crate::clusters::*;
use crate::led::{DEVICE_STATE, LedPwmState, TARGET_STATE, TargetState, hsv_to_rgb};
use crate::led::{GLOBAL_LED_PIN, mireds_to_cwww};
use rs_matter::dm::Cluster;
use rs_matter::error::Error as MatterError;
use rs_matter::tlv::Nullable;

/// Device logic implementation for LED control via Matter
#[derive(Default)]
pub struct LedDeviceLogic;

impl LedDeviceLogic {
    /// Get current on/off state from device state
    fn get_current_on_off() -> bool {
        if let Ok(state_guard) = DEVICE_STATE.try_lock() {
            state_guard.on
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

        // Update device state
        if let Ok(mut state) = DEVICE_STATE.try_lock() {
            state.on = on;
            defmt::info!("Device state: {:?}", *state);
        }

        // Note: When coupled with LevelControlHandler, the level control cluster
        // will handle smooth transitions using on_transition_time, off_transition_time,
        // and on_off_transition_time attributes. The coupled handler will call
        // set_device_level which updates the PWM via TARGET_STATE.
        //
        // If running standalone (not coupled), directly signal PWM with transition.
        // This happens when level_control_handler.init(Some(&on_off_handler)) is not called.
        if let Ok(state) = DEVICE_STATE.try_lock() {
            let pwm_state = state.to_pwm_state();
            defmt::info!("PWM state: {:?}", pwm_state);

            // Default 400ms transition for standalone mode
            // When coupled, LevelControl cluster handles transitions with configurable timing
            TARGET_STATE.signal(TargetState {
                target: pwm_state,
                transition_duration_ms: 400,
            });
        }

        // Direct write to GPIO (legacy support)
        if let Ok(mut pin_guard) = GLOBAL_LED_PIN.try_lock() {
            if let Some(pin) = pin_guard.as_mut() {
                if on {
                    pin.set_high();
                } else {
                    pin.set_low();
                }
            }
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
use crate::led::pwm::IDENTIFY_SIGNAL;

impl IdentifyHooks for LedDeviceLogic {
    fn on_identify(&self, time: u16) {
        defmt::info!("Identify command received: {} seconds", time);
        if time > 0 {
            // Signal the PWM task to execute the R->G->B identify sequence
            // The sequence will run uninterrupted for 3 seconds (1s per color)
            IDENTIFY_SIGNAL.signal(time);
            defmt::info!("Identify sequence triggered");
        }
    }
}

impl LevelControlHooks for LedDeviceLogic {
    const MIN_LEVEL: u8 = 1;

    const MAX_LEVEL: u8 = 254;

    const FASTEST_RATE: u8 = 50;

    const CLUSTER: Cluster<'static> = LEVEL_CONTROL_FULL_CLUSTER;

    fn set_device_level(&self, level: u8) -> Result<Option<u8>, ()> {
        defmt::info!("Device level set to {}", level);

        if let Ok(mut state) = DEVICE_STATE.try_lock() {
            state.brightness = level;
            state.max_brightness_override = false; // Clear override on brightness change
            let pwm_state = state.to_pwm_state();

            defmt::info!("Brightness updated: level={}, on={}", level, state.on);
            defmt::info!("PWM state: {:?}", pwm_state);

            // Signal the PWM task with the new target state
            TARGET_STATE.signal(TargetState {
                target: pwm_state,
                transition_duration_ms: 0,
            });

            Ok(Some(level))
        } else {
            Err(())
        }
    }

    fn current_level(&self) -> Option<u8> {
        if let Ok(state) = DEVICE_STATE.try_lock() {
            Some(state.brightness)
        } else {
            Some(0)
        }
    }

    fn set_current_level(&self, level: Option<u8>) {
        if let Some(lvl) = level {
            defmt::info!("Current level set to {}", lvl);
            let _ = self.set_device_level(lvl);
        }
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
        defmt::info!("Set LED color state to: {:?}", new_state);

        // Signal the PWM task with the new target state (instant transition)
        TARGET_STATE.signal(TargetState {
            target: new_state,
            transition_duration_ms: 0,
        });
    }
}

/// Max Brightness Switch Logic - controls the max_brightness_override flag
#[derive(Default)]
pub struct MaxBrightnessSwitch;

impl MaxBrightnessSwitch {
    /// Get current override state
    fn get_current_override() -> bool {
        if let Ok(state_guard) = DEVICE_STATE.try_lock() {
            state_guard.max_brightness_override
        } else {
            false
        }
    }
}

impl OnOffHooks for MaxBrightnessSwitch {
    const CLUSTER: Cluster<'static> = ON_OFF_FULL_CLUSTER;

    fn on_off(&self) -> bool {
        Self::get_current_override()
    }

    fn set_on_off(&self, on: bool) {
        defmt::info!("Max Brightness Override: {}", on);

        // Update device state override flag
        if let Ok(mut state) = DEVICE_STATE.try_lock() {
            state.max_brightness_override = on;
            let pwm_state = state.to_pwm_state();

            defmt::info!("Override state: {:?}", *state);
            defmt::info!("PWM state: {:?}", pwm_state);

            // Signal the PWM task with smooth transition
            TARGET_STATE.signal(TargetState {
                target: pwm_state,
                transition_duration_ms: 400,
            });
        }
    }

    fn start_up_on_off(&self) -> Nullable<on_off::StartUpOnOffEnum> {
        Nullable::none()
    }

    fn set_start_up_on_off(
        &self,
        _value: Nullable<on_off::StartUpOnOffEnum>,
    ) -> Result<(), MatterError> {
        Ok(())
    }

    async fn handle_off_with_effect(
        &self,
        _effect: rs_matter::dm::clusters::on_off::EffectVariantEnum,
    ) {
        self.set_on_off(false);
    }
}
