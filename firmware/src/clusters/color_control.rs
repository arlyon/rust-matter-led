use crate::led::{DEVICE_STATE, TARGET_STATE, TargetState};
use core::cell::Cell;
use rs_matter::{
    dm::{Access, Cluster, Dataver, HandlerContext, Quality},
    error::Error,
};

use crate::clusters::decl::color_control::{
    AttributeId, ColorCapabilitiesBitmap, ColorLoopSetRequest, ColorModeEnum, CommandId,
    EnhancedColorModeEnum, EnhancedMoveHueRequest, EnhancedMoveToHueAndSaturationRequest,
    EnhancedMoveToHueRequest, EnhancedStepHueRequest, MoveColorRequest,
    MoveColorTemperatureRequest, MoveHueRequest, MoveSaturationRequest, MoveToColorRequest,
    MoveToColorTemperatureRequest, MoveToHueAndSaturationRequest, MoveToHueRequest,
    MoveToSaturationRequest, OptionsBitmap, StepColorRequest, StepColorTemperatureRequest,
    StepHueRequest, StepSaturationRequest, StopMoveStepRequest,
};

use super::decl::color_control::ClusterAsyncHandler;

pub const CLUSTER_ID: u32 = 0x0300;

pub const CLUSTER: Cluster<'static> = Cluster::new(
    CLUSTER_ID,
    4,                  // revision
    0x01 | 0x02 | 0x10, // hue/sat + enhanced hue + color temp
    &[
        rs_matter::dm::Attribute::new(AttributeId::ColorMode as u32, Access::RV, Quality::NONE),
        rs_matter::dm::Attribute::new(
            AttributeId::EnhancedColorMode as u32,
            Access::RV,
            Quality::NONE,
        ),
        rs_matter::dm::Attribute::new(
            AttributeId::ColorCapabilities as u32,
            Access::RV,
            Quality::NONE,
        ),
        rs_matter::dm::Attribute::new(AttributeId::RemainingTime as u32, Access::RV, Quality::NONE),
        rs_matter::dm::Attribute::new(AttributeId::Options as u32, Access::RWVM, Quality::NONE),
        // --- Hue & Saturation (0x01) ---
        rs_matter::dm::Attribute::new(AttributeId::CurrentHue as u32, Access::RV, Quality::NONE),
        rs_matter::dm::Attribute::new(
            AttributeId::CurrentSaturation as u32,
            Access::RV,
            Quality::NONE,
        ),
        // --- Enhanced Hue (0x02) ---
        rs_matter::dm::Attribute::new(
            AttributeId::EnhancedCurrentHue as u32,
            Access::RV,
            Quality::NONE,
        ),
        // --- Color Temperature (0x10) ---
        rs_matter::dm::Attribute::new(
            AttributeId::ColorTemperatureMireds as u32,
            Access::RV,
            Quality::NONE,
        ),
        rs_matter::dm::Attribute::new(
            AttributeId::ColorTempPhysicalMinMireds as u32,
            Access::RV,
            Quality::NONE,
        ),
        rs_matter::dm::Attribute::new(
            AttributeId::ColorTempPhysicalMaxMireds as u32,
            Access::RV,
            Quality::NONE,
        ),
        // Startup color temp should be Non-Volatile (survives reboots) and Nullable (if not set)
        rs_matter::dm::Attribute::new(
            AttributeId::StartUpColorTemperatureMireds as u32,
            Access::RWVM,
            Quality::from_bits_truncate(Quality::N.bits() | Quality::X.bits()),
        ),
    ],
    &[
        rs_matter::dm::Command::new(CommandId::StopMoveStep as u32, None, Access::NEED_OPERATE), // StopMoveStep
        // --- Hue & Saturation Commands ---
        rs_matter::dm::Command::new(CommandId::MoveToHue as u32, None, Access::NEED_OPERATE),
        rs_matter::dm::Command::new(
            CommandId::MoveToSaturation as u32,
            None,
            Access::NEED_OPERATE,
        ),
        rs_matter::dm::Command::new(
            CommandId::MoveToHueAndSaturation as u32,
            None,
            Access::NEED_OPERATE,
        ),
        rs_matter::dm::Command::new(CommandId::StepHue as u32, None, Access::NEED_OPERATE), // StepHue
        rs_matter::dm::Command::new(CommandId::MoveSaturation as u32, None, Access::NEED_OPERATE), // MoveSaturation
        rs_matter::dm::Command::new(CommandId::StepSaturation as u32, None, Access::NEED_OPERATE), // StepSaturation
        // --- Enhanced Hue Commands ---
        rs_matter::dm::Command::new(
            CommandId::EnhancedMoveToHue as u32,
            None,
            Access::NEED_OPERATE,
        ), // EnhancedMoveToHue
        rs_matter::dm::Command::new(
            CommandId::EnhancedMoveHue as u32,
            None,
            Access::NEED_OPERATE,
        ), // EnhancedMoveHue
        rs_matter::dm::Command::new(
            CommandId::EnhancedStepHue as u32,
            None,
            Access::NEED_OPERATE,
        ), // EnhancedStepHue
        rs_matter::dm::Command::new(
            CommandId::EnhancedMoveToHueAndSaturation as u32,
            None,
            Access::NEED_OPERATE,
        ), // EnhancedMoveToHueAndSaturation
        // --- Color Temp Commands ---
        rs_matter::dm::Command::new(
            CommandId::MoveToColorTemperature as u32,
            None,
            Access::NEED_OPERATE,
        ),
        rs_matter::dm::Command::new(
            CommandId::MoveColorTemperature as u32,
            None,
            Access::NEED_OPERATE,
        ), // MoveColorTemperature
        rs_matter::dm::Command::new(
            CommandId::StepColorTemperature as u32,
            None,
            Access::NEED_OPERATE,
        ), // StepColorTemperature
    ],
    rs_matter::with!(all),
    rs_matter::with!(all),
);

pub trait ColorControlHooks {
    fn on_color_control(&self, hue: u8, saturation: u8, temp: u16);
}

pub struct ColorControlHandler<'a, H: ColorControlHooks> {
    dataver: Dataver,
    hooks: H,
    hue: Cell<u8>,
    saturation: Cell<u8>,
    temp: Cell<u16>,
    _phantom: core::marker::PhantomData<&'a ()>,
}

impl<'a, H: ColorControlHooks> ColorControlHandler<'a, H> {
    pub fn new(dataver: Dataver, hooks: H) -> Self {
        Self {
            dataver,
            hooks,
            hue: Cell::new(0),
            saturation: Cell::new(0),
            temp: Cell::new(250), // Default 4000K
            _phantom: core::marker::PhantomData,
        }
    }

    pub const fn adapt(self) -> super::decl::color_control::HandlerAsyncAdaptor<Self> {
        super::decl::color_control::HandlerAsyncAdaptor(self)
    }

    fn notify(&self) {
        self.hooks
            .on_color_control(self.hue.get(), self.saturation.get(), self.temp.get());
    }
}

impl<H: ColorControlHooks> ClusterAsyncHandler for ColorControlHandler<'_, H> {
    const CLUSTER: Cluster<'static> = CLUSTER;

    fn dataver(&self) -> u32 {
        self.dataver.get()
    }

    fn dataver_changed(&self) {
        self.dataver.changed();
    }

    async fn run(&self, _ctx: impl HandlerContext) -> Result<(), Error> {
        core::future::pending().await
    }

    async fn color_mode(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<ColorModeEnum, rs_matter::error::Error> {
        defmt::info!("color_mode read");
        if let Ok(state) = DEVICE_STATE.try_lock() {
            if state.use_color_temp {
                Ok(ColorModeEnum::ColorTemperatureMireds)
            } else {
                Ok(ColorModeEnum::CurrentHueAndCurrentSaturation)
            }
        } else {
            Ok(ColorModeEnum::ColorTemperatureMireds)
        }
    }

    async fn options(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<OptionsBitmap, rs_matter::error::Error> {
        defmt::info!("options read");
        Ok(OptionsBitmap::empty())
    }

    async fn number_of_primaries(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<rs_matter::tlv::Nullable<u8>, rs_matter::error::Error> {
        defmt::info!("number_of_primaries read");
        Ok(rs_matter::tlv::Nullable::some(1))
    }

    async fn enhanced_color_mode(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<EnhancedColorModeEnum, rs_matter::error::Error> {
        defmt::info!("enhanced_color_mode read");
        if let Ok(state) = DEVICE_STATE.try_lock() {
            if state.use_color_temp {
                Ok(EnhancedColorModeEnum::ColorTemperatureMireds)
            } else {
                Ok(EnhancedColorModeEnum::EnhancedCurrentHueAndCurrentSaturation)
            }
        } else {
            Ok(EnhancedColorModeEnum::ColorTemperatureMireds)
        }
    }

    async fn color_capabilities(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<ColorCapabilitiesBitmap, rs_matter::error::Error> {
        defmt::info!("color_capabilities read");
        Ok(ColorCapabilitiesBitmap::COLOR_TEMPERATURE
            | ColorCapabilitiesBitmap::HUE_SATURATION
            | ColorCapabilitiesBitmap::ENHANCED_HUE)
    }

    async fn set_options(
        &self,
        ctx: impl rs_matter::dm::WriteContext,
        value: OptionsBitmap,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("set_options called");
        Ok(())
    }

    async fn handle_move_to_hue(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        request: MoveToHueRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        let hue = request.hue()?;
        let transition_time = request.transition_time()?;

        defmt::info!(
            "handle_move_to_hue: hue={}, transition={}",
            hue,
            transition_time
        );

        // Update internal state
        self.hue.set(hue);

        // Update device state and switch to HSV mode
        if let Ok(mut state) = DEVICE_STATE.try_lock() {
            state.hue = hue;
            state.use_color_temp = false; // Switch to HSV mode
            let pwm_state = state.to_pwm_state();

            defmt::info!("Hue updated: {:?}", *state);
            defmt::info!("PWM state: {:?}", pwm_state);

            // Convert transition time (1/10ths of a second) to milliseconds
            let transition_ms = (transition_time as u32) * 100;

            // Signal the PWM task
            TARGET_STATE.signal(TargetState {
                target: pwm_state,
                transition_duration_ms: transition_ms,
            });
        }

        self.dataver_changed();
        Ok(())
    }

    async fn handle_move_hue(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: MoveHueRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("handle_move_hue called");
        Ok(())
    }

    async fn handle_step_hue(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: StepHueRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("handle_step_hue called");
        Ok(())
    }

    async fn handle_move_to_saturation(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        request: MoveToSaturationRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        let saturation = request.saturation()?;
        let transition_time = request.transition_time()?;

        defmt::info!(
            "handle_move_to_saturation: sat={}, transition={}",
            saturation,
            transition_time
        );

        // Update internal state
        self.saturation.set(saturation);

        // Update device state and switch to HSV mode
        if let Ok(mut state) = DEVICE_STATE.try_lock() {
            state.saturation = saturation;
            state.use_color_temp = false; // Switch to HSV mode
            let pwm_state = state.to_pwm_state();

            defmt::info!("Saturation updated: {:?}", *state);
            defmt::info!("PWM state: {:?}", pwm_state);

            // Convert transition time (1/10ths of a second) to milliseconds
            let transition_ms = (transition_time as u32) * 100;

            // Signal the PWM task
            TARGET_STATE.signal(TargetState {
                target: pwm_state,
                transition_duration_ms: transition_ms,
            });
        }

        self.dataver_changed();
        Ok(())
    }

    async fn handle_move_saturation(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: MoveSaturationRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("handle_move_saturation called");
        Ok(())
    }

    async fn handle_step_saturation(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: StepSaturationRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("handle_step_saturation called");
        Ok(())
    }

    async fn handle_move_to_hue_and_saturation(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        request: MoveToHueAndSaturationRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        let hue = request.hue()?;
        let saturation = request.saturation()?;
        let transition_time = request.transition_time()?;

        defmt::info!(
            "handle_move_to_hue_and_saturation: hue={}, sat={}, transition={}",
            hue,
            saturation,
            transition_time
        );

        // Update internal state
        self.hue.set(hue);
        self.saturation.set(saturation);

        // Update device state and switch to HSV mode
        if let Ok(mut state) = DEVICE_STATE.try_lock() {
            state.hue = hue;
            state.saturation = saturation;
            state.use_color_temp = false; // Switch to HSV mode
            let pwm_state = state.to_pwm_state();

            defmt::info!("HSV color updated: {:?}", *state);
            defmt::info!("PWM state: {:?}", pwm_state);

            // Convert transition time (1/10ths of a second) to milliseconds
            let transition_ms = (transition_time as u32) * 100;

            // Signal the PWM task
            TARGET_STATE.signal(TargetState {
                target: pwm_state,
                transition_duration_ms: transition_ms,
            });
        }

        self.dataver_changed();
        Ok(())
    }

    async fn handle_move_to_color(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: MoveToColorRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("handle_move_to_color called");
        Ok(())
    }

    async fn handle_move_color(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: MoveColorRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("handle_move_color called");
        Ok(())
    }

    async fn handle_step_color(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: StepColorRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("handle_step_color called");
        Ok(())
    }

    async fn handle_move_to_color_temperature(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        request: MoveToColorTemperatureRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        let color_temp = request.color_temperature_mireds()?;
        let transition_time = request.transition_time()?;

        defmt::info!(
            "handle_move_to_color_temperature: temp={} mireds, transition={}",
            color_temp,
            transition_time
        );

        // Update internal state
        self.temp.set(color_temp);

        // Update device state and switch to color temp mode
        if let Ok(mut state) = DEVICE_STATE.try_lock() {
            state.color_temp_mireds = color_temp;
            state.use_color_temp = true;
            let pwm_state = state.to_pwm_state();

            defmt::info!("Color temp updated: {:?}", *state);
            defmt::info!("PWM state: {:?}", pwm_state);

            // Convert transition time (1/10ths of a second) to milliseconds
            let transition_ms = (transition_time as u32) * 100;

            // Signal the PWM task
            TARGET_STATE.signal(TargetState {
                target: pwm_state,
                transition_duration_ms: transition_ms,
            });
        }

        self.dataver_changed();
        Ok(())
    }

    async fn handle_enhanced_move_to_hue(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        request: EnhancedMoveToHueRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        let enhanced_hue = request.enhanced_hue()?;
        let transition_time = request.transition_time()?;

        defmt::info!(
            "handle_enhanced_move_to_hue: enhanced_hue={}, transition={}",
            enhanced_hue,
            transition_time
        );

        // Convert enhanced hue (16-bit) to regular hue (8-bit)
        let hue = (enhanced_hue / 256) as u8;
        self.hue.set(hue);

        // Update device state and switch to HSV mode
        if let Ok(mut state) = DEVICE_STATE.try_lock() {
            state.hue = hue;
            state.use_color_temp = false; // Switch to HSV mode
            let pwm_state = state.to_pwm_state();

            defmt::info!("Enhanced hue updated: {:?}", *state);
            defmt::info!("PWM state: {:?}", pwm_state);

            // Convert transition time (1/10ths of a second) to milliseconds
            let transition_ms = (transition_time as u32) * 100;

            // Signal the PWM task
            TARGET_STATE.signal(TargetState {
                target: pwm_state,
                transition_duration_ms: transition_ms,
            });
        }

        self.dataver_changed();
        Ok(())
    }

    async fn handle_enhanced_move_hue(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: EnhancedMoveHueRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("handle_enhanced_move_hue called");
        Ok(())
    }

    async fn handle_enhanced_step_hue(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: EnhancedStepHueRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("handle_enhanced_step_hue called");
        Ok(())
    }

    async fn handle_enhanced_move_to_hue_and_saturation(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        request: EnhancedMoveToHueAndSaturationRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        let enhanced_hue = request.enhanced_hue()?;
        let saturation = request.saturation()?;
        let transition_time = request.transition_time()?;

        defmt::info!(
            "handle_enhanced_move_to_hue_and_saturation: enhanced_hue={}, sat={}, transition={}",
            enhanced_hue,
            saturation,
            transition_time
        );

        // Convert enhanced hue (16-bit) to regular hue (8-bit)
        let hue = (enhanced_hue / 256) as u8;
        self.hue.set(hue);
        self.saturation.set(saturation);

        // Update device state and switch to HSV mode
        if let Ok(mut state) = DEVICE_STATE.try_lock() {
            state.hue = hue;
            state.saturation = saturation;
            state.use_color_temp = false; // Switch to HSV mode
            let pwm_state = state.to_pwm_state();

            defmt::info!("Enhanced HSV updated: {:?}", *state);
            defmt::info!("PWM state: {:?}", pwm_state);

            // Convert transition time (1/10ths of a second) to milliseconds
            let transition_ms = (transition_time as u32) * 100;

            // Signal the PWM task
            TARGET_STATE.signal(TargetState {
                target: pwm_state,
                transition_duration_ms: transition_ms,
            });
        }

        self.dataver_changed();
        Ok(())
    }

    async fn handle_color_loop_set(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: ColorLoopSetRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("handle_color_loop_set called");
        Ok(())
    }

    async fn handle_stop_move_step(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: StopMoveStepRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("handle_stop_move_step called");
        Ok(())
    }

    async fn handle_move_color_temperature(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: MoveColorTemperatureRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("handle_move_color_temperature called");
        Ok(())
    }

    async fn handle_step_color_temperature(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: StepColorTemperatureRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("handle_step_color_temperature called");
        Ok(())
    }

    async fn current_hue(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<u8, rs_matter::error::Error> {
        if let Ok(state) = DEVICE_STATE.try_lock() {
            Ok(state.hue)
        } else {
            Ok(0)
        }
    }

    async fn current_saturation(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<u8, rs_matter::error::Error> {
        if let Ok(state) = DEVICE_STATE.try_lock() {
            Ok(state.saturation)
        } else {
            Ok(0)
        }
    }

    async fn color_temperature_mireds(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<u16, rs_matter::error::Error> {
        if let Ok(state) = DEVICE_STATE.try_lock() {
            Ok(state.color_temp_mireds)
        } else {
            Ok(250) // Default 4000K if we can't read state
        }
    }

    async fn enhanced_current_hue(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<u16, rs_matter::error::Error> {
        if let Ok(state) = DEVICE_STATE.try_lock() {
            // Enhanced hue is 16-bit, regular hue is 8-bit
            // Scale up: enhanced_hue = hue * 256 / 254 * 254 = hue * 256
            Ok((state.hue as u16) * 256)
        } else {
            Ok(0)
        }
    }

    async fn color_temp_physical_min_mireds(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<u16, rs_matter::error::Error> {
        Ok(154) // 6500K
    }

    async fn color_temp_physical_max_mireds(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<u16, rs_matter::error::Error> {
        Ok(500) // 2000K
    }

    async fn start_up_color_temperature_mireds(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<rs_matter::tlv::Nullable<u16>, rs_matter::error::Error> {
        Ok(rs_matter::tlv::Nullable::some(200))
    }

    async fn set_start_up_color_temperature_mireds(
        &self,
        ctx: impl rs_matter::dm::WriteContext,
        value: rs_matter::tlv::Nullable<u16>,
    ) -> Result<(), rs_matter::error::Error> {
        Ok(())
    }
}
