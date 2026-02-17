use core::cell::Cell;
use rs_matter::{
    dm::{Cluster, Dataver, HandlerContext},
    error::Error,
};

use crate::clusters::decl::color_control::{
    ColorCapabilitiesBitmap, ColorLoopSetRequest, ColorModeEnum, EnhancedColorModeEnum,
    EnhancedMoveHueRequest, EnhancedMoveToHueAndSaturationRequest, EnhancedMoveToHueRequest,
    EnhancedStepHueRequest, MoveColorRequest, MoveColorTemperatureRequest, MoveHueRequest,
    MoveSaturationRequest, MoveToColorRequest, MoveToColorTemperatureRequest,
    MoveToHueAndSaturationRequest, MoveToHueRequest, MoveToSaturationRequest, OptionsBitmap,
    StepColorRequest, StepColorTemperatureRequest, StepHueRequest, StepSaturationRequest,
    StopMoveStepRequest,
};

use super::decl::color_control::ClusterAsyncHandler;

pub const CLUSTER_ID: u32 = 0x0300;

pub enum AttributeId {
    CurrentHue = 0x0000,
    CurrentSaturation = 0x0001,
    RemainingTime = 0x0002,
    CurrentX = 0x0003,
    CurrentY = 0x0004,
    DriftCompensation = 0x0005,
    CompensationText = 0x0006,
    ColorTemperatureMireds = 0x0007,
    ColorMode = 0x0008,
    Options = 0x000F,
    NumberOfPrimaries = 0x0010,
    Primary1X = 0x0011,
    Primary1Y = 0x0012,
    Primary1Intensity = 0x0013,
    Primary2X = 0x0015,
    Primary2Y = 0x0016,
    Primary2Intensity = 0x0017,
    Primary3X = 0x0019,
    Primary3Y = 0x001A,
    Primary3Intensity = 0x001B,
    Primary4X = 0x0020,
    Primary4Y = 0x0021,
    Primary4Intensity = 0x0022,
    Primary5X = 0x0024,
    Primary5Y = 0x0025,
    Primary5Intensity = 0x0026,
    Primary6X = 0x0028,
    Primary6Y = 0x0029,
    Primary6Intensity = 0x002A,
    WhitePointX = 0x0030,
    WhitePointY = 0x0031,
    ColorPointRX = 0x0032,
    ColorPointRY = 0x0033,
    ColorPointRIntensity = 0x0034,
    ColorPointGX = 0x0036,
    ColorPointGY = 0x0037,
    ColorPointGIntensity = 0x0038,
    ColorPointBX = 0x003A,
    ColorPointBY = 0x003B,
    ColorPointBIntensity = 0x003C,
    EnhancedCurrentHue = 0x4000,
    EnhancedColorMode = 0x4001,
    ColorLoopActive = 0x4002,
    ColorLoopDirection = 0x4003,
    ColorLoopTime = 0x4004,
    ColorLoopStartEnhancedHue = 0x4005,
    ColorLoopStoredEnhancedHue = 0x4006,
    ColorCapabilities = 0x400A,
    ColorTempPhysicalMin = 0x400B,
    ColorTempPhysicalMax = 0x400C,
    CoupleColorTempToLevelMinMireds = 0x400D,
    StartUpColorTemperatureMireds = 0x4010,
}

pub enum CommandId {
    MoveToHue = 0x00,
    MoveToSaturation = 0x01,
    MoveToHueAndSaturation = 0x06,
    MoveToColor = 0x07,
    MoveToColorTemperature = 0x0A,
}

pub const CLUSTER: Cluster<'static> = Cluster::new(
    CLUSTER_ID,
    4,    // revision
    0x1F, // feature_map: Init(1) | EnhancedHue(1) | ColorLoop(1) | XY(1) | ColorTemperature(1) -> 0x1F. Ideally check specs.
    &[
        // TODO
        // rs_matter::dm::Attribute::new(
        //     AttributeId::CurrentHue as u32,
        //     rs_matter::dm::AttrValue::Uint8(0),
        //     rs_matter::dm::AttrAccess::ReadWrite,
        // ),
        // rs_matter::dm::Attribute::new(
        //     AttributeId::CurrentSaturation as u32,
        //     rs_matter::dm::AttrValue::Uint8(0),
        //     rs_matter::dm::AttrAccess::ReadWrite,
        // ),
        // rs_matter::dm::Attribute::new(
        //     AttributeId::ColorTemperatureMireds as u32,
        //     rs_matter::dm::AttrValue::Uint16(0), // 0 is invalid usually, typically 370 (2700K) or similar
        //     rs_matter::dm::AttrAccess::ReadWrite,
        // ),
        // rs_matter::dm::Attribute::new(
        //     AttributeId::ColorMode as u32,
        //     rs_matter::dm::AttrValue::Uint8(0), // 0: CurrentHue and CurrentSaturation
        //     rs_matter::dm::AttrAccess::Read,
        // ),
        // rs_matter::dm::Attribute::new(
        //     AttributeId::ColorCapabilities as u32,
        //     rs_matter::dm::AttrValue::Uint16(0x001F), // HS(1)|EH(1)|CL(1)|XY(1)|CT(1)
        //     rs_matter::dm::AttrAccess::Read,
        // ),
        // rs_matter::dm::Attribute::new(
        //     AttributeId::ColorTempPhysicalMin as u32,
        //     rs_matter::dm::AttrValue::Uint16(153), // 6500K
        //     rs_matter::dm::AttrAccess::Read,
        // ),
        // rs_matter::dm::Attribute::new(
        //     AttributeId::ColorTempPhysicalMax as u32,
        //     rs_matter::dm::AttrValue::Uint16(500), // 2000K
        //     rs_matter::dm::AttrAccess::Read,
        // ),
    ],
    &[
        // TODO
        // rs_matter::dm::Command::new(
        //     CommandId::MoveToHue as u32,
        //     rs_matter::dm::CmdAccess::Invoke,
        //     rs_matter::dm::CmdFields::Config(&[
        //         rs_matter::dm::CmdField::new(0, rs_matter::dm::DataType::Uint8), // Hue
        //         rs_matter::dm::CmdField::new(1, rs_matter::dm::DataType::Uint8), // Direction
        //         rs_matter::dm::CmdField::new(2, rs_matter::dm::DataType::Uint16), // TransitionTime
        //         rs_matter::dm::CmdField::new(3, rs_matter::dm::DataType::Uint8), // OptionsMask
        //         rs_matter::dm::CmdField::new(4, rs_matter::dm::DataType::Uint8), // OptionsOverride
        //     ]),
        // ),
        // rs_matter::dm::Command::new(
        //     CommandId::MoveToSaturation as u32,
        //     rs_matter::dm::CmdAccess::Invoke,
        //     rs_matter::dm::CmdFields::Config(&[
        //         rs_matter::dm::CmdField::new(0, rs_matter::dm::DataType::Uint8), // Saturation
        //         rs_matter::dm::CmdField::new(1, rs_matter::dm::DataType::Uint16), // TransitionTime
        //         rs_matter::dm::CmdField::new(2, rs_matter::dm::DataType::Uint8), // OptionsMask
        //         rs_matter::dm::CmdField::new(3, rs_matter::dm::DataType::Uint8), // OptionsOverride
        //     ]),
        // ),
        // rs_matter::dm::Command::new(
        //     CommandId::MoveToHueAndSaturation as u32,
        //     rs_matter::dm::CmdAccess::Invoke,
        //     rs_matter::dm::CmdFields::Config(&[
        //         rs_matter::dm::CmdField::new(0, rs_matter::dm::DataType::Uint8), // Hue
        //         rs_matter::dm::CmdField::new(1, rs_matter::dm::DataType::Uint8), // Saturation
        //         rs_matter::dm::CmdField::new(2, rs_matter::dm::DataType::Uint16), // TransitionTime
        //         rs_matter::dm::CmdField::new(3, rs_matter::dm::DataType::Uint8), // OptionsMask
        //         rs_matter::dm::CmdField::new(4, rs_matter::dm::DataType::Uint8), // OptionsOverride
        //     ]),
        // ),
        // rs_matter::dm::Command::new(
        //     CommandId::MoveToColorTemperature as u32,
        //     rs_matter::dm::CmdAccess::Invoke,
        //     rs_matter::dm::CmdFields::Config(&[
        //         rs_matter::dm::CmdField::new(0, rs_matter::dm::DataType::Uint16), // ColorTemperatureMireds
        //         rs_matter::dm::CmdField::new(1, rs_matter::dm::DataType::Uint16), // TransitionTime
        //         rs_matter::dm::CmdField::new(2, rs_matter::dm::DataType::Uint8),  // OptionsMask
        //         rs_matter::dm::CmdField::new(3, rs_matter::dm::DataType::Uint8),  // OptionsOverride
        //     ]),
        // ),
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
        todo!()
    }

    async fn options(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<OptionsBitmap, rs_matter::error::Error> {
        todo!()
    }

    async fn number_of_primaries(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<rs_matter::tlv::Nullable<u8>, rs_matter::error::Error> {
        todo!()
    }

    async fn enhanced_color_mode(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<EnhancedColorModeEnum, rs_matter::error::Error> {
        todo!()
    }

    async fn color_capabilities(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<ColorCapabilitiesBitmap, rs_matter::error::Error> {
        todo!()
    }

    async fn set_options(
        &self,
        ctx: impl rs_matter::dm::WriteContext,
        value: OptionsBitmap,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_move_to_hue(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: MoveToHueRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_move_hue(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: MoveHueRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_step_hue(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: StepHueRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_move_to_saturation(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: MoveToSaturationRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_move_saturation(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: MoveSaturationRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_step_saturation(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: StepSaturationRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_move_to_hue_and_saturation(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: MoveToHueAndSaturationRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_move_to_color(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: MoveToColorRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_move_color(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: MoveColorRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_step_color(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: StepColorRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_move_to_color_temperature(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: MoveToColorTemperatureRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_enhanced_move_to_hue(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: EnhancedMoveToHueRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_enhanced_move_hue(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: EnhancedMoveHueRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_enhanced_step_hue(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: EnhancedStepHueRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_enhanced_move_to_hue_and_saturation(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: EnhancedMoveToHueAndSaturationRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_color_loop_set(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: ColorLoopSetRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_stop_move_step(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: StopMoveStepRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_move_color_temperature(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: MoveColorTemperatureRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_step_color_temperature(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: StepColorTemperatureRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }
}
