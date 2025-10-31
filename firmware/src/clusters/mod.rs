use defmt::Format;
pub use rs_matter_embassy::matter::dm::clusters::on_off::{
    AttributeId as OnOffAttributeId, CommandId as OnOffCommandId,
    FULL_CLUSTER as ON_OFF_FULL_CLUSTER, OnOffHandler, OnOffHooks,
};

pub use rs_matter_embassy::matter::dm::clusters::level_control::{
    AttributeId as LevelControlAttributeId, CommandId as LevelControlCommandId,
    FULL_CLUSTER as LEVEL_CONTROL_FULL_CLUSTER, LevelControlHandler, LevelControlHooks,
};

rs_matter::import! {
    ColorControl,
}

impl Format for color_control::MetadataDebug {}
