pub use rs_matter_embassy::matter::dm::clusters::on_off::{
    AttributeId as OnOffAttributeId, CommandId as OnOffCommandId,
    FULL_CLUSTER as ON_OFF_FULL_CLUSTER, OnOffHandler, OnOffHooks,
};

pub use rs_matter_embassy::matter::dm::clusters::level_control::{
    AttributeId as LevelControlAttributeId, CommandId as LevelControlCommandId,
    FULL_CLUSTER as LEVEL_CONTROL_FULL_CLUSTER, LevelControlHandler, LevelControlHooks,
};

pub mod color_control;
pub mod identify;
pub use rs_matter::dm::clusters::level_control;
pub use rs_matter::dm::clusters::on_off;

pub mod decl {
    rs_matter::import! {
        ColorControl,
        Identify,
    }
}
