use crate::clusters::{color_control, decl, identify, level_control, on_off};

use rs_matter::dm::DeviceType;
use rs_matter::dm::clusters::desc::ClusterHandler;
use rs_matter_embassy::matter::dm::clusters::desc;
use rs_matter_embassy::matter::dm::{Endpoint, Node};
use rs_matter_embassy::matter::{clusters, devices};
use rs_matter_embassy::wireless::EmbassyWifiMatterStack;

/// Endpoint ID for the light device
pub const LIGHT_ENDPOINT_ID: u16 = 1;

pub const DEV_TYPE_EXTENDED_COLOR_LIGHT: DeviceType = DeviceType {
    dtype: 0x10D,
    drev: 1,
};

/// Matter Node definition with endpoints
pub const NODE: Node = Node {
    id: 0,
    endpoints: &[
        EmbassyWifiMatterStack::<0, ()>::root_endpoint(),
        Endpoint {
            id: LIGHT_ENDPOINT_ID,
            // Define device type (On/Off Light)
            device_types: devices!(DEV_TYPE_EXTENDED_COLOR_LIGHT),
            // List clusters implemented on this endpoint
            clusters: clusters!(
                desc::DescHandler::CLUSTER, // Descriptor cluster (Mandatory)
                on_off::FULL_CLUSTER,
                level_control::FULL_CLUSTER,
                decl::identify::FULL_CLUSTER,
                decl::color_control::FULL_CLUSTER,
            ),
        },
    ],
};
