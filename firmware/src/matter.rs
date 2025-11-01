use crate::device::LedDeviceLogic;
use rs_matter::dm::clusters::desc::ClusterHandler;
use rs_matter::dm::clusters::on_off::OnOffHooks;
use rs_matter_embassy::matter::dm::clusters::desc;
use rs_matter_embassy::matter::dm::devices::DEV_TYPE_ON_OFF_LIGHT;
use rs_matter_embassy::matter::dm::{Endpoint, Node};
use rs_matter_embassy::matter::{clusters, devices};
use rs_matter_embassy::wireless::EmbassyWifiMatterStack;

/// Endpoint ID for the light device
pub const LIGHT_ENDPOINT_ID: u16 = 1;

/// Matter Node definition with endpoints
pub const NODE: Node = Node {
    id: 0,
    endpoints: &[
        EmbassyWifiMatterStack::<0, ()>::root_endpoint(),
        Endpoint {
            id: LIGHT_ENDPOINT_ID,
            // Define device type (On/Off Light)
            device_types: devices!(DEV_TYPE_ON_OFF_LIGHT),
            // List clusters implemented on this endpoint
            clusters: clusters!(
                desc::DescHandler::CLUSTER, // Descriptor cluster (Mandatory)
                LedDeviceLogic::CLUSTER,    // OnOff cluster
            ),
        },
    ],
};
