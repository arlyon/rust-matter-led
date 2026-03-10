use crate::clusters::{color_control, decl, identify, level_control, on_off};

use rs_matter::dm::DeviceType;
use rs_matter::dm::clusters::desc::ClusterHandler;
use rs_matter::BasicCommData;
use rs_matter_embassy::matter::dm::clusters::desc;
use rs_matter_embassy::matter::dm::{Endpoint, Node};
use rs_matter_embassy::matter::dm::devices::test::TEST_DEV_COMM;
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

/// Default passcode for Matter pairing (matches TEST_DEV_COMM)
pub const DEFAULT_PASSCODE: u32 = 20202021;

/// Generate unique commissioning data based on MAC address
/// This ensures each board has a unique but consistent identity
pub fn generate_commissioning_data(mac: [u8; 6]) -> BasicCommData {
    // Use last 12 bits of MAC for discriminator (0-4095 range)
    let discriminator = u16::from_le_bytes([mac[4], mac[5]]) & 0x0FFF;

    // Create new BasicCommData with the same password as TEST_DEV_COMM
    // but with a unique discriminator based on this device's MAC address
    BasicCommData {
        password: TEST_DEV_COMM.password,
        discriminator,
    }
}

/// Log pairing information for QR code generation
pub fn log_pairing_info(discriminator: u16, passcode: u32) {
    defmt::info!("========================================");
    defmt::info!("Matter Pairing Information:");
    defmt::info!("  Discriminator: {}", discriminator);
    defmt::info!("  Passcode: {}", passcode);
    defmt::info!("========================================");
    defmt::info!("Use a Matter QR code generator with:");
    defmt::info!("  - Discriminator: {}", discriminator);
    defmt::info!("  - Setup PIN Code: {}", passcode);
    defmt::info!("  - Vendor ID: 0xFFF1 (test)");
    defmt::info!("  - Product ID: 0x8000 (test)");
    defmt::info!("========================================");
}
