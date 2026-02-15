#![no_std]
#![no_main]
#![recursion_limit = "256"]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types"
)]

extern crate alloc;

use alloc::boxed::Box;
use core::pin::pin;
use embedded_storage_async::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use esp_alloc::heap_allocator;
use esp_storage::{FlashStorage, FlashStorageError};
use rs_matter::{
    BasicCommData,
    dm::{
        clusters::{basic_info::BasicInfoConfig, on_off::HandlerAsyncAdaptor},
        devices::test::{TEST_PID, TEST_VID},
    },
    pairing::{DiscoveryCapabilities, qr::QrTextType},
};

use embassy_executor::Spawner;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, Io, Pull},
    timer::timg::TimerGroup,
};

use esp_backtrace as _;
use esp_println as _;

use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::wireless::esp::EspWifiDriver;
use rs_matter_embassy::{epoch::epoch, wireless::EmbassyWifiMatterStack};
use rs_matter_embassy::{
    matter::dm::clusters::desc::{self, ClusterHandler as _},
    wireless::EmbassyWifi,
};
use rs_matter_embassy::{
    matter::dm::{Async as MatterAsync, Dataver, EmptyHandler, EpClMatcher},
    persist::EmbassyKvBlobStore,
};

use firmware::clusters::*;
use firmware::device::LedDeviceLogic;
use firmware::led::pwm::{PwmConfig, init_pwm, pwm_task};
use firmware::matter::{LIGHT_ENDPOINT_ID, NODE};

// Define Heap and Bump sizes
const BUMP_SIZE: usize = 65536; // 64KB for Matter stack operations (certificates, TLV encoding, etc.)
// ESP32-C6 has unified RAM, so only one heap allocator needed.
// Adjust size based on testing, Matter + TCP/IP + BLE + PWM can be memory intensive.
const HEAP_SIZE: usize = 200 * 1024; // Start with 200KB, adjust as needed

// Storage configuration (must match between boot cycles)
const STORAGE_START: u32 = 0x9000; // Start at a safe offset in flash
const STORAGE_SIZE: u32 = 32 * 4096; // 32 sectors = 128KB

esp_bootloader_esp_idf::esp_app_desc!();

/// Factory reset: erase the Matter storage and reboot
async fn factory_reset(mut flash: BlockingFlashStorage<'_>) {
    use embedded_storage::nor_flash::NorFlash as _;

    defmt::warn!("Factory reset initiated!");
    defmt::info!(
        "Erasing flash storage from 0x{:x} to 0x{:x}",
        STORAGE_START,
        STORAGE_START + STORAGE_SIZE
    );

    // Erase the storage area in sector-sized chunks
    if let Err(e) = flash.0.erase(STORAGE_START, STORAGE_START + STORAGE_SIZE) {
        defmt::error!("Failed to erase flash: {:?}", e);
    } else {
        defmt::info!("Flash erased successfully!");
    }

    defmt::info!("Rebooting in 2 seconds...");
    embassy_time::Timer::after(embassy_time::Duration::from_secs(2)).await;

    esp_hal::system::software_reset();
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    defmt::info!("Starting Matter + Direct PWM example (ESP32-C6)...");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let io = Io::new(peripherals.IO_MUX);

    // Initialize Heap
    heap_allocator!(size: HEAP_SIZE);

    // Check if BOOT button (GPIO9) is held down for factory reset
    let boot_button = Input::new(
        peripherals.GPIO9,
        esp_hal::gpio::InputConfig::default().with_pull(Pull::Up),
    );
    let factory_reset_requested = boot_button.is_low();

    if factory_reset_requested {
        defmt::warn!("BOOT button held - Factory reset will be performed after initialization");
    }

    // Initialize Embassy Timer Driver
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    defmt::info!("Embassy initialized!");

    // --- PWM Initialization ---
    let (pwm_pins, period) = init_pwm(
        peripherals.MCPWM0,
        PwmConfig::default(),
        peripherals.GPIO3,  // R
        peripherals.GPIO2,  // G
        peripherals.GPIO10, // B
        peripherals.GPIO1,  // CW
        peripherals.GPIO0,  // WW
    )
    .expect("Failed to initialize PWM");

    // Spawn the PWM Task
    spawner.spawn(pwm_task(pwm_pins, period)).unwrap();
    defmt::info!("Spawned PWM task");

    // Initialize Wi-Fi/BLE Radio Controller
    let init = esp_radio::init().expect("Failed to initialize radio controller");

    // == Matter Stack Initialization ==
    // Custom rand function for esp-hal
    fn esp_rand(buf: &mut [u8]) {
        // TODO: Implement proper random number generation using ESP32 hardware RNG
        // For now, use a simple counter-based approach (NOT CRYPTOGRAPHICALLY SECURE)
        static mut COUNTER: u32 = 0;
        for byte in buf.iter_mut() {
            unsafe {
                COUNTER = COUNTER.wrapping_add(1);
                *byte = (COUNTER & 0xFF) as u8;
            }
        }
    }

    let info: &'static _ = Box::leak(Box::new(BasicInfoConfig {
        vid: TEST_VID,
        pid: TEST_PID,
        hw_ver: 1,
        hw_ver_str: "1",
        sw_ver: 1,
        sw_ver_str: "1",
        serial_no: "123456789",
        product_name: "ACME Test",
        vendor_name: "ACME",
        device_name: "MyTest",
        ..BasicInfoConfig::new()
    }));

    let stack =
        &*Box::leak(Box::new_uninit()).init_with(EmbassyWifiMatterStack::<BUMP_SIZE, ()>::init(
            &info,
            BasicCommData {
                discriminator: 0x0F00, // 12-bit value (0-4095)
                password: 20202021,    // 27-bit passcode (must be formatted as 8 decimal digits)
            },
            &firmware::TestDevAtt(()),
            epoch,
            esp_rand,
        ));

    // == Matter Device Definition (Using custom OnOff logic) ==
    let on_off_handler = OnOffHandler::new_standalone(
        Dataver::new_rand(stack.matter().rand()),
        LIGHT_ENDPOINT_ID,
        LedDeviceLogic, // Use our custom logic struct
    );

    // Chain clusters for the endpoint
    let handler = EmptyHandler
        .chain(
            EpClMatcher::new(
                // Match OnOff cluster on our endpoint
                Some(LIGHT_ENDPOINT_ID),
                Some(LedDeviceLogic::CLUSTER.id), // Use the cluster ID
            ),
            HandlerAsyncAdaptor(&on_off_handler),
        )
        .chain(
            // Add the mandatory Descriptor cluster
            EpClMatcher::new(Some(LIGHT_ENDPOINT_ID), Some(desc::DescHandler::CLUSTER.id)),
            MatterAsync(desc::DescHandler::new(Dataver::new_rand(stack.matter().rand())).adapt()),
        );

    if !stack.matter().is_commissioned() {
        let matter = stack.matter();
        matter
            .print_standard_qr_code(QrTextType::Unicode, DiscoveryCapabilities::IP)
            .unwrap();
        defmt::info!("ready to commission");
    } else {
        defmt::info!("commissioned already");
    }

    let storage = esp_storage::FlashStorage::new(peripherals.FLASH);
    let storage = BlockingFlashStorage(storage);

    // Handle factory reset if BOOT button was held during startup
    if factory_reset_requested {
        factory_reset(storage).await;
        unreachable!();
    }

    // sequential-storage requires:
    // - Start/end aligned to ERASE_SIZE (4096 bytes for ESP32)
    // - At least 2 sectors (8192 bytes minimum)
    // Using 32 sectors (128KB) for Matter key-value storage (fabrics, certs, ACLs, etc.)
    let kv_store = EmbassyKvBlobStore::new(storage, STORAGE_START..(STORAGE_START + STORAGE_SIZE));

    // Persistence (Dummy for now)
    let persist = stack
        .create_persist_with_comm_window(kv_store)
        .await
        .unwrap();

    // == Run the Matter Stack ==
    defmt::info!("Running Matter stack...");
    let matter = pin!(stack.run_coex(
        EmbassyWifi::new(
            // C6 uses WIFI and BLE peripherals
            EspWifiDriver::new(&init, peripherals.WIFI, peripherals.BT),
            stack
        ),
        &persist,
        (NODE, handler), // Use the handler chain we built
        (),
    ));

    match matter.await {
        Ok(_) => {
            unreachable!();
        }
        Err(e) => {
            defmt::error!("Matter error: {:?}", e);
            panic!();
        }
    }
}

struct BlockingFlashStorage<'a>(FlashStorage<'a>);

impl ErrorType for BlockingFlashStorage<'_> {
    type Error = FlashStorageError;
}

impl NorFlash for BlockingFlashStorage<'_> {
    const WRITE_SIZE: usize = FlashStorage::WORD_SIZE as _;

    const ERASE_SIZE: usize = FlashStorage::SECTOR_SIZE as _;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        embedded_storage::nor_flash::NorFlash::erase(&mut self.0, from, to)
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        embedded_storage::nor_flash::NorFlash::write(&mut self.0, offset, bytes)
    }
}

impl ReadNorFlash for BlockingFlashStorage<'_> {
    const READ_SIZE: usize = FlashStorage::WORD_SIZE as _;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        embedded_storage::nor_flash::ReadNorFlash::read(&mut self.0, offset, bytes)
    }

    fn capacity(&self) -> usize {
        embedded_storage::nor_flash::ReadNorFlash::capacity(&self.0)
    }
}

/// The esp storage is synchronous, so we just provide an async wrapper.
///
/// THIS WILL STILL BLOCK
/// see: https://github.com/esp-rs/esp-storage/issues/39#issuecomment-1980991446
impl embedded_storage_async::nor_flash::MultiwriteNorFlash for BlockingFlashStorage<'_> {}
