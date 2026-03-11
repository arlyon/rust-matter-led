#![no_std]
#![no_main]
#![recursion_limit = "256"]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types"
)]
#![feature(alloc_error_handler)]
#![feature(type_alias_impl_trait)]

extern crate alloc;

use core::pin::pin;
use embedded_storage_async::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use esp_alloc::heap_allocator;
use esp_storage::{FlashStorage, FlashStorageError};

use rs_matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::dm::devices::test::{TEST_DEV_ATT, TEST_DEV_DET};

use embassy_executor::Spawner;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, Io, Pull},
    peripherals::SW_INTERRUPT,
    ram,
    rng::Rng,
    timer::timg::TimerGroup,
};

use esp_backtrace as _;
use esp_println as _;

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

use esp_metadata_generated::memory_range;
use firmware::clusters::color_control::ColorControlHandler;
use firmware::clusters::*;
use firmware::device::LedDeviceLogic;
use firmware::matter::{LIGHT_ENDPOINT_ID, NODE};

// we can reclaim RAM from the bootloader!!!
const RECLAIMED_RAM: usize =
    memory_range!("DRAM2_UNINIT").end - memory_range!("DRAM2_UNINIT").start;

const BUMP_SIZE: usize = 24 * 1024; //  typical peak is about 16kb
const HEAP_SIZE: usize = 112 * 1024; // typical peak is about 72kb

// Storage configuration (must match between boot cycles)
// we have 8MB flash, so lets 'reserve' the last 1MB for the storage
const STORAGE_START: u32 = 0x700_000; // Start at a safe offset in flash
const STORAGE_SIZE: u32 = 32 * 4096; // 32 sectors = 128KB

esp_bootloader_esp_idf::esp_app_desc!();

pub struct MatterRng(pub esp_hal::rng::Rng);

static STATIC_CELL: static_cell::StaticCell<EmbassyWifiMatterStack<BUMP_SIZE, ()>> =
    static_cell::StaticCell::new();

impl rand_core::RngCore for MatterRng {
    fn next_u32(&mut self) -> u32 {
        // esp_hal::rng::Rng has a random() method that returns u32
        self.0.random()
    }

    fn next_u64(&mut self) -> u64 {
        let mut buf = [0u8; 8];
        self.fill_bytes(&mut buf);
        u64::from_le_bytes(buf)
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        for chunk in dest.chunks_mut(4) {
            let rand_val = self.0.random().to_le_bytes();
            let len = chunk.len();
            chunk.copy_from_slice(&rand_val[..len]);
        }
    }
}

// rs_matter requires this marker trait to accept the RNG
impl rand_core::CryptoRng for MatterRng {}

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

#[alloc_error_handler]
fn alloc_error(layout: core::alloc::Layout) -> ! {
    defmt::panic!(
        "OOM: alloc of {} bytes, align {}",
        layout.size(),
        layout.align()
    );
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    defmt::info!("Starting Matter + Direct PWM example (ESP32-C6)...");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let _io = Io::new(peripherals.IO_MUX);

    // Initialize Heap
    heap_allocator!(size: HEAP_SIZE - RECLAIMED_RAM);
    heap_allocator!(#[ram(reclaimed)] size: RECLAIMED_RAM);

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
    use esp_hal::interrupt::software::SoftwareInterruptControl;
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, software_interrupt.software_interrupt0);

    defmt::info!("Embassy initialized!");

    // Initialize LEDC for 5-channel LED control
    let ledc_channels = firmware::led::pwm::init_ledc(
        peripherals.LEDC,
        peripherals.GPIO1,  // Red
        peripherals.GPIO0,  // Green
        peripherals.GPIO10, // Blue
        peripherals.GPIO3,  // Cold White
        peripherals.GPIO2,  // Warm White
    );

    // Spawn PWM task
    spawner.spawn(firmware::led::pwm::pwm_task(ledc_channels).unwrap());
    defmt::info!("PWM task spawned");

    embassy_futures::yield_now().await;

    // Get MAC address for unique identity
    let mac = esp_hal::efuse::Efuse::mac_address();
    defmt::info!(
        "Device MAC: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5]
    );

    // Generate unique commissioning data based on MAC
    let comm_data = firmware::matter::generate_commissioning_data(mac);
    firmware::matter::log_pairing_info(comm_data.discriminator, firmware::matter::DEFAULT_PASSCODE);

    let stack = STATIC_CELL
        .uninit()
        .init_with(EmbassyWifiMatterStack::<BUMP_SIZE, ()>::init(
            &TEST_DEV_DET,
            comm_data,
            &TEST_DEV_ATT,
            epoch,
        ));

    // Initialize the hardware RNG and wrap it to satisfy rs_matter's rand_core 0.9.x trait bounds
    let hw_rng = Rng::new();
    let matter_rng = MatterRng(hw_rng);

    let crypto =
        rs_matter::crypto::default_crypto::<embassy_sync::blocking_mutex::raw::NoopRawMutex, _>(
            rs_matter_stack::rand::reseeding_csprng(matter_rng, 1000).unwrap(),
            rs_matter::dm::devices::test::DAC_PRIVKEY,
        );

    use rs_matter::crypto::Crypto;

    let mut weak_rand = crypto.weak_rand().unwrap();

    // == Matter Device Definition (Using custom OnOff logic) ==
    let on_off_handler = OnOffHandler::new_standalone(
        Dataver::new_rand(&mut weak_rand),
        LIGHT_ENDPOINT_ID,
        LedDeviceLogic, // Use our custom logic struct
    );

    let identify_handler = firmware::clusters::identify::IdentifyHandler::new(
        Dataver::new_rand(&mut weak_rand),
        LedDeviceLogic,
    );

    let color_control_handler =
        ColorControlHandler::new(Dataver::new_rand(&mut weak_rand), LedDeviceLogic);

    let level_control_handler = LevelControlHandler::new_standalone(
        Dataver::new_rand(&mut weak_rand),
        LIGHT_ENDPOINT_ID,
        LedDeviceLogic,
        Default::default(),
    );

    // TODO: figure this out, and use new instead of new_standalone
    // on_off_handler.init(Some(&level_control_handler));
    // level_control_handler.init(Some(&on_off_handler));

    // Chain clusters for the endpoint
    let handler = EmptyHandler
        .chain(
            EpClMatcher::new(
                // Match OnOff cluster on our endpoint
                Some(LIGHT_ENDPOINT_ID),
                Some(firmware::clusters::ON_OFF_FULL_CLUSTER.id), // Use the cluster ID
            ),
            on_off_handler.adapt(),
        )
        .chain(
            EpClMatcher::new(
                Some(LIGHT_ENDPOINT_ID),
                Some(firmware::clusters::identify::CLUSTER_ID),
            ),
            identify_handler.adapt(),
        )
        .chain(
            EpClMatcher::new(
                Some(LIGHT_ENDPOINT_ID),
                Some(firmware::clusters::color_control::CLUSTER_ID),
            ),
            color_control_handler.adapt(),
        )
        .chain(
            EpClMatcher::new(
                Some(LIGHT_ENDPOINT_ID),
                Some(firmware::clusters::LEVEL_CONTROL_FULL_CLUSTER.id),
            ),
            level_control_handler.adapt(),
        )
        .chain(
            // Add the mandatory Descriptor cluster
            EpClMatcher::new(Some(LIGHT_ENDPOINT_ID), Some(desc::DescHandler::CLUSTER.id)),
            MatterAsync(desc::DescHandler::new(Dataver::new_rand(&mut weak_rand)).adapt()),
        );

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
        .create_persist_with_comm_window(&crypto, kv_store)
        .await
        .unwrap();

    // == Run the Matter Stack ==
    defmt::info!("Running Matter stack...");
    let matter = pin!(stack.run_coex(
        EmbassyWifi::new(
            // C6 uses WIFI and BLE peripherals
            EspWifiDriver::new(peripherals.WIFI, peripherals.BT),
            weak_rand,
            true,
            stack,
        ),
        &persist,
        &crypto,
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
