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
use embassy_sync::mutex::Mutex;
use esp_alloc::heap_allocator;
use rs_matter::{dm::Cluster, with};

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp_hal::{
    Async,
    clock::CpuClock,
    mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig},
    peripherals::MCPWM0,
    time::Rate,
    timer::timg::TimerGroup,
};

use defmt::info;
use defmt_rtt as _;

// --- LED PWM Definitions ---
#[derive(Clone, Copy, Debug, defmt::Format, PartialEq)]
struct LedPwmState {
    r: u8,
    g: u8,
    b: u8,
    cw: u8,
    ww: u8,
}

impl Default for LedPwmState {
    fn default() -> Self {
        Self::new()
    }
}

impl LedPwmState {
    pub const fn new() -> Self {
        Self {
            r: 0,
            g: 0,
            b: 0,
            cw: 0,
            ww: 0,
        }
    }
}

static LED_STATE: Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, LedPwmState> =
    Mutex::new(LedPwmState::new());

// --- Matter Imports ---
use rs_matter_embassy::matter::dm::clusters::on_off::{
    AttributeId, CommandId, FULL_CLUSTER as ON_OFF_FULL_CLUSTER, OnOffHandler, OnOffHooks,
}; // Keep OnOffHooks
use rs_matter_embassy::matter::dm::devices::DEV_TYPE_ON_OFF_LIGHT; // Basic On/Off Light type
use rs_matter_embassy::matter::dm::devices::test::{TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET};
use rs_matter_embassy::matter::dm::{
    Async as MatterAsync, Dataver, EmptyHandler, Endpoint, EpClMatcher, Node,
};
use rs_matter_embassy::matter::error::Error as MatterError;
use rs_matter_embassy::matter::tlv::Nullable;
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::{clusters, devices};
use rs_matter_embassy::stack::persist::DummyKvBlobStore;
use rs_matter_embassy::{epoch::epoch, wireless::EmbassyWifiMatterStack};
use rs_matter_embassy::{
    matter::dm::clusters::desc::{self, ClusterHandler as _},
    wireless::EmbassyWifi,
};
// --- End Matter Imports ---
//
type MatterResult<T> = Result<T, MatterError>;

// Define Heap and Bump sizes
const BUMP_SIZE: usize = 16500;
// ESP32-C6 has unified RAM, so only one heap allocator needed.
// Adjust size based on testing, Matter + TCP/IP + BLE + PWM can be memory intensive.
const HEAP_SIZE: usize = 200 * 1024; // Start with 200KB, adjust as needed

esp_bootloader_esp_idf::esp_app_desc!();

// --- Define a struct that implements OnOffHooks ---
#[derive(Default)]
struct LedDeviceLogic;

impl OnOffHooks for LedDeviceLogic {
    const CLUSTER: Cluster<'static> = ON_OFF_FULL_CLUSTER
        .with_revision(1)
        .with_attrs(with!(
            required;
            AttributeId::OnOff
        ))
        .with_cmds(with!(CommandId::Off | CommandId::On | CommandId::Toggle));

    fn on_off(&self) -> bool {
        todo!()
    }

    fn set_on_off(&self, on: bool) {
        todo!()
    }

    fn start_up_on_off(&self) -> Nullable<rs_matter::dm::clusters::on_off::StartUpOnOffEnum> {
        todo!()
    }

    fn set_start_up_on_off(
        &self,
        value: Nullable<rs_matter::dm::clusters::on_off::StartUpOnOffEnum>,
    ) -> Result<(), MatterError> {
        todo!()
    }

    async fn handle_off_with_effect(
        &self,
        effect: rs_matter::dm::clusters::on_off::EffectVariantEnum,
    ) {
        todo!()
    }

    // async fn on_off_changed(&self, value: bool) -> MatterResult<()> {
    //     info!("OnOff command received: {}", value);
    //     let new_state = if value {
    //         // Define ON state (e.g., Warm White at 50%)
    //         LedPwmState {
    //             r: 0,
    //             g: 0,
    //             b: 0,
    //             cw: 0,
    //             ww: 128,
    //         }
    //     } else {
    //         LedPwmState::default() // OFF
    //     };
    //     let mut state_guard = LED_STATE.lock().await;
    //     *state_guard = new_state;
    //     info!("Set LED target state to: {:?}", new_state);
    //     Ok(())
    // }

    // async fn get_on_off(&self) -> MatterResult<Nullable<bool>> {
    //     let state_guard = LED_STATE.lock().await;
    //     let is_on = state_guard.r != 0
    //         || state_guard.g != 0
    //         || state_guard.b != 0
    //         || state_guard.cw != 0
    //         || state_guard.ww != 0;
    //     Ok(Nullable::NotNull(is_on))
    // }
}
// --- End OnOffHooks implementation ---

// --- Define the PWM update task ---
#[embassy_executor::task]
async fn pwm_task(mut pwm: McPwm<'static, MCPWM0<'static>>) {
    // Added Async marker
    info!("PWM task started");
    let mut ticker = Ticker::every(Duration::from_millis(20)); // ~50 Hz update rate

    // ESP-HAL PWM objects hold internal state, no need to access timer/operators directly in loop
    // Calculation needs to happen once based on config
    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_khz(20u32)).unwrap();
    let max_duty = clock_cfg.frequency().as_hz() / (2 * 20_000); // For UpDown mode
    info!("PWM Max Duty: {}", max_duty);

    let mut last_applied_state = LedPwmState::default();

    loop {
        ticker.next().await;
        let target_state = *LED_STATE.lock().await;

        if target_state != last_applied_state {
            info!("Applying PWM state: {:?}", target_state);
            let scale = |val: u8| (val as u32 * max_duty / 255) as u16;

            // Set duty cycles - Adjust comparisons based on pin connections to operators
            pwm.operator0.set_comparison_a(scale(target_state.r)); // Assumes R on Op0 A
            pwm.operator0.set_comparison_b(scale(target_state.g)); // Assumes G on Op0 B
            pwm.operator1.set_comparison_a(scale(target_state.b)); // Assumes B on Op1 A
            pwm.operator1.set_comparison_b(scale(target_state.cw)); // Assumes CW on Op1 B
            pwm.operator2.set_comparison_a(scale(target_state.ww)); // Assumes WW on Op2 A

            last_applied_state = target_state;
        }
    }
}
// --- End PWM update task ---

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_print!();
    info!("Starting Matter + Direct PWM example (ESP32-C6)...");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // --- Select your 5 PWM GPIO pins for ESP32-C6 ---
    // IMPORTANT: Check the ESP32-C6 datasheet for valid MCPWM output pins!
    // Using GPIO0-GPIO4 as placeholders:
    let pin_r = io.pins.gpio0.into_push_pull_output();
    let pin_g = io.pins.gpio1.into_push_pull_output();
    let pin_b = io.pins.gpio2.into_push_pull_output();
    let pin_cw = io.pins.gpio3.into_push_pull_output();
    let pin_ww = io.pins.gpio4.into_push_pull_output();
    // --- End Pin Selection ---

    // Initialize Heap
    heap_allocator!(size: HEAP_SIZE);

    // Initialize Embassy Timer Driver
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    // Note: C6 might use different interrupt setup, esp_rtos::start handles it.
    // Check esp-hal examples for specific C6 interrupt setup if needed.
    // For C6 (RISC-V), SW_INTERRUPT might not be the standard, but esp_rtos handles it.
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT); // Use SYSTEM for SW interrupt on C6
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    // Initialize Wi-Fi/BLE Radio Controller
    let init = esp_radio::init().expect("Failed to initialize radio controller");

    // --- PWM Initialization ---
    info!("Initializing MCPWM...");
    // Assuming SYSTIMER is a valid clock source for MCPWM on C6, check esp-hal if unsure.
    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_khz(20)).unwrap();
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg); // Use .into_async()

    // Configure Operators and connect pins (adjust assignments as needed)
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    mcpwm
        .operator0
        .connect_pin_a(pin_r, PwmPinConfig::new_active_high());
    mcpwm
        .operator0
        .connect_pin_b(pin_g, PwmPinConfig::new_active_high());

    mcpwm.operator1.set_timer(&mcpwm.timer0);
    mcpwm
        .operator1
        .connect_pin_a(pin_b, PwmPinConfig::new_active_high());
    mcpwm
        .operator1
        .connect_pin_b(pin_cw, PwmPinConfig::new_active_high());

    mcpwm.operator2.set_timer(&mcpwm.timer0);
    mcpwm
        .operator2
        .connect_pin_a(pin_ww, PwmPinConfig::new_active_high());
    // Operator 2 B is unused

    info!("MCPWM initialized.");
    // --- End PWM Initialization ---

    // == Matter Stack Initialization ==
    let stack =
        &*Box::leak(Box::new_uninit()).init_with(EmbassyWifiMatterStack::<BUMP_SIZE, ()>::init(
            &TEST_DEV_DET,
            TEST_DEV_COMM,
            &TEST_DEV_ATT,
            epoch,
            esp_rand,
        ));

    // == Matter Device Definition (Using custom OnOff logic) ==
    let on_off_handler = OnOffHandler::new(
        Dataver::new(0),
        OnOffData::new(Dataver::new_rand(stack.matter().rand())), // Initialize OnOff cluster data
        LedDeviceLogic::default(),                                // Use our custom logic struct
    );

    // Chain clusters for the endpoint
    let handler = EmptyHandler
        .chain(
            EpClMatcher::new(
                // Match OnOff cluster on our endpoint
                Some(LIGHT_ENDPOINT_ID),
                Some(desc::DescHandler::CLUSTER.id), // Use the cluster ID
            ),
            MatterAsync(desc::DescHandler::new(Dataver::new_rand(matter.rand())).adapt()), // Wrap our handler in Async
        )
        .chain(
            // Add the mandatory Descriptor cluster
            EpClMatcher::new(Some(LIGHT_ENDPOINT_ID), Some(desc::DescHandler::CLUSTER.id)),
            MatterAsync(desc::DescHandler::new(Dataver::new_rand(stack.matter().rand())).adapt()),
        );

    // Persistence (Dummy for now)
    let persist = stack
        .create_persist_with_comm_window(DummyKvBlobStore)
        .await
        .unwrap();

    // == Spawn the PWM Task ==
    // Pass the initialized MCPWM peripheral to the task
    spawner.spawn(pwm_task(mcpwm)).unwrap();
    info!("Spawned PWM task");

    // == Run the Matter Stack ==
    info!("Running Matter stack...");
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

    matter.await; // This runs forever

    unreachable!();
}

// Matter Node definition
const LIGHT_ENDPOINT_ID: u16 = 1;
const NODE: Node = Node {
    id: 0,
    endpoints: &[
        EmbassyWifiMatterStack::<0, ()>::root_endpoint(),
        Endpoint {
            id: LIGHT_ENDPOINT_ID,
            // Define device type (On/Off Light is simple to start)
            device_types: devices!(DEV_TYPE_ON_OFF_LIGHT),
            // List clusters implemented on this endpoint
            clusters: clusters!(
                desc::DescHandler::CLUSTER, // Descriptor cluster (Mandatory)
                LedDeviceLogic::CLUSTER,    // OnOff cluster
                                            // Add more cluster IDs here later (e.g., LevelControl, ColorControl)
            ),
        },
    ],
};
