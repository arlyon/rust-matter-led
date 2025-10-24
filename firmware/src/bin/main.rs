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
use rs_matter::{
    dm::{Cluster, clusters::on_off::HandlerAsyncAdaptor},
    with,
};

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp_hal::{
    clock::CpuClock,
    gpio::Io,
    mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode},
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
use rs_matter_embassy::wireless::esp::EspWifiDriver;
use rs_matter_embassy::{epoch::epoch, wireless::EmbassyWifiMatterStack};
use rs_matter_embassy::{
    matter::dm::clusters::desc::{self, ClusterHandler as _},
    wireless::EmbassyWifi,
};
// --- End Matter Imports ---

// Define Heap and Bump sizes
const BUMP_SIZE: usize = 16500;
// ESP32-C6 has unified RAM, so only one heap allocator needed.
// Adjust size based on testing, Matter + TCP/IP + BLE + PWM can be memory intensive.
const HEAP_SIZE: usize = 200 * 1024; // Start with 200KB, adjust as needed

esp_bootloader_esp_idf::esp_app_desc!();

#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));
    loop {}
}

// --- Define a struct that implements OnOffHooks ---
#[derive(Default)]
struct LedDeviceLogic;

impl LedDeviceLogic {
    fn get_current_on_off() -> bool {
        // We need to check this synchronously, so we use try_lock
        // In a real implementation, you might want to store this state separately
        if let Ok(state_guard) = LED_STATE.try_lock() {
            state_guard.r != 0
                || state_guard.g != 0
                || state_guard.b != 0
                || state_guard.cw != 0
                || state_guard.ww != 0
        } else {
            false // Default to off if we can't get the lock
        }
    }
}

impl OnOffHooks for LedDeviceLogic {
    const CLUSTER: Cluster<'static> = ON_OFF_FULL_CLUSTER
        .with_revision(1)
        .with_attrs(with!(
            required;
            AttributeId::OnOff
        ))
        .with_cmds(with!(CommandId::Off | CommandId::On | CommandId::Toggle));

    fn on_off(&self) -> bool {
        Self::get_current_on_off()
    }

    fn set_on_off(&self, on: bool) {
        info!("OnOff command received: {}", on);
        let new_state = if on {
            // Define ON state (e.g., Warm White at 50%)
            LedPwmState {
                r: 0,
                g: 0,
                b: 0,
                cw: 0,
                ww: 128,
            }
        } else {
            LedPwmState::default() // OFF
        };

        // Update the LED state
        if let Ok(mut state_guard) = LED_STATE.try_lock() {
            *state_guard = new_state;
            info!("Set LED target state to: {:?}", new_state);
        } else {
            defmt::warn!("Could not acquire LED_STATE lock to set on/off");
        }
    }

    fn start_up_on_off(&self) -> Nullable<rs_matter::dm::clusters::on_off::StartUpOnOffEnum> {
        // Return null to indicate we don't support startup on/off configuration
        Nullable::none()
    }

    fn set_start_up_on_off(
        &self,
        _value: Nullable<rs_matter::dm::clusters::on_off::StartUpOnOffEnum>,
    ) -> Result<(), MatterError> {
        // We don't support configuring startup behavior, but return Ok to not cause errors
        Ok(())
    }

    async fn handle_off_with_effect(
        &self,
        _effect: rs_matter::dm::clusters::on_off::EffectVariantEnum,
    ) {
        // For now, just turn off immediately
        // You could implement fade effects here based on the effect parameter
        self.set_on_off(false);
    }
}
// --- End OnOffHooks implementation ---

// PWM pin holder struct
struct PwmPins {
    pin_r: esp_hal::mcpwm::operator::PwmPin<'static, MCPWM0<'static>, 0, true>,
    pin_g: esp_hal::mcpwm::operator::PwmPin<'static, MCPWM0<'static>, 0, false>,
    pin_b: esp_hal::mcpwm::operator::PwmPin<'static, MCPWM0<'static>, 1, true>,
    pin_cw: esp_hal::mcpwm::operator::PwmPin<'static, MCPWM0<'static>, 1, false>,
    pin_ww: esp_hal::mcpwm::operator::PwmPin<'static, MCPWM0<'static>, 2, true>,
}

// --- Define the PWM update task ---
#[embassy_executor::task]
async fn pwm_task(mut pwm_pins: PwmPins, period: u16) {
    info!("PWM task started");
    let mut ticker = Ticker::every(Duration::from_millis(20)); // ~50 Hz update rate

    info!("PWM Period: {}", period);

    let mut last_applied_state = LedPwmState::default();

    loop {
        ticker.next().await;
        let target_state = *LED_STATE.lock().await;

        if target_state != last_applied_state {
            info!("Applying PWM state: {:?}", target_state);
            let scale = |val: u8| (val as u32 * period as u32 / 255) as u16;

            // Set duty cycles using timestamps
            pwm_pins.pin_r.set_timestamp(scale(target_state.r));
            pwm_pins.pin_g.set_timestamp(scale(target_state.g));
            pwm_pins.pin_b.set_timestamp(scale(target_state.b));
            pwm_pins.pin_cw.set_timestamp(scale(target_state.cw));
            pwm_pins.pin_ww.set_timestamp(scale(target_state.ww));

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
    let _io = Io::new(peripherals.IO_MUX);

    // --- Select your 5 PWM GPIO pins for ESP32-C6 ---
    // IMPORTANT: Check the ESP32-C6 datasheet for valid MCPWM output pins!
    // Using GPIO0-GPIO4 as placeholders:
    let pin_r = peripherals.GPIO0;
    let pin_g = peripherals.GPIO1;
    let pin_b = peripherals.GPIO2;
    let pin_cw = peripherals.GPIO3;
    let pin_ww = peripherals.GPIO4;
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
    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_khz(20)).unwrap();
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);

    // Configure timer - using period of 99 for 0-99 range (like 0-100%)
    let period = 99u16;
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(period, PwmWorkingMode::Increase, Rate::from_khz(20))
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    // Configure Operators and connect pins
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    let (pin_r, pin_g) = mcpwm.operator0.with_pins(
        pin_r,
        PwmPinConfig::UP_ACTIVE_HIGH,
        pin_g,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );

    mcpwm.operator1.set_timer(&mcpwm.timer0);
    let (pin_b, pin_cw) = mcpwm.operator1.with_pins(
        pin_b,
        PwmPinConfig::UP_ACTIVE_HIGH,
        pin_cw,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );

    mcpwm.operator2.set_timer(&mcpwm.timer0);
    let pin_ww = mcpwm
        .operator2
        .with_pin_a(pin_ww, PwmPinConfig::UP_ACTIVE_HIGH);

    let pwm_pins = PwmPins {
        pin_r,
        pin_g,
        pin_b,
        pin_cw,
        pin_ww,
    };

    info!("MCPWM initialized.");
    // --- End PWM Initialization ---

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

    let stack =
        &*Box::leak(Box::new_uninit()).init_with(EmbassyWifiMatterStack::<BUMP_SIZE, ()>::init(
            &TEST_DEV_DET,
            TEST_DEV_COMM,
            &TEST_DEV_ATT,
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

    // Persistence (Dummy for now)
    let persist = stack
        .create_persist_with_comm_window(DummyKvBlobStore)
        .await
        .unwrap();

    // == Spawn the PWM Task ==
    // Pass the PWM pins to the task
    spawner.spawn(pwm_task(pwm_pins, period)).unwrap();
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

    matter.await.expect("matter stack crashed"); // This runs forever

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
