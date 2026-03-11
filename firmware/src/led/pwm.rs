use super::{LED_STATE, LedPwmState, TARGET_STATE, TargetState};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};
use esp_hal::{
    mcpwm::{
        FrequencyError, McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode,
    },
    peripherals::MCPWM0,
    time::Rate,
};

/// Signal to trigger identify sequence
pub static IDENTIFY_SIGNAL: Signal<CriticalSectionRawMutex, u16> = Signal::new();

/// PWM pin holder struct
pub struct PwmPins {
    pub pin_r: esp_hal::mcpwm::operator::PwmPin<'static, MCPWM0<'static>, 0, true>,
    pub pin_g: esp_hal::mcpwm::operator::PwmPin<'static, MCPWM0<'static>, 0, false>,
    pub pin_b: esp_hal::mcpwm::operator::PwmPin<'static, MCPWM0<'static>, 1, true>,
    pub pin_cw: esp_hal::mcpwm::operator::PwmPin<'static, MCPWM0<'static>, 1, false>,
    pub pin_ww: esp_hal::mcpwm::operator::PwmPin<'static, MCPWM0<'static>, 2, true>,
}

/// Configuration for PWM initialization
pub struct PwmConfig {
    /// PWM frequency in kHz (e.g., 20 for 20kHz)
    pub frequency_khz: u32,
    /// PWM period (number of steps, e.g., 999 for 0-999 range)
    pub period: u16,
    /// Clock frequency in MHz (e.g., 32 for 32MHz)
    pub clock_mhz: u32,
}

impl Default for PwmConfig {
    fn default() -> Self {
        Self {
            frequency_khz: 20,
            period: 999,
            clock_mhz: 32,
        }
    }
}

/// Initialize MCPWM peripheral and return configured pins
pub fn init_pwm(
    mcpwm: MCPWM0<'static>,
    config: PwmConfig,
    pin_r: impl esp_hal::gpio::OutputPin + 'static,
    pin_g: impl esp_hal::gpio::OutputPin + 'static,
    pin_b: impl esp_hal::gpio::OutputPin + 'static,
    pin_cw: impl esp_hal::gpio::OutputPin + 'static,
    pin_ww: impl esp_hal::gpio::OutputPin + 'static,
) -> Result<(PwmPins, u16), FrequencyError> {
    defmt::info!("Initializing MCPWM...");

    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(config.clock_mhz))?;
    let mut mcpwm = McPwm::new(mcpwm, clock_cfg);

    // Configure timer for specified PWM frequency with given period
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(
            config.period,
            PwmWorkingMode::Increase,
            Rate::from_khz(config.frequency_khz),
        )
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

    defmt::info!("MCPWM initialized with period: {}", config.period);

    Ok((pwm_pins, config.period))
}

/// PWM update task - reads LED state from TARGET_STATE signal and updates PWM outputs
/// Also handles identify sequences which take priority and run uninterrupted
#[embassy_executor::task]
pub async fn pwm_task(mut pwm_pins: PwmPins, period: u16) {
    defmt::info!("PWM task started. FORCING 50% DUTY CYCLE FOR 10s...");

    // 50% duty cycle math: period / 2
    let test_val = period / 2;

    // Force pins to toggle immediately
    pwm_pins.pin_r.set_timestamp(test_val);
    pwm_pins.pin_g.set_timestamp(test_val);
    pwm_pins.pin_b.set_timestamp(test_val);
    pwm_pins.pin_cw.set_timestamp(test_val);
    pwm_pins.pin_ww.set_timestamp(test_val);

    // Wait 10 seconds to let you probe with the multimeter
    Timer::after(Duration::from_secs(10)).await;

    defmt::info!("Diagnostic over. Entering normal loop.");

    // defmt::info!("PWM task started with period: {}", period);

    // let mut last_applied_state = LedPwmState::default();

    // // Helper to apply a PWM state
    // let apply_state = |pins: &mut PwmPins, state: &LedPwmState| {
    //     let scale = |val: u8| ((val as u32 * period as u32) / 255) as u16;
    //     pins.pin_r.set_timestamp(scale(state.r));
    //     pins.pin_g.set_timestamp(scale(state.g));
    //     pins.pin_b.set_timestamp(scale(state.b));
    //     pins.pin_cw.set_timestamp(scale(state.cw));
    //     pins.pin_ww.set_timestamp(scale(state.ww));
    // };

    // loop {
    //     // Check for identify signal first (higher priority)
    //     if let Some(time_seconds) = IDENTIFY_SIGNAL.try_take() {
    //         if time_seconds > 0 {
    //             defmt::info!("Identify sequence triggered for {} seconds", time_seconds);

    //             // Save current state to restore later
    //             let saved_state = TARGET_STATE.peek();

    //             // RED phase (1 second)
    //             defmt::info!("Identify: RED");
    //             apply_state(
    //                 &mut pwm_pins,
    //                 &LedPwmState {
    //                     r: 255,
    //                     g: 0,
    //                     b: 0,
    //                     cw: 0,
    //                     ww: 0,
    //                 },
    //             );
    //             Timer::after(Duration::from_secs(1)).await;

    //             // GREEN phase (1 second)
    //             defmt::info!("Identify: GREEN");
    //             apply_state(
    //                 &mut pwm_pins,
    //                 &LedPwmState {
    //                     r: 0,
    //                     g: 255,
    //                     b: 0,
    //                     cw: 0,
    //                     ww: 0,
    //                 },
    //             );
    //             Timer::after(Duration::from_secs(1)).await;

    //             // BLUE phase (1 second)
    //             defmt::info!("Identify: BLUE");
    //             apply_state(
    //                 &mut pwm_pins,
    //                 &LedPwmState {
    //                     r: 0,
    //                     g: 0,
    //                     b: 255,
    //                     cw: 0,
    //                     ww: 0,
    //                 },
    //             );
    //             Timer::after(Duration::from_secs(1)).await;

    //             defmt::info!("Identify sequence complete");

    //             // Restore saved state if it existed
    //             if let Some(saved) = saved_state {
    //                 defmt::info!("Restoring previous state: {:?}", saved.target);
    //                 apply_state(&mut pwm_pins, &saved.target);
    //                 last_applied_state = saved.target;
    //             } else {
    //                 // No previous state, turn off
    //                 defmt::info!("No previous state, turning off");
    //                 apply_state(&mut pwm_pins, &LedPwmState::default());
    //                 last_applied_state = LedPwmState::default();
    //             }

    //             continue;
    //         }
    //     }

    //     // Wait for a new target state
    //     let target = TARGET_STATE.wait("pwm").await;

    //     defmt::info!(
    //         "PWM task received new target: {:?}, transition: {}ms",
    //         target.target,
    //         target.transition_duration_ms
    //     );

    //     // For now, apply instantly (animation/interpolation will be added in task #12)
    //     if target.target != last_applied_state {
    //         defmt::info!("Applying PWM state: {:?}", target.target);
    //         apply_state(&mut pwm_pins, &target.target);
    //         last_applied_state = target.target;

    //         // Also update LED_STATE for backwards compatibility
    //         *LED_STATE.lock().await = target.target;
    //     }
    // }
}
