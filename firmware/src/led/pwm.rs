use super::{LED_STATE, LedPwmState};
use embassy_time::{Duration, Ticker};
use esp_hal::{
    mcpwm::{
        FrequencyError, McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode,
    },
    peripherals::MCPWM0,
    time::Rate,
};

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

/// PWM update task - reads LED state and updates PWM outputs
#[embassy_executor::task]
pub async fn pwm_task(mut pwm_pins: PwmPins, period: u16) {
    defmt::info!("PWM task started with period: {}", period);
    let mut ticker = Ticker::every(Duration::from_millis(20)); // ~50 Hz update rate

    let mut last_applied_state = LedPwmState::default();

    loop {
        ticker.next().await;
        let target_state = *LED_STATE.lock().await;

        if target_state != last_applied_state {
            defmt::info!("Applying PWM state: {:?}", target_state);
            // Scale 0-255 input to 0-period range
            let scale = |val: u8| ((val as u32 * period as u32) / 255) as u16;

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
