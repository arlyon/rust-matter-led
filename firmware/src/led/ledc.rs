use esp_hal::{
    ledc::{Ledc, LowSpeed, channel::{self, Channel}, timer::{self, Timer}},
    peripherals::LEDC,
    time::Rate,
};

/// PWM pin holder struct using LEDC
pub struct LedcPins {
    pub pin_r: Channel<'static, LowSpeed, 0>,
    pub pin_g: Channel<'static, LowSpeed, 1>,
    pub pin_b: Channel<'static, LowSpeed, 2>,
    pub pin_cw: Channel<'static, LowSpeed, 3>,
    pub pin_ww: Channel<'static, LowSpeed, 4>,
}

/// Initialize LEDC peripheral for RGB LED control
pub fn init_ledc(
    ledc: LEDC,
    pin_r: impl esp_hal::gpio::OutputPin,
    pin_g: impl esp_hal::gpio::OutputPin,
    pin_b: impl esp_hal::gpio::OutputPin,
    pin_cw: impl esp_hal::gpio::OutputPin,
    pin_ww: impl esp_hal::gpio::OutputPin,
) -> (LedcPins, u32) {
    defmt::info!("Initializing LEDC...");

    let mut ledc = Ledc::new(ledc);

    // Configure timer0 for 20kHz PWM with 10-bit resolution (0-1023)
    ledc.timer0.configure(timer::config::Config {
        duty: timer::config::Duty::Duty10Bit,
        clock_source: timer::LSClockSource::default(),
        frequency: Rate::from_khz(20),
    }).unwrap();

    // Create channels for each LED color
    let pin_r = ledc.channel0.configure(channel::config::Config {
        timer: &ledc.timer0,
        duty_pct: 0,
        pin_config: channel::config::PinConfig::PushPull,
    }, pin_r).unwrap();

    let pin_g = ledc.channel1.configure(channel::config::Config {
        timer: &ledc.timer0,
        duty_pct: 0,
        pin_config: channel::config::PinConfig::PushPull,
    }, pin_g).unwrap();

    let pin_b = ledc.channel2.configure(channel::config::Config {
        timer: &ledc.timer0,
        duty_pct: 0,
        pin_config: channel::config::PinConfig::PushPull,
    }, pin_b).unwrap();

    let pin_cw = ledc.channel3.configure(channel::config::Config {
        timer: &ledc.timer0,
        duty_pct: 0,
        pin_config: channel::config::PinConfig::PushPull,
    }, pin_cw).unwrap();

    let pin_ww = ledc.channel4.configure(channel::config::Config {
        timer: &ledc.timer0,
        duty_pct: 0,
        pin_config: channel::config::PinConfig::PushPull,
    }, pin_ww).unwrap();

    let pins = LedcPins {
        pin_r,
        pin_g,
        pin_b,
        pin_cw,
        pin_ww,
    };

    defmt::info!("LEDC initialized with 10-bit resolution (max duty: 1023)");

    // Return pins and max duty value (2^10 - 1 = 1023)
    (pins, 1023)
}
