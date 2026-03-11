use super::{LED_STATE, LedPwmState, TARGET_STATE};
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use easer::functions::Easing;
use esp_hal::{
    ledc::{
        LowSpeed,
        channel::{self, ChannelHW, ChannelIFace},
        timer::{self, TimerIFace},
    },
    peripherals::LEDC,
    time::Rate,
};

extern crate alloc;
use alloc::boxed::Box;

/// Signal to trigger identify sequence
pub static IDENTIFY_SIGNAL: Signal<CriticalSectionRawMutex, u16> = Signal::new();

/// PWM channels holder struct for LEDC
pub struct LedcChannels {
    pub ch_r: channel::Channel<'static, LowSpeed>,
    pub ch_g: channel::Channel<'static, LowSpeed>,
    pub ch_b: channel::Channel<'static, LowSpeed>,
    pub ch_cw: channel::Channel<'static, LowSpeed>,
    pub ch_ww: channel::Channel<'static, LowSpeed>,
}

/// Initialize LEDC peripheral for 5-channel LED control
/// Returns configured channels ready for PWM control
pub fn init_ledc(
    ledc: LEDC<'static>,
    pin_r: impl esp_hal::gpio::OutputPin + 'static,
    pin_g: impl esp_hal::gpio::OutputPin + 'static,
    pin_b: impl esp_hal::gpio::OutputPin + 'static,
    pin_cw: impl esp_hal::gpio::OutputPin + 'static,
    pin_ww: impl esp_hal::gpio::OutputPin + 'static,
) -> LedcChannels {
    defmt::info!("Initializing LEDC for 5-channel LED control...");

    let mut ledc = esp_hal::ledc::Ledc::new(ledc);
    ledc.set_global_slow_clock(esp_hal::ledc::LSGlobalClkSource::APBClk);

    // Configure timer: 20kHz, 10-bit resolution (0-1023)
    let mut timer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    timer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty10Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(20),
        })
        .unwrap();

    // Leak the timer to get a 'static reference
    // This is safe because we only initialize once and need it forever
    let timer0: &'static _ = Box::leak(Box::new(timer0));

    // Configure all 5 channels
    let mut ch_r = ledc.channel(channel::Number::Channel0, pin_r);
    ch_r.configure(channel::config::Config {
        timer: timer0,
        duty_pct: 0,
        drive_mode: esp_hal::gpio::DriveMode::PushPull,
    })
    .unwrap();

    let mut ch_g = ledc.channel(channel::Number::Channel1, pin_g);
    ch_g.configure(channel::config::Config {
        timer: timer0,
        duty_pct: 0,
        drive_mode: esp_hal::gpio::DriveMode::PushPull,
    })
    .unwrap();

    let mut ch_b = ledc.channel(channel::Number::Channel2, pin_b);
    ch_b.configure(channel::config::Config {
        timer: timer0,
        duty_pct: 0,
        drive_mode: esp_hal::gpio::DriveMode::PushPull,
    })
    .unwrap();

    let mut ch_cw = ledc.channel(channel::Number::Channel3, pin_cw);
    ch_cw.configure(channel::config::Config {
        timer: timer0,
        duty_pct: 0,
        drive_mode: esp_hal::gpio::DriveMode::PushPull,
    })
    .unwrap();

    let mut ch_ww = ledc.channel(channel::Number::Channel4, pin_ww);
    ch_ww.configure(channel::config::Config {
        timer: timer0,
        duty_pct: 0,
        drive_mode: esp_hal::gpio::DriveMode::PushPull,
    })
    .unwrap();

    defmt::info!("LEDC initialized: 20kHz, 10-bit resolution (0-1023)");

    LedcChannels {
        ch_r,
        ch_g,
        ch_b,
        ch_cw,
        ch_ww,
    }
}

/// PWM update task - reads LED state from TARGET_STATE signal and updates PWM outputs
/// Also handles identify sequences which take priority and run uninterrupted
/// Supports smooth transitions with gamma correction and ease-in-out curves
#[embassy_executor::task]
pub async fn pwm_task(mut channels: LedcChannels) {
    defmt::info!("PWM task started");

    let mut last_applied_state = LedPwmState::default();

    // Helper to apply a PWM state with Gamma 2.0 correction
    // Maps linear 8-bit (0-255) to human-eye corrected 10-bit (0-1023)
    let apply_state = |channels: &mut LedcChannels, state: &LedPwmState| {
        // Gamma 2.0 correction: (value^2 * 1023) / (255^2)
        let scale = |val: u8| {
            let v = val as u32;
            (v * v * 1023) / 65025 // 65025 = 255^2
        };
        channels.ch_r.set_duty_hw(scale(state.r));
        channels.ch_g.set_duty_hw(scale(state.g));
        channels.ch_b.set_duty_hw(scale(state.b));
        channels.ch_cw.set_duty_hw(scale(state.cw));
        channels.ch_ww.set_duty_hw(scale(state.ww));
    };

    // Helper to linearly interpolate between two states based on progress (0.0 to 1.0)
    let lerp_state = |start: &LedPwmState, end: &LedPwmState, t: f32| -> LedPwmState {
        let lerp_u8 = |a: u8, b: u8, t: f32| -> u8 {
            (a as f32 + (b as f32 - a as f32) * t).clamp(0.0, 255.0) as u8
        };
        LedPwmState {
            r: lerp_u8(start.r, end.r, t),
            g: lerp_u8(start.g, end.g, t),
            b: lerp_u8(start.b, end.b, t),
            cw: lerp_u8(start.cw, end.cw, t),
            ww: lerp_u8(start.ww, end.ww, t),
        }
    };

    loop {
        // Select over both signals to handle whichever arrives first
        match select(IDENTIFY_SIGNAL.wait(), TARGET_STATE.wait("pwm")).await {
            Either::First(time_seconds) => {
                // Identify signal received
                if time_seconds > 0 {
                    defmt::info!("Identify sequence triggered for {} seconds", time_seconds);

                    // Save current state to restore later
                    let saved_state = TARGET_STATE.peek();

                    // RED phase (1 second)
                    defmt::info!("Identify: RED");
                    apply_state(
                        &mut channels,
                        &LedPwmState {
                            r: 255,
                            g: 0,
                            b: 0,
                            cw: 0,
                            ww: 0,
                        },
                    );
                    Timer::after(Duration::from_secs(1)).await;

                    // GREEN phase (1 second)
                    defmt::info!("Identify: GREEN");
                    apply_state(
                        &mut channels,
                        &LedPwmState {
                            r: 0,
                            g: 255,
                            b: 0,
                            cw: 0,
                            ww: 0,
                        },
                    );
                    Timer::after(Duration::from_secs(1)).await;

                    // BLUE phase (1 second)
                    defmt::info!("Identify: BLUE");
                    apply_state(
                        &mut channels,
                        &LedPwmState {
                            r: 0,
                            g: 0,
                            b: 255,
                            cw: 0,
                            ww: 0,
                        },
                    );
                    Timer::after(Duration::from_secs(1)).await;

                    defmt::info!("Identify sequence complete");

                    // Restore saved state if it existed
                    if let Some(saved) = saved_state {
                        defmt::info!("Restoring previous state: {:?}", saved.target);
                        apply_state(&mut channels, &saved.target);
                        last_applied_state = saved.target;
                    } else {
                        // No previous state, turn off
                        defmt::info!("No previous state, turning off");
                        apply_state(&mut channels, &LedPwmState::default());
                        last_applied_state = LedPwmState::default();
                    }
                }
            }
            Either::Second(target) => {
                // Target state received
                defmt::info!(
                    "PWM task received new target: {:?}, transition: {}ms",
                    target.target,
                    target.transition_duration_ms
                );

                let start_state = last_applied_state;
                let end_state = target.target;
                let duration_ms = target.transition_duration_ms;

                // Handle instant transitions (0ms duration)
                if duration_ms == 0 {
                    defmt::info!("Instant transition to: {:?}", end_state);
                    apply_state(&mut channels, &end_state);
                    last_applied_state = end_state;
                    *LED_STATE.lock().await = end_state;
                    continue;
                }

                // Smooth transition with 50Hz update rate (20ms per frame)
                defmt::info!(
                    "Starting smooth transition from {:?} to {:?} over {}ms",
                    start_state,
                    end_state,
                    duration_ms
                );

                let start_time = Instant::now();
                let duration = Duration::from_millis(duration_ms as u64);
                const FRAME_TIME: Duration = Duration::from_millis(20); // 50Hz

                loop {
                    let elapsed = start_time.elapsed();

                    // Check if transition is complete
                    if elapsed >= duration {
                        // Apply final state
                        apply_state(&mut channels, &end_state);
                        last_applied_state = end_state;
                        *LED_STATE.lock().await = end_state;
                        defmt::info!("Transition complete");
                        break;
                    }

                    // Calculate linear progress (0.0 to 1.0)
                    let progress = elapsed.as_millis() as f32 / duration_ms as f32;

                    // Apply cubic ease-in-out for smooth acceleration/deceleration
                    let eased = easer::functions::Cubic::ease_in_out(progress, 0.0, 1.0, 1.0);

                    // Interpolate state
                    let current_state = lerp_state(&start_state, &end_state, eased);

                    // Apply to hardware
                    apply_state(&mut channels, &current_state);

                    // Wait for next frame
                    Timer::after(FRAME_TIME).await;
                }
            }
        }
    }
}
