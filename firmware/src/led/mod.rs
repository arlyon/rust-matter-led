pub mod pwm;

use embassy_sync::mutex::Mutex;

/// LED PWM state representing the current/target values for all channels
#[derive(Clone, Copy, Debug, defmt::Format, PartialEq)]
pub struct LedPwmState {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub cw: u8,
    pub ww: u8,
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

    /// Check if any channel is on
    pub fn is_on(&self) -> bool {
        self.r != 0 || self.g != 0 || self.b != 0 || self.cw != 0 || self.ww != 0
    }
}

/// Global LED state shared between Matter handlers and PWM task
pub static LED_STATE: Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, LedPwmState> =
    Mutex::new(LedPwmState::new());
