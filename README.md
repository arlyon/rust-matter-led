# matter-led

A Matter-compatible 5-channel RGBCW LED controller built on a custom PCB
with an ESP32-C6, written entirely in Rust. Pairs natively with Apple Home,
Google Home, and any other Matter controller -- no proprietary hub required.

## Hardware

### Board

The controller is a 2-layer PCB designed in KiCad 9. It accepts 24 V DC
input through a Molex 2601-1106 connector, protected by a 10 A mini blade
fuse (Littelfuse 0297010.WXNV). A Recom R-78E5.0-1.0 switching regulator
steps 24 V down to 5 V / 1 A to power the Waveshare ESP32-C6 module. Two
470 uF electrolytic capacitors (Panasonic EEUFR1V471L) filter the converter
input.

Five AO3400A N-channel MOSFETs in SOT-23 packages switch the LED channels
on the low side. Each gate is driven from a GPIO through a 100 ohm series
resistor; 10 kohm pull-downs hold the gates low during boot. The MOSFETs
switch the 24 V rail directly, so the LED strips see full supply voltage.

### LED strip

The board drives 24 V RGBCCT LED strips with a common-anode, 6-pin
configuration (V+, R, G, B, CW, WW). The target strip uses 5050-package
RGB LEDs alongside 2835-package tunable-white LEDs, with a colour
temperature range of 2000--6500 K and a CRI of 95. At 168 LEDs/m drawing
34.56 W/m, a 5 m reel pulls around 173 W. The strip is IP20 (no
weatherproofing). Five independent PWM channels allow continuous colour
temperature tuning between 2000 K and 6500 K, as well as full-gamut RGB
colour mixing.

An example strip that matches these specs:
https://www.aliexpress.com/item/1005006132338333.html

### Power supply

Use a 24 V DC supply rated for the strip length you intend to drive. The
board fuse is 10 A, so the practical ceiling is around 240 W at the input
connector. A Mean Well LRS-350-24 or equivalent is a reasonable choice for
a full 5 m strip.

### GPIO mapping

| GPIO | Channel    |
|------|------------|
| 1    | Red        |
| 0    | Green      |
| 10   | Blue       |
| 3    | Cold White |
| 2    | Warm White |
| 9    | BOOT (factory reset) |

## Firmware

The firmware is a `no_std` Rust binary targeting `riscv32imac-unknown-none-elf`.
It requires nightly Rust (minimum 1.88) for `type_alias_impl_trait` and
the alloc error handler.

### Architecture

The async runtime is [Embassy](https://embassy.dev). Hardware access goes
through `esp-hal`, and the Matter protocol stack is
[rs-matter](https://github.com/project-chip/rs-matter) with the
`rs-matter-embassy` integration layer. Networking uses `smoltcp` via
`embassy-net` with DHCPv4 and IPv6 support. BLE commissioning is handled
by `trouble-host` over the ESP32-C6's radio.

The device advertises as a Matter Extended Color Light (device type 0x10D)
on endpoint 1, exposing the following clusters:

- **OnOff** -- power state with smooth 400 ms transitions.
- **LevelControl** -- brightness from 1 to 254 (Matter range).
- **ColorControl** -- hue/saturation mode and colour temperature in mireds
  (153--500, i.e. 6500 K down to 2000 K).
- **Identify** -- cycles through red, green, blue to visually locate the
  device.

A second endpoint (endpoint 2) exposes an On/Off Light Switch that acts as
a "max brightness" override, driving all five channels at the current
brightness level simultaneously.

### PWM control

The LEDC peripheral is configured with a single 13-bit timer at 4882 Hz,
giving 8192 discrete duty levels per channel. A dedicated async task runs
at 50 Hz (20 ms per frame), interpolating between the current and target
PWM state with a cubic ease-in-out curve. RGB transitions are mixed in
Oklab colour space for perceptually uniform blending; CW/WW transitions
use linear interpolation in Kelvin space (converted from mireds). All
output values pass through gamma 2.2 correction before hitting the LEDC
registers.

### Colour temperature

The firmware maps the Matter mired range (153--500) to a mix ratio between
the CW and WW channels. Conversion goes through Kelvin
(`K = 1,000,000 / mireds`) and interpolates linearly in that space, which
is more perceptually uniform than interpolating in mireds directly.

### Memory layout

The ESP32-C6 has 8 MB of flash. The firmware reclaims bootloader RAM for
heap use:

| Region         | Size     | Purpose                             |
|----------------|----------|-------------------------------------|
| Heap           | 112 KB   | General allocations (peak ~72 KB)   |
| Bump allocator | 24 KB    | rs-matter internals (peak ~16 KB)   |
| Flash KV store | 128 KB   | Matter fabric and ACL persistence   |

The KV store sits at offset `0x700000` and uses `sequential-storage` with
32 x 4 KB sectors.

### Matter commissioning

The discriminator is derived from the last 12 bits of the device MAC
address, giving each board a unique but deterministic identity. The default
passcode is `20202021` (the Matter test value). Pairing data is printed to
the defmt log on boot.

To commission, use any Matter controller and enter the discriminator and
passcode, or generate a QR code with vendor ID `0xFFF1` and product ID
`0x8000`.

### Factory reset

Hold the BOOT button (GPIO 9) during power-on. The firmware erases the
128 KB flash KV store and reboots after a 2-second delay. All Matter
fabrics, ACLs, and saved state are wiped.

## Building

### Prerequisites

Install the toolchain via [mise](https://mise.jdx.dev):

```
mise install
```

This sets up nightly Rust with the `riscv32imac-unknown-none-elf` target
and the `espflash` tool.

Alternatively, install manually:

```
rustup toolchain install nightly
rustup target add riscv32imac-unknown-none-elf
cargo install espflash
```

### Compile and flash

```
cd firmware
cargo build --release
cargo espflash flash --release --monitor
```

The `--monitor` flag opens a serial console after flashing so you can see
defmt log output including the pairing discriminator.

## Hardware CI with KiBot

A GitHub Actions workflow runs [KiBot](https://github.com/INTI-CMNB/KiBot)
on every push or PR that touches schematic, PCB, or KiBot configuration
files. It uses the `ghcr.io/inti-cmnb/kicad9_auto_full` container image
and runs ERC/DRC preflight checks before generating outputs.

### Generated artifacts

All outputs land in `hardware/output/` and are uploaded as build artifacts.

| Directory    | Contents                                              |
|--------------|-------------------------------------------------------|
| `schematics` | Schematic PDF and SVG                                 |
| `gerbers`    | Gerber files (F/B Cu, SilkS, Mask, Paste, Edge.Cuts) + Excellon drill |
| `assembly`   | Pick-and-place CSV (JLC format, SMD only), full position file, interactive HTML BOM |
| `bom`        | JLC BOM (LCSC parts), Mouser/Digikey BOM (non-LCSC parts), Octopart 1-click import |
| `pcb`        | Top and bottom layer PDFs                             |
| `3d`         | STEP and VRML 3D models                               |

The BOM is split by sourcing. Components with an `LCSC` field go into the
JLC BOM for PCBA ordering; everything else (the ESP32-C6 module, Recom
regulator, fuse, connectors) goes into a separate BOM formatted for
Mouser/Digikey with MPN grouping. A third BOM variant is formatted for
[Octopart](https://octopart.com) one-click import -- upload the CSV and
Octopart resolves MPNs across distributors, shows stock and pricing, and
lets you build a cart in one step.

## Bill of materials

### On-board components

| Ref            | Part                   | Value / MPN         | Package    | LCSC    | Notes                          |
|----------------|------------------------|---------------------|------------|---------|--------------------------------|
| U1             | DC-DC converter        | R-78E5.0-1.0        | SIP-3      | --      | 24 V to 5 V, 1 A (Recom)      |
| U2             | MCU module             | ESP32-C6-Mini       | Castellated| --      | Waveshare, RISC-V 160 MHz, WiFi 6 + BLE 5 + Zigbee 3.0, 8 MB flash |
| Q1--Q5         | N-ch MOSFET            | AO3400A             | SOT-23     | --      | 30 V / 5.7 A, low-side LED switches |
| D1             | Schottky rectifier     | SS34                | SMC        | C211777 | 3 A / 40 V, reverse polarity protection |
| F1             | Fuse                   | 10 A                | Mini Blade | --      | Littelfuse 0297010.WXNV        |
| C1, C2         | Ceramic capacitor      | 100 nF              | 1206       | C13585  | Decoupling                     |
| C3             | Electrolytic capacitor | 470 uF 35 V         | Radial     | --      | EEUFR1V471L, regulator input   |
| R1,R3,R5,R7,R9 | Resistor              | 100 ohm             | 0402       | C25076  | MOSFET gate current limiting   |
| R2,R4,R6,R8,R10| Resistor              | 10 kohm             | 0402       | C25744  | MOSFET gate pull-down          |
| J1             | Power input connector  | Molex 2601-1106     | 2-pin      | --      | 24 V DC input                  |
| J2             | LED output connector   | XT30U-M             | XT30       | --      | RGBCW strip connection         |

### Off-board (not on PCB)

| Part              | Example                          | Notes                                    |
|-------------------|----------------------------------|------------------------------------------|
| 24 V power supply | Mean Well LRS-350-24             | Size to strip length; fuse limits 240 W  |
| RGBCCT LED strip  | 5050 RGB + 2835 CCT, 24 V, IP20 | 2000--6500 K, CRI 95, 168 LEDs/m, 34.56 W/m, 6-pin |

The KiBot CI generates three BOM formats automatically: a JLC-formatted
CSV for LCSC parts (ready for JLCPCB PCBA upload), a Mouser/Digikey BOM
grouped by MPN for manual ordering, and an Octopart CSV for one-click
quoting across distributors. All three are available as build artifacts
under `hardware/output/bom/`.

## Project structure

```
.
├── firmware/           Rust firmware for the ESP32-C6
│   ├── src/bin/        Entry point (main.rs)
│   ├── src/clusters/   Matter cluster handlers (OnOff, LevelControl, ColorControl, Identify)
│   ├── src/led/        PWM control, colour conversion, device state
│   └── tests/          On-target embedded tests
├── hardware/           KiCad 9 schematic and PCB
│   ├── external/       Third-party symbol libraries
│   └── internal/       Custom symbols (Waveshare module)
├── crates/
│   └── sticky-signal/  Async signal primitive (no_std, multi-waiter)
├── config.kibot.yaml   KiBot output definitions
└── .github/workflows/  CI for hardware artifact generation
```

## License

See individual crate and hardware directories for licensing details.
