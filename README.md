# 6-Channel TRIAC Controller HAT

A Raspberry Pi HAT for mains AC power control across six independent channels. Each channel supports both **phase-angle dimming** and **burst-fire (zero-crossing) switching**, selectable individually per channel. An ATtiny1616 microcontroller handles all real-time mains timing, zero-crossing detection, and thermal management, communicating with the Raspberry Pi over I²C.

> ⚠️ **Safety warning:** This board connects directly to mains voltage (110–240V AC). It must only be assembled, tested, and operated by persons with appropriate electrical knowledge. Always work with the board fully de-energised. Fit an external fuse. Use a suitable enclosure. Never touch the mains-side components while the board is powered.

---

## Table of Contents

- [Features](#features)
- [Hardware Overview](#hardware-overview)
- [Bill of Materials](#bill-of-materials)
- [Board Specifications](#board-specifications)
- [Pin Assignment](#pin-assignment)
- [I²C Address Selection](#i2c-address-selection)
- [Firmware](#firmware)
  - [Building](#building)
  - [Flashing](#flashing)
  - [Fuse Settings](#fuse-settings)
- [I²C Register Map](#i2c-register-map)
  - [STATUS (0x00)](#status-0x00)
  - [CH1–CH6 Level (0x01–0x06)](#ch1ch6-level-0x010x06)
  - [CONFIG (0x07)](#config-0x07)
  - [BURST\_CH (0x08)](#burst_ch-0x08)
  - [TEMP\_RAW (0x09)](#temp_raw-0x09)
  - [ZC\_COUNT (0x0A)](#zc_count-0x0a)
  - [FAN\_DUTY (0x0B)](#fan_duty-0x0b)
  - [CH\_MODE (0x0C)](#ch_mode-0x0c)
- [Control Modes](#control-modes)
  - [Phase-Angle Control](#phase-angle-control)
  - [Burst-Fire Control](#burst-fire-control)
  - [Per-Channel Mode Selection](#per-channel-mode-selection)
- [Fan Control](#fan-control)
  - [Automatic Thermal Curve](#automatic-thermal-curve)
  - [Manual Override](#manual-override)
- [Status LEDs](#status-leds)
- [Safety Features](#safety-features)
  - [ZC Watchdog](#zc-watchdog)
  - [Over-Temperature Shutdown](#over-temperature-shutdown)
  - [EEPROM Persistence](#eeprom-persistence)
- [Python Driver](#python-driver)
  - [Installation](#installation)
  - [Quick Start](#quick-start)
  - [API Reference](#api-reference)
- [HAT ID EEPROM](#hat-id-eeprom)
- [PCB Layout Guidelines](#pcb-layout-guidelines)
- [Repository Structure](#repository-structure)
- [Licence](#licence)

---

## Features

| Feature | Detail |
|---|---|
| Output channels | 6 × isolated TRIAC (BT136-600D, 600V 4A) |
| Control modes | Phase-angle dimming **or** burst-fire, per channel |
| MCU | ATtiny1616, 20 MHz internal oscillator |
| Interface | I²C slave, 100/400 kHz, address 0x40–0x43 |
| Multi-board | Up to 4 boards on one bus = 24 channels |
| Mains frequency | 50 Hz or 60 Hz, auto-detected |
| Fan control | PWM, automatic thermal curve from NTC, 78 kHz |
| Over-temperature | Hardware latch at 85°C, fan forced full speed |
| ZC watchdog | All outputs safe-off after 630 ms without ZC |
| EEPROM | Channel levels + config persist across power cycles |
| Status LEDs | Green (healthy / I²C idle blink) + Red (fault / boot) |
| Power source | Raspberry Pi 40-pin HAT header (+5V, GND) |
| Isolation | PC817 and EL357NB optocouplers, ≥5000V |
| Form factor | Raspberry Pi HAT (65 × 56 mm) |

---

## Hardware Overview

```
┌─────────────────────────────────────────────────────┐
│                                                     │
│   AC L/N ──► BR1 bridge ──► 78L05 ──► +5V (logic)  │
│                 │                                   │
│                 └──► R1 ──► PC817 ZC opto ──► PA3   │
│                                                     │
│   RPi 40-pin header ──► +5V, GND, SDA, SCL         │
│                                                     │
│   ATtiny1616 ─┬─► PB0–PB3, PC0–PC1 (CH1–CH6 gates) │
│               ├─► PC2 (fan PWM → MOSFET)           │
│               ├─► PB4 (green LED)                  │
│               └─► PC3 (red LED)                    │
│                                                     │
│   CH_gate ──► 330R ──► EL357N ──► BT136 ──► LOAD   │
│                                                     │
│   NTC divider (PA6) ──► thermal fan curve          │
│   Addr solder bridges (PA4/PA5) ──► I²C address    │
│                                                     │
└─────────────────────────────────────────────────────┘
```

Each of the six channels is independently isolated through an EL357N TRIAC-driver optocoupler. The mains-side ground (GNDPWR) is fully separated from the logic ground (GND) — the only path between them is through the optocouplers.

---

## Bill of Materials

| Ref | Value | Digikey P/N | Qty |
|---|---|---|---|
| U1 | ATtiny1616-SFR | ATTINY1616-SFR-ND | 1 |
| U2–U7 | EL357NB opto | EL357NB(TA)-ND | 6 |
| U8 | PC817B ZC opto | PC817B-ND | 1 |
| U9 | 1N4148WS flyback | 1N4148WDICT-ND | 1 |
| U10 | AT24C32D EEPROM | AT24C32D-STUM-TCT-ND | 1 |
| Q1–Q6 | BT136-600D TRIAC | BT136-600D,127CT-ND | 6 |
| Q7 | LM78L05 SOT-23 | LM78L05ACM3XNOPBCT-ND | 1 |
| Q8 | AO3400A N-ch FET | AO3400ADICT-ND | 1 |
| D1 | MB6S bridge rect | MB6SFSCT-ND | 1 |
| R1–R6, R21, R22 | 330R 0805 | RMCF0805JT330RCT-ND | 8 |
| R7, R9, R11, R13, R15, R17 | 330R 1210 | RMCF1210JT330RCT-ND | 6 |
| R8, R10, R12, R14, R16, R18 | 39R 1W axial | 39QBK-ND | 6 |
| R19 | 10k axial | CF14JT10K0CT-ND | 1 |
| R20, R24, R25 | 10k 0805 | RMCF0805JT10K0CT-ND | 3 |
| R23 | 100R 0805 | RMCF0805JT100RCT-ND | 1 |
| R26, R27 | 0R 0603 DNP | RMCF0603ZT0R00CT-ND | 2 |
| R28 | 10k 0603 | RMCF0603JT10K0CT-ND | 1 |
| C1–C6 | 10nF 630V film | 495-1511-ND | 6 |
| C7, C8 | 100nF 0805 | 399-1170-1-ND | 2 |
| C9 | 100nF 1206 | 399-C1206C104K5RACT-ND | 1 |
| P1–P7 | 2-pin 5.08mm | A98333-ND | 7 |
| FAN1, NTC | 2-pin JST PH 2mm | 455-1719-ND | 2 |
| J1 | 3-pin 2.54mm (UPDI) | S1011EC-03-ND | 1 |
| P8 | 2×20 stacking header | S5751-40-ND | 1 |
| LED1 | Red 0805 | 160-1167-1-ND | 1 |
| LED2 | Green 0805 | 160-1183-1-ND | 1 |

> **C1–C6:** Snubber capacitors **must** be 630V-rated film type. Do not substitute ceramic capacitors here.
> **R8, R10, R12, R14, R16, R18:** Snubber resistors carry mains-side pulse current. Use 1W-rated axial parts.
> **R26, R27:** Address solder bridges. Leave unpopulated (DNP) for default address 0x40. See [I²C Address Selection](#i2c-address-selection).

---

## Board Specifications

| Parameter | Value |
|---|---|
| Input voltage (logic) | 5V from RPi HAT header |
| Input voltage (mains) | 85–264V AC, 50/60 Hz |
| Max load current per channel | 4A RMS (BT136 rating) |
| Max load current (PCB traces) | ~5A with dual 2mm parallel top/bottom traces |
| TRIAC voltage rating | 600V |
| Gate isolation voltage | ≥5000V (EL357N) |
| ZC isolation voltage | ≥5000V (PC817) |
| Phase-angle resolution | 128 steps (~77 µs per step at 50 Hz) |
| Burst-fire resolution | 1/128 half-cycles |
| Fan PWM frequency | 78 kHz |
| MCU clock | 20 MHz (internal oscillator) |
| I²C speed | 100 kHz / 400 kHz |
| Operating temperature | 0–70°C (derate above 50°C) |
| Over-temperature trip | 85°C (NTC at heatsink) |
| Over-temperature reset | 75°C (10°C hysteresis) |

---

## Pin Assignment

| MCU Pin | Function | Direction | Notes |
|---|---|---|---|
| PA1 | SDA | Input | I²C data — RPi GPIO2 (pin 3) |
| PA2 | SCL | Input | I²C clock — RPi GPIO3 (pin 5) |
| PA3 | ZC_IN | Input | Zero-crossing from PC817, falling-edge interrupt |
| PA4 | ADDR0 | Input | I²C address bit 0, internal pull-up |
| PA5 | ADDR1 | Input | I²C address bit 1, internal pull-up |
| PA6 | NTC_ADC | Input | NTC thermistor voltage divider (ADC) |
| PA0 | UPDI | I/O | Programming interface — expose as test point |
| PB0 | CH1 gate | Output | 330Ω → EL357N → BT136 |
| PB1 | CH2 gate | Output | |
| PB2 | CH3 gate | Output | |
| PB3 | CH4 gate | Output | |
| PB4 | Green LED | Output | 330Ω series resistor to GND, HIGH = on |
| PC0 | CH5 gate | Output | |
| PC1 | CH6 gate | Output | |
| PC2 | Fan PWM | Output | TCB0 WO, 78 kHz, 100Ω → AO3400A gate |
| PC3 | Red LED | Output | 330Ω series resistor to GND, HIGH = on |

---

## I²C Address Selection

The board I²C address is set at power-on by solder bridges SB1 (PA4) and SB2 (PA5). Each pin has a 10kΩ pull-up to +5V. Soldering the bridge ties the pin to GND.

| SB1 (PA4) | SB2 (PA5) | I²C Address |
|---|---|---|
| Open | Open | **0x40** (default) |
| Bridged | Open | 0x41 |
| Open | Bridged | 0x42 |
| Bridged | Bridged | 0x43 |

Up to four boards can share one I²C bus, giving 24 independently controlled channels.

---

## Firmware

### Building

Requires `avr-gcc`, `avr-libc`, and `avrdude`.

**macOS:**
```bash
brew install avr-gcc avrdude
```

**Linux (Debian/Ubuntu):**
```bash
sudo apt install gcc-avr avr-libc avrdude
```

**Build:**
```bash
cd firmware
make
```

This produces `triac_controller.elf` and `triac_controller.hex`.

### Flashing

Using an [Adafruit UPDI Friend](https://www.adafruit.com/product/5879) or any SerialUPDI-compatible programmer:

```bash
# Set PORT to match your system:
#   Linux:   /dev/ttyUSB0
#   macOS:   /dev/tty.usbserial-XXXXXXXX
#   Windows: COM3

make flash PORT=/dev/ttyUSB0
```

Or manually:
```bash
avrdude -c serialupdi -p attiny1616 -P /dev/ttyUSB0 -b 115200 \
        -U flash:w:triac_controller.hex:i -v
```

**UPDI wiring:**

```
UPDI Friend     ATtiny1616 (J1 header)
───────────     ──────────────────────
5V          →   VCC  (pin 1 of J1)
GND         →   GND  (pin 2 of J1)
UPDI        →   PA0  (pin 3 of J1)  via 470Ω series resistor
```

### Fuse Settings

Set the MCU to run at 20 MHz by default (one-time operation):

```bash
avrdude -c serialupdi -p attiny1616 -P /dev/ttyUSB0 -b 115200 \
        -U fuse2:w:0x02:m
```

This sets CLKSEL to the 20 MHz internal oscillator with no prescaler.

---

## I²C Register Map

All registers are 8-bit. The register pointer auto-increments on burst reads/writes. Multi-byte writes should send the register address as the first byte followed by data bytes.

### STATUS (0x00)

**Read-only.**

| Bit | Name | Description |
|---|---|---|
| 7 | ZC_LOCK | 1 = zero crossings being received and locked |
| 6–3 | — | Reserved, read as 0 |
| 2 | FAN_ON | 1 = fan is currently spinning |
| 1 | OVERTEMP | 1 = over-temperature fault latched |
| 0 | 60HZ | 1 = 60 Hz mains detected; 0 = 50 Hz |

### CH1–CH6 Level (0x01–0x06)

**Read/Write.**

Controls the output power level for each channel. Encoding is the same in both phase-angle and burst-fire modes:

| Value | Meaning |
|---|---|
| 0 | Channel OFF (gate never fires) |
| 1–127 | Proportional power (phase delay or burst duty) |
| 128 | Channel fully ON (gate fires at every ZC immediately) |

### CONFIG (0x07)

**Read/Write.**

| Bit | Name | Description |
|---|---|---|
| 7 | fan_force_off | Force fan off regardless of temperature (overridden by OVERTEMP) |
| 6 | fan_manual | Enable manual fan control via FAN_DUTY register |
| 5 | fault_clr* | Write 1 to attempt over-temperature fault clear |
| 4 | ee_load* | Write 1 to restore channel levels from EEPROM |
| 3 | ee_save* | Write 1 to save current channel levels to EEPROM |
| 2 | — | Reserved (was global burst_mode — use CH_MODE register instead) |
| 1 | invert | Invert level mapping (128 = off, 0 = full on) |
| 0 | force_60hz | Force 60 Hz timing before ZC lock is established |

`*` Self-clearing bits: the firmware executes the action within one TCA overflow (~210 ms) then clears the bit automatically. Poll until the bit reads 0 to confirm completion.

### BURST_CH (0x08)

**Write-only convenience register.**

Writing to this register address followed by 6 data bytes sets CH1–CH6 levels in a single I²C transaction. This is more efficient than six individual writes and ensures all channels update atomically on the same zero crossing.

```
I²C write: [0x08] [CH1_val] [CH2_val] [CH3_val] [CH4_val] [CH5_val] [CH6_val]
```

### TEMP_RAW (0x09)

**Read-only.**

Raw ADC byte from the NTC thermistor voltage divider. **Higher byte = colder.** This is because the NTC is on the GND side of the divider — as temperature rises, NTC resistance falls, pulling the midpoint voltage lower.

| ADC Byte | Approx. Temperature |
|---|---|
| 142 | 20°C |
| 88 | 40°C |
| 50 | 60°C (fan starts) |
| 28 | 80°C (fan full speed) |
| 25 | 85°C (overtemp trip) |

Use `ntc_byte_to_celsius()` in the Python driver to convert to degrees Celsius.

### ZC_COUNT (0x0A)

**Read-only.**

Rolling 8-bit counter incremented on every zero-crossing interrupt (wraps 255→0). At 50 Hz this increments 100 times per second (both edges of the full-wave rectified waveform). Read it twice with a known delay to verify mains is present and the MCU is alive without reading STATUS.

### FAN_DUTY (0x0B)

**Read always. Write requires CONFIG Bit6 (fan_manual) = 1.**

- **Read:** Returns the current PWM duty byte (0–255) actually being applied to the fan MOSFET, whether set by the thermal curve or manual override.
- **Write:** Sets the desired duty for manual override mode. Has no effect unless CONFIG Bit6 is set first.

### CH_MODE (0x0C)

**Read/Write.**

Per-channel firing mode bitmask. Bits 0–5 correspond to channels 1–6.

| Bit | Channel | Value 0 | Value 1 |
|---|---|---|---|
| 0 | CH1 | Phase-angle | Burst-fire |
| 1 | CH2 | Phase-angle | Burst-fire |
| 2 | CH3 | Phase-angle | Burst-fire |
| 3 | CH4 | Phase-angle | Burst-fire |
| 4 | CH5 | Phase-angle | Burst-fire |
| 5 | CH6 | Phase-angle | Burst-fire |
| 6–7 | — | Reserved | Reserved |

**Example:** `0x03` = CH1 and CH2 in burst-fire mode, CH3–CH6 in phase-angle mode.

Writing this register resets the Bresenham burst accumulator for any channel transitioning from phase-angle into burst-fire mode.

---

## Control Modes

### Phase-Angle Control

The MCU detects each zero crossing and fires the TRIAC gate after a programmable delay within the half-cycle. A longer delay means less of the half-cycle conducts, reducing delivered power. This gives smooth continuous control from 0–100%.

- **Best for:** Light dimming, motor speed control, voltage-sensitive loads
- **Resolution:** 128 steps, approximately 77 µs per step at 50 Hz
- **EMI:** Produces some harmonic EMI due to mid-cycle switching — use a line filter for sensitive environments

### Burst-Fire Control

The MCU decides at each zero crossing whether that entire half-cycle should fire or not, using a Bresenham accumulator to distribute ON half-cycles evenly across the window. A level of 64 fires approximately every other half-cycle (50% power).

- **Best for:** Resistive heating elements, incandescent lamps, loads that tolerate on/off cycling
- **EMI:** Negligible — switching only occurs at zero crossings where V=0
- **Flicker:** Visible at low duty cycles on lighting loads (use phase-angle for dimmers)

### Per-Channel Mode Selection

Modes are set independently per channel via the CH_MODE register. There is no restriction on mixing modes across channels:

```python
# CH1 + CH2 as burst-fire heater control, CH3–CH6 as phase-angle dimmers
tc.set_channels_mode([True, True, False, False, False, False])

# Change just one channel
tc.set_channel_mode(3, burst=True)

# All channels the same
tc.set_burst_mode(True)   # all burst-fire
tc.set_burst_mode(False)  # all phase-angle
```

---

## Fan Control

### Automatic Thermal Curve

The fan is driven automatically by the NTC thermistor reading on PA6. The NTC thermistor (10kΩ B3950) should be mounted on or thermally coupled to the BT136 TRIAC heatsink.

| Temperature | Fan Behaviour |
|---|---|
| < 57°C | Fan OFF |
| 57–60°C | Hysteresis band — fan stays in current state |
| 60–80°C | Linear ramp from 31% to 100% duty |
| > 80°C | Fan at full speed (duty = 255) |
| > 85°C | Overtemp latch — all TRIACs off, fan at 100% |

A minimum duty of 31% (80/255) is applied when the fan first turns on to guarantee the motor overcomes static friction.

### Manual Override

```python
# Take manual control
tc.fan_set_manual(128)      # 50% speed

# Suppress fan (quiet hours) — overridden by overtemp
tc.fan_force_off(True)

# Return to automatic thermal curve
tc.fan_set_auto()

# Read current speed
print(f"Fan: {tc.fan_get_speed_pct():.1f}%")
print(f"Temperature: {tc.read_temperature():.1f}°C")
```

---

## Status LEDs

Two LEDs provide at-a-glance system status without needing a serial connection.

| Green LED | Red LED | System state |
|---|---|---|
| OFF | ON solid | Boot / initialising / no ZC lock |
| ON solid | OFF | Healthy — ZC locked, I²C active, no faults |
| Blinking ~1 Hz | OFF | ZC locked but no I²C writes for >5 seconds (RPi silent) |
| OFF | ON solid | Fault — over-temperature OR ZC watchdog timeout |
| ON (1 flash) | ON (1 flash) | EEPROM save confirmation (~210 ms) |

The blinking green state is particularly useful for diagnosing a crashed Python driver — if the green LED was solid and starts blinking, the RPi application has stopped communicating while the controller and mains are still running.

---

## Safety Features

### ZC Watchdog

The firmware monitors zero-crossing events using TCA0 overflow as a timebase. If three consecutive TCA overflows pass without a ZC edge (~630 ms total), all TRIAC gate outputs are forced low and the ZC_LOCK status bit clears. The fan continues running normally — loss of mains does not change the thermal situation.

Firing resumes automatically as soon as ZC events return — no I²C intervention is needed.

### Over-Temperature Shutdown

The NTC thermistor is sampled once per TCA overflow (~210 ms). If the temperature exceeds 85°C (ADC byte ≤ 25):

1. All six TRIAC gates are forced low immediately
2. The OVERTEMP status bit is latched
3. The fan is forced to full speed (duty = 255), overriding all other fan settings
4. The red LED comes on solid

The latch persists even after the board cools. To reset:

```python
cleared = tc.clear_fault()
if not cleared:
    print("Board still too hot — wait for it to cool below 75°C")
```

The firmware only accepts the fault clear if the temperature has also dropped below 75°C (ADC byte ≥ 33), providing 10°C of hysteresis.

### EEPROM Persistence

Channel levels, CH_MODE, and CONFIG are saved to the ATtiny1616's internal EEPROM. On power-up the firmware reads back the saved values with a checksum verification. If the checksum fails (blank chip or corrupt data) all channels default to off — a safe state.

```python
# Save current state
tc.save_to_eeprom()   # blocks until confirmed (~210 ms)

# Restore saved state at runtime (also happens automatically on power-up)
tc.load_from_eeprom()
```

Fan duty and thermal thresholds are not saved — they are always determined from the live NTC reading.

---

## Python Driver

### Installation

```bash
pip install smbus2
```

Copy `triac_controller.py` to your project directory, or install from this repository:

```bash
pip install git+https://github.com/yourname/6ch-triac-hat.git
```

Enable I²C on the Raspberry Pi if not already done:
```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

### Quick Start

```python
from triac_controller import TriacController

# Single board — context manager ensures clean shutdown
with TriacController() as tc:

    # Check board is healthy
    status = tc.status()
    print(status)
    # ZC:locked 50Hz | CH1=0(p) ... | temp=42.3°C fan=0%(auto) zc_cnt=87

    # Set individual channels
    tc.set_channel(1, 64)       # CH1 at 50% power
    tc.set_channel(2, 128)      # CH2 fully on

    # Set all channels at once (one I²C transaction)
    tc.set_all([128, 96, 64, 32, 16, 0])

    # Use percentage
    tc.set_percent(3, 75.0)     # CH3 at 75%

    # Smooth fade
    tc.fade(1, 0, duration_s=3.0)   # fade CH1 to off over 3 seconds

    # Fade all channels simultaneously
    tc.fade_all([0]*6, duration_s=2.0)

    # Per-channel burst/phase mode
    tc.set_channel_mode(1, burst=True)   # CH1 → burst-fire
    tc.set_channel_mode(2, burst=False)  # CH2 → phase-angle

    # Save current levels so they restore on next power-up
    tc.save_to_eeprom()
```

### API Reference

#### TriacController

```python
TriacController(i2c_bus=1, address=0x40, freq_hz=50)
```

| Method | Description |
|---|---|
| `set_channel(ch, level)` | Set channel 1–6 to level 0–128 |
| `set_all(levels)` | Set all 6 channels in one burst write |
| `get_channel(ch)` | Read current level for one channel |
| `get_all_levels()` | Read all 6 channel levels |
| `set_percent(ch, pct)` | Set channel by percentage 0.0–100.0 |
| `all_off()` | Turn all channels off immediately |
| `all_on()` | Set all channels to full power |
| `fade(ch, target, duration_s)` | Linear fade one channel |
| `fade_all(targets, duration_s)` | Fade all 6 channels simultaneously |
| `go_to_scene(scene)` | Transition to a Scene object |
| `set_channel_mode(ch, burst)` | Set mode for one channel |
| `set_channels_mode(modes)` | Set mode for all 6 channels (list of bool) |
| `set_burst_mode(enable)` | Set all channels to same mode |
| `get_channel_modes()` | Read current modes for all channels |
| `set_config(force_60hz, invert)` | Update CONFIG register fields |
| `fan_set_auto()` | Return fan to thermal curve |
| `fan_set_manual(duty)` | Manual fan control 0–255 |
| `fan_force_off(suppress)` | Suppress fan (quiet hours) |
| `fan_get_duty()` | Read current fan duty byte |
| `fan_get_speed_pct()` | Read fan speed as percentage |
| `read_temperature()` | Read NTC temperature in °C |
| `save_to_eeprom()` | Persist levels and config to EEPROM |
| `load_from_eeprom()` | Restore levels from EEPROM |
| `clear_fault()` | Attempt over-temperature fault clear |
| `status()` | Read all registers, returns TriacStatus |
| `is_zc_locked()` | Quick ZC lock check |
| `start_monitor(...)` | Start background status polling thread |
| `stop_monitor()` | Stop background monitor thread |
| `close()` | Safe shutdown — all off, release bus |

#### TriacStatus

Returned by `status()`. Raises `TriacOvertemp` if overtemp is latched.

| Field | Type | Description |
|---|---|---|
| `zc_locked` | bool | ZC events are being received |
| `overtemp` | bool | Over-temperature latch is set |
| `fan_on` | bool | Fan is currently spinning |
| `freq_60hz` | bool | 60 Hz mains detected |
| `levels` | list[int] | Current levels for CH1–CH6 |
| `temp_raw` | int | Raw ADC byte from NTC |
| `temp_celsius` | float | Temperature in °C |
| `fan_duty` | int | Current fan PWM duty 0–255 |
| `ch_mode` | int | Raw CH_MODE register byte |
| `channel_modes` | list[str] | `['phase'` or `'burst'`] for each channel |
| `fan_manual` | bool | Fan manual override active |
| `fan_force_off` | bool | Fan suppression active |

#### Multiple Boards

```python
from triac_controller import TriacBus

with TriacBus(addresses=[0x40, 0x41, 0x42, 0x43]) as bus:
    bus[0x40].set_all([128] * 6)      # all channels on board 0x40
    bus[0x41].set_channel(1, 64)
    bus.all_off()                      # all off on all boards
```

#### Background Monitor

```python
tc.start_monitor(
    poll_s        = 0.5,
    on_zc_lost    = lambda: print("ZC lost!"),
    on_overtemp   = lambda c, r: print(f"Overtemp: {c:.1f}°C"),
    on_fan_change = lambda d, p: print(f"Fan → {p:.1f}%"),
)
```

---

## HAT ID EEPROM

The board includes an AT24C32D EEPROM (U10) at I²C address 0x50 on the dedicated HAT ID bus (RPi GPIO0/GPIO1, header pins 27/28). This is **separate** from the main I²C-1 bus used by the ATtiny1616.

To build and flash the HAT EEPROM image:

```bash
# Enable I2C-0 (ID bus) — add to /boot/config.txt then reboot
echo "dtparam=i2c_vc=on" | sudo tee -a /boot/config.txt
sudo reboot

# Clone RPi HAT utilities
git clone https://github.com/raspberrypi/hats
cd hats/eepromutils && make

# Edit hat_eeprom.txt — generate your own UUID:
python3 -c "import uuid; print(uuid.uuid4())"
# Paste it into the uuid field in hat_eeprom.txt

# Compile and flash
./eepmake ../../hat_eeprom.txt triac_hat.eep
sudo ./eepflash.sh -w -f=triac_hat.eep -t=24c32 -d=0 -a=50

# Verify
cat /proc/device-tree/hat/product
cat /proc/device-tree/hat/vendor
```

The EEPROM is optional for personal use. The ATtiny1616 will communicate over I²C-1 regardless of whether the EEPROM is programmed.

---

## PCB Layout Guidelines

Observe these rules to ensure safety and correct operation:

**Creepage distance:** Maintain a minimum of **6 mm** between any mains-side conductor (GNDPWR, AC/L, AC/N, TRIAC T1/T2 tracks) and any logic-side conductor (GND, +5V, signal traces). Consider milling a slot in the PCB under the optocouplers to increase the creepage path.

**Mains trace width:** Use dual parallel 2mm traces on both top and bottom copper layers for all TRIAC load-current paths, stitched with vias every 5–10 mm. This gives approximately 5A capacity at 10°C rise with 1oz copper, matching the BT136 rating.

**TRIAC heatsinking:** The BT136 TO-220 packages require a heatsink for sustained loads above 3A per channel. Mount the NTC thermistor (RT1) directly on the common heatsink for accurate thermal sensing.

**Decoupling:** Place C9 (100nF, 1206) as close as possible to the ATtiny1616 VCC pin. Place C7/C8 on the 78L05 input and output.

**Ground planes:** Keep GNDPWR and GND as separate copper pours with no direct connection between them on the PCB. The only return path between them is through the optocoupler circuits.

**Fusing:** Fit an external slow-blow fuse (1A) in series with the AC Live line before J1. This is not on the board and must be in the enclosure wiring.

---

## Repository Structure

```
├── firmware/
│   ├── main.c                    ATtiny1616 firmware (C, avr-gcc)
│   └── Makefile                  Build and flash targets
├── driver/
│   └── triac_controller.py       Raspberry Pi Python I²C driver
├── hardware/
│   ├── triac_controller_hat.kicad_sch   KiCad 7 schematic
│   ├── triac_controller_hat.kicad_pro   KiCad project file
│   └── BOM_Digikey_corrected.csv        Digikey BOM with part numbers
├── eeprom/
│   └── hat_eeprom.txt            RPi HAT EEPROM config (eepmake format)
└── README.md
```

---

## Licence

This project is released under the **MIT Licence**. See `LICENSE` for details.

Hardware designs (schematics, BOM) are released under **CERN Open Hardware Licence v2 — Permissive (CERN-OHL-P)**.

> **Disclaimer:** This project involves mains voltage. The authors accept no liability for damage, injury, or death resulting from the construction or use of this design. You build and use this hardware entirely at your own risk. Ensure compliance with all applicable electrical safety regulations in your jurisdiction.
