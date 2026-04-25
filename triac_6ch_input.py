# coding=utf-8
"""
Custom Input: 6-Channel TRIAC Controller — Temperature & Status Monitor
========================================================================
Companion input module to triac_6ch_phase_angle.py (custom output).

Reads three registers from the ATtiny1616 TRIAC board each poll cycle:

  0x09  TEMP_RAW   — NTC ADC byte → converted to °C (see lookup table below)
  0x0A  ZC_COUNT   — rolling zero-crossing counter (used to derive Hz)
  0x00  STATUS     — parsed into boolean flags for Mycodo alerts

Measurement channels
────────────────────
  0  temperature   °C     — NTC board temperature (converted from ADC byte)
  1  boolean       none   — overtemp flag  (1 = firmware overtemp latch active)
  2  boolean       none   — zc_lock flag   (1 = zero-crossing signal healthy)
  3  frequency     Hz     — mains frequency derived from ZC_COUNT delta
  4  boolean       none   — fan_on flag    (1 = fan currently spinning)

NTC conversion
──────────────
The firmware returns the upper 8 bits of a 4-sample-accumulated 10-bit ADC
reading.  The NTC is a 10 kΩ B3950 to GND with a 10 kΩ fixed resistor to
VCC (5 V), so the ADC byte DECREASES as temperature increases.

Known calibration points from the firmware source:
  ADC byte  →  Temperature
  ────────     ───────────
    142          20 °C
     88          40 °C
     67          50 °C
     58          55 °C
     50          60 °C
     33          75 °C   (overtemp-clear hysteresis)
     28          80 °C
     25          85 °C   (overtemp shutdown)

A piecewise-linear interpolation is used between these anchor points.
For bytes outside the calibrated range the nearest endpoint is clamped.

Overtemp alert guidance
───────────────────────
After importing this module, set up a Mycodo Alert on channel 0 (temperature):
  • Condition : Above threshold
  • Threshold : 75 (°C) — matches the firmware's overtemp-clear hysteresis
  • Hysteresis: 5 (°C)  — avoids chattering near the threshold
  • Email/SMS  : your notification method

You can also create a Conditional Function on channel 1 (overtemp boolean):
  if measurement(overtemp) == 1: send alert and optionally write fault_clr
  to the output module's CONFIG register via a Python Command Output.
"""

from mycodo.inputs.base_input import AbstractInput

# ── Measurement metadata ───────────────────────────────────────────────────────

measurements_dict = {
    0: {"measurement": "temperature",  "unit": "C"},
    1: {"measurement": "boolean",      "unit": "none"},  # overtemp flag
    2: {"measurement": "boolean",      "unit": "none"},  # zc_lock flag
    3: {"measurement": "frequency",    "unit": "Hz"},    # mains frequency
    4: {"measurement": "boolean",      "unit": "none"},  # fan_on flag
}

# ── INPUT_INFORMATION dictionary ───────────────────────────────────────────────

INPUT_INFORMATION = {
    "input_name_unique": "TRIAC_6CH_ATTINY1616_MONITOR",
    "input_manufacturer": "Custom",
    "input_name": "6-Channel TRIAC Controller Monitor (ATtiny1616)",
    "input_library": "smbus2",
    "measurements_name": "Temperature/Status",
    "measurements_dict": measurements_dict,

    "url_manufacturer": "",
    "url_datasheet": "",

    "message": (
        "Reads temperature (converted from NTC ADC byte), overtemp flag, "
        "zero-crossing lock status, mains frequency, and fan state from the "
        "ATtiny1616 TRIAC controller board. Use the temperature channel "
        "(channel 0) or the overtemp boolean (channel 1) to trigger Mycodo "
        "alerts or Conditional Functions when the board overheats. "
        "Set the poll period to 10–30 seconds; the firmware updates TEMP_RAW "
        "every ~200 ms so polling faster than 5 s provides no benefit."
    ),

    "dependencies_module": [
        ("pip-pypi", "smbus2", "smbus2"),
    ],

    "interfaces": ["I2C"],
    "i2c_location": ["0x40", "0x41", "0x42", "0x43"],
    "i2c_address_editable": False,

    "options_enabled": [
        "i2c_location",
        "i2c_bus",
        "period",
        "pre_output",
    ],
    "options_disabled": ["interface"],

    "custom_options": [
        {
            "id": "zc_poll_interval",
            "type": "float",
            "default_value": 1.0,
            "constraints_pass": lambda val, err: (
                (True, None) if 0.5 <= val <= 60.0
                else (False, "Must be between 0.5 and 60 seconds")
            ),
            "name": "ZC frequency sample window (seconds)",
            "phrase": (
                "The mains frequency is derived by reading ZC_COUNT twice, "
                "separated by this delay, and computing the delta. The "
                "ZC_COUNT register rolls over at 256, so keep this value "
                "below ~5 s for 50 Hz mains (256 / 100 crossings/s ≈ 2.56 s "
                "max before rollover ambiguity). Default 1 s is safe for both "
                "50 Hz and 60 Hz mains."
            ),
        },
        {
            "id": "temp_warn_threshold",
            "type": "float",
            "default_value": 70.0,
            "name": "High-temperature log warning threshold (°C)",
            "phrase": (
                "If the converted board temperature exceeds this value, a "
                "WARNING entry is written to the Mycodo log. This is separate "
                "from any Alert you configure in the Mycodo UI. "
                "Set to 0 to disable. The firmware's own overtemp shutdown "
                "trips at 85 °C regardless of this setting."
            ),
        },
    ],
}

# ── NTC piecewise-linear lookup table ─────────────────────────────────────────
#
# Anchor points derived directly from the firmware source comments.
# Stored as (adc_byte, temp_C) pairs, sorted descending by adc_byte
# (i.e. ascending by temperature) so we can binary-search / iterate.
#
# IMPORTANT: adc_byte decreases as temperature increases.  The interpolation
# below accounts for this inverted relationship.

_NTC_TABLE = [
    (142, 20.0),
    ( 88, 40.0),
    ( 67, 50.0),
    ( 58, 55.0),
    ( 50, 60.0),
    ( 33, 75.0),
    ( 28, 80.0),
    ( 25, 85.0),
]  # sorted high-byte → low-byte  (cold → hot)


def _adc_byte_to_celsius(adc_byte: int) -> float:
    """
    Convert an ATtiny1616 NTC ADC byte to degrees Celsius using
    piecewise-linear interpolation over the firmware's calibration points.

    adc_byte : 0–255, higher = colder (inverted NTC divider).
    Returns  : temperature in °C, clamped to the calibrated range.
    """
    # Clamp to calibrated range
    if adc_byte >= _NTC_TABLE[0][0]:
        return _NTC_TABLE[0][1]   # colder than 20 °C — return 20 °C
    if adc_byte <= _NTC_TABLE[-1][0]:
        return _NTC_TABLE[-1][1]  # hotter than 85 °C — return 85 °C

    # Find the two surrounding anchor points
    for i in range(len(_NTC_TABLE) - 1):
        high_byte, low_temp = _NTC_TABLE[i]
        low_byte,  high_temp = _NTC_TABLE[i + 1]
        if low_byte <= adc_byte <= high_byte:
            # Linear interpolation.
            # As adc_byte falls from high_byte → low_byte, temp rises.
            fraction = (high_byte - adc_byte) / (high_byte - low_byte)
            return low_temp + fraction * (high_temp - low_temp)

    # Should never reach here given the clamps above
    return float("nan")


# ── Input class ────────────────────────────────────────────────────────────────

class InputModule(AbstractInput):
    """
    Mycodo input module for the ATtiny1616 6-channel TRIAC controller.

    Each poll cycle reads:
      • TEMP_RAW  (0x09) — converts ADC byte to °C via NTC lookup table
      • STATUS    (0x00) — extracts overtemp, zc_lock, fan_on boolean flags
      • ZC_COUNT  (0x0A) — sampled twice to derive mains frequency

    All five values are stored as separate Mycodo measurements so they can
    be graphed independently and used in Alerts / Conditional Functions.
    """

    def __init__(self, input_dev, testing=False):
        super().__init__(input_dev, testing=testing, name=__name__)

        self.i2c_address = None
        self.i2c_bus_num = None
        self.bus = None

        self.zc_poll_interval = 1.0
        self.temp_warn_threshold = 70.0

        self.setup_custom_options(
            INPUT_INFORMATION["custom_options"], input_dev
        )

        if not testing:
            self.try_initialize()

    # ── Mycodo lifecycle ───────────────────────────────────────────────────────

    def initialize(self):
        from smbus2 import SMBus

        self.i2c_address = int(str(self.input_dev.i2c_location), 16)
        self.i2c_bus_num = self.input_dev.i2c_bus

        self.zc_poll_interval   = self.get_custom_option("zc_poll_interval")
        self.temp_warn_threshold = self.get_custom_option("temp_warn_threshold")

        try:
            self.bus = SMBus(self.i2c_bus_num)
            self.logger.info(
                f"TRIAC monitor opened I2C {self.i2c_address:#04x} "
                f"on bus {self.i2c_bus_num}."
            )
        except Exception as exc:
            self.logger.error(f"Could not open I2C bus {self.i2c_bus_num}: {exc}")

    def get_measurement(self):
        """
        Called by Mycodo on each poll cycle.
        Returns a dict of {channel_index: value} for all five measurements.
        """
        if not self.bus:
            self.logger.error("I2C bus not open — skipping measurement.")
            return None

        self.return_dict = self.setup_return_dict(measurements_dict)

        try:
            # ── 1. Temperature (channel 0) ─────────────────────────────────
            adc_byte = self._read_reg(0x09)
            temp_c   = _adc_byte_to_celsius(adc_byte)
            self.value_set(0, round(temp_c, 1))

            if (self.temp_warn_threshold > 0
                    and temp_c >= self.temp_warn_threshold):
                self.logger.warning(
                    f"Board temperature {temp_c:.1f} °C exceeds warning "
                    f"threshold {self.temp_warn_threshold:.1f} °C "
                    f"(ADC byte={adc_byte})."
                )

            # ── 2. STATUS register → boolean flags (channels 1, 2, 4) ──────
            status   = self._read_reg(0x00)
            overtemp = int(bool(status & 0x02))   # Bit1
            zc_lock  = int(bool(status & 0x80))   # Bit7
            fan_on   = int(bool(status & 0x04))   # Bit2

            self.value_set(1, overtemp)
            self.value_set(2, zc_lock)
            self.value_set(4, fan_on)

            if overtemp:
                self.logger.warning(
                    f"OVERTEMP latch is ACTIVE (ADC byte={adc_byte}, "
                    f"temp≈{temp_c:.1f} °C). All TRIAC gates are disabled "
                    f"by the firmware. Clear the fault via the output module "
                    f"once temperature drops below 75 °C."
                )

            if not zc_lock:
                self.logger.warning(
                    "ZC_LOCK is CLEAR — zero-crossing signal lost or mains "
                    "not present. TRIAC outputs are disabled by the firmware."
                )

            # ── 3. Mains frequency from ZC_COUNT delta (channel 3) ─────────
            hz = self._measure_frequency()
            if hz is not None:
                self.value_set(3, round(hz, 1))

        except Exception as exc:
            self.logger.error(f"Measurement error: {exc}")
            return None

        return self.return_dict

    # ── Internal helpers ───────────────────────────────────────────────────────

    def _read_reg(self, reg: int) -> int:
        """Read a single byte from the given register address."""
        return self.bus.read_byte_data(self.i2c_address, reg) & 0xFF

    def _measure_frequency(self):
        """
        Derive mains frequency from two ZC_COUNT samples separated by
        zc_poll_interval seconds.

        ZC_COUNT increments once per half-cycle, so:
          frequency = (delta_count / 2) / elapsed_seconds

        The counter is 8-bit and rolls over at 256; we handle one rollover.
        Returns None if the count delta is zero (mains absent / ZC lost).
        """
        import time

        try:
            count_a = self._read_reg(0x0A)
            time.sleep(self.zc_poll_interval)
            count_b = self._read_reg(0x0A)
        except Exception as exc:
            self.logger.error(f"ZC_COUNT read error: {exc}")
            return None

        # Handle 8-bit rollover
        delta = (count_b - count_a) & 0xFF

        if delta == 0:
            self.logger.debug("ZC_COUNT delta=0 — mains may be absent.")
            return None

        # Each count = one half-cycle crossing, so divide by 2 for full cycles
        hz = (delta / 2.0) / self.zc_poll_interval
        return hz
