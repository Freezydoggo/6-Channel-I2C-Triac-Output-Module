# coding=utf-8
"""
Custom Output: 6-Channel TRIAC Phase-Angle / Burst-Fire Controller
===================================================================
Target firmware: ATtiny1616 I2C slave (6-Channel TRIAC + PWM Fan Controller)
Default I2C address: 0x40  (selectable 0x40–0x43 via PA4/PA5 address pins)

Register map
────────────
0x00  STATUS    R    Bit7=ZC_LOCK  Bit2=FAN_ON  Bit1=OVERTEMP  Bit0=60Hz
0x01  CH1_LEVEL R/W  0=off … 128=full on
0x02  CH2_LEVEL R/W
0x03  CH3_LEVEL R/W
0x04  CH4_LEVEL R/W
0x05  CH5_LEVEL R/W
0x06  CH6_LEVEL R/W
0x07  CONFIG    R/W  Bit0=force_60Hz  Bit1=invert  Bit2=ee_save  Bit3=ee_load
                     Bit4=fault_clr   Bit5=fan_manual  Bit6=fan_force_off
0x08  BURST_CH  W    7-byte burst: sets CH1–CH6 in one transaction
0x09  TEMP_RAW  R    ADC byte (higher=colder; ~50=60°C, ~28=80°C)
0x0A  ZC_COUNT  R    Rolling 8-bit zero-crossing counter
0x0B  FAN_DUTY  R/W  Fan PWM duty 0–255 (write requires CONFIG Bit5 set)
0x0C  CH_MODE   R/W  Per-channel burst-fire bitmask (bit n=1 → CH(n+1) burst)

Mycodo channel mapping
──────────────────────
Channels 0–5  →  firmware CH1–CH6  (level registers 0x01–0x06)
Each channel value is 0–100 % which is mapped to firmware level 0–128.
"""

import time

from mycodo.outputs.base_output import AbstractOutput
from mycodo.databases.models import OutputChannel
from mycodo.utils.database import db_retrieve_table_daemon

# ── Measurement / channel metadata ────────────────────────────────────────────

measurements_dict = {
    0: {"measurement": "duty_cycle", "unit": "percent"},
    1: {"measurement": "duty_cycle", "unit": "percent"},
    2: {"measurement": "duty_cycle", "unit": "percent"},
    3: {"measurement": "duty_cycle", "unit": "percent"},
    4: {"measurement": "duty_cycle", "unit": "percent"},
    5: {"measurement": "duty_cycle", "unit": "percent"},
}

channels_dict = {
    0: {"name": "Channel 1 (TRIAC)", "types": ["value"], "measurements": [0]},
    1: {"name": "Channel 2 (TRIAC)", "types": ["value"], "measurements": [1]},
    2: {"name": "Channel 3 (TRIAC)", "types": ["value"], "measurements": [2]},
    3: {"name": "Channel 4 (TRIAC)", "types": ["value"], "measurements": [3]},
    4: {"name": "Channel 5 (TRIAC)", "types": ["value"], "measurements": [4]},
    5: {"name": "Channel 6 (TRIAC)", "types": ["value"], "measurements": [5]},
}

# ── OUTPUT_INFORMATION dictionary ─────────────────────────────────────────────

OUTPUT_INFORMATION = {
    "output_name_unique": "TRIAC_6CH_ATTINY1616",
    "output_name": "6-Channel TRIAC Controller (ATtiny1616)",
    "output_manufacturer": "Custom",
    "output_library": "smbus2",
    "measurements_dict": measurements_dict,
    "channels_dict": channels_dict,
    "output_types": ["value"],

    "url_manufacturer": "",
    "url_datasheet": "",
    "url_product_purchase": [],

    "message": (
        "6-channel phase-angle / burst-fire TRIAC controller connected via I2C. "
        "Each channel accepts a level from 0 (off) to 100 % (full power). "
        "Internally the firmware maps 0–100 % to 0–128 counts. "
        "The board also exposes temperature readback, fan control, EEPROM "
        "save/load, and per-channel burst-fire mode via the Custom Options below."
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
    ],
    "options_disabled": ["interface"],

    # Custom options exposed in the Mycodo UI
    "custom_options": [
        {
            "id": "force_60hz",
            "type": "bool",
            "default_value": False,
            "name": "Force 60 Hz mains",
            "phrase": (
                "Set CONFIG Bit0. When enabled the firmware uses 60 Hz "
                "half-period timing instead of auto-detecting from the "
                "zero-crossing signal."
            ),
        },
        {
            "id": "invert_output",
            "type": "bool",
            "default_value": False,
            "name": "Invert output polarity",
            "phrase": (
                "Set CONFIG Bit1. Inverts the phase-angle sense so that "
                "level=0 means fully on and level=128 means fully off."
            ),
        },

        {
            "id": "eeprom_save_on_startup",
            "type": "bool",
            "default_value": False,
            "name": "Save settings to EEPROM on connect",
            "phrase": (
                "Trigger an EEPROM save (CONFIG Bit2) each time Mycodo "
                "(re)connects to the board. Useful when Mycodo is the sole "
                "configurator of the board."
            ),
        },
        {
            "id": "fan_manual_enable",
            "type": "bool",
            "default_value": False,
            "name": "Enable manual fan control",
            "phrase": (
                "Set CONFIG Bit5 to take over the fan from the automatic "
                "thermal curve. You must also set 'Manual fan duty cycle' "
                "below. Leaving this off lets the firmware run the fan "
                "automatically based on the NTC temperature."
            ),
        },
        {
            "id": "fan_manual_duty",
            "type": "integer",
            "default_value": 128,
            "constraints_pass": lambda val, err: (
                (True, None) if 0 <= val <= 255
                else (False, "Must be 0–255")
            ),
            "name": "Manual fan duty cycle (0–255)",
            "phrase": (
                "Written to register 0x0B (FAN_DUTY). Only takes effect "
                "when 'Enable manual fan control' is checked. "
                "0 = fan off, 255 = fan full speed."
            ),
        },
        {
            "id": "fan_force_off",
            "type": "bool",
            "default_value": False,
            "name": "Force fan off (quiet mode)",
            "phrase": (
                "Set CONFIG Bit6 to guarantee the fan stays off regardless "
                "of temperature (except overtemp — safety always wins). "
                "Use during quiet hours. This bit is overridden by the "
                "overtemp latch."
            ),
        },
    ],

    # Per-channel custom options
    "custom_channel_options": [
        {
            "id": "name",
            "type": "text",
            "default_value": "",
            "required": False,
            "name": "Channel name",
            "phrase": "Descriptive label for this TRIAC channel.",
        },
        {
            "id": "burst_fire",
            "type": "bool",
            "default_value": False,
            "name": "Burst-fire mode",
            "phrase": (
                "When checked, this channel uses burst-fire (whole-cycle on/off) "
                "control instead of phase-angle (leading-edge) control. "
                "Burst-fire produces less RF interference and suits resistive "
                "loads such as heating elements. "
                "Phase-angle (unchecked) gives finer resolution and faster "
                "response but generates more EMI — preferred for lighting."
            ),
        },
        {
            "id": "state_startup",
            "type": "select",
            "default_value": "off",
            "options_select": [
                ("off", "Off (level = 0)"),
                ("on",  "On (level = last saved / EEPROM)"),
                ("set", "Set to startup value below"),
            ],
            "name": "Startup state",
            "phrase": "Channel state when Mycodo starts.",
        },
        {
            "id": "startup_value",
            "type": "float",
            "default_value": 0.0,
            "constraints_pass": lambda val, err: (
                (True, None) if 0.0 <= val <= 100.0
                else (False, "Must be 0–100 %")
            ),
            "name": "Startup value (%)",
            "phrase": "Level (0–100 %) applied at startup when state is 'Set'.",
        },
        {
            "id": "state_shutdown",
            "type": "select",
            "default_value": "off",
            "options_select": [
                ("off",  "Off (level = 0)"),
                ("last", "Leave at last commanded level"),
            ],
            "name": "Shutdown state",
            "phrase": "Channel state when Mycodo shuts down.",
        },
    ],
}


# ── Helper: percent → firmware level ──────────────────────────────────────────

def _pct_to_level(pct: float) -> int:
    """Convert 0–100 % to 0–128 firmware level."""
    level = int(round(pct / 100.0 * 128.0))
    return max(0, min(128, level))


# ── Output class ──────────────────────────────────────────────────────────────

class OutputModule(AbstractOutput):
    """
    Mycodo output module for the 6-channel ATtiny1616 TRIAC controller.

    The firmware exposes a flat register file over I2C.  This module:
      • Reads custom options to build the CONFIG and CH_MODE registers.
      • Writes individual channel levels (0x01–0x06) as Value outputs.
      • Supports burst-fire and phase-angle modes per channel.
      • Exposes temperature and ZC heartbeat as diagnostic log entries.
    """

    def __init__(self, output, testing=False):
        super().__init__(output, testing=testing, name=__name__)

        self.i2c_address = None
        self.i2c_bus = None
        self.bus = None

        # Cached channel levels (0–128)
        self._level = [0] * 6

        # Module-level custom options (set by setup())
        self.force_60hz = False
        self.invert_output = False
        self.eeprom_save_on_startup = False
        self.fan_manual_enable = False
        self.fan_manual_duty = 128
        self.fan_force_off = False

        self.setup_output_variables(OUTPUT_INFORMATION)

        if not testing:
            self.try_initialize()

    # ── Mycodo lifecycle ──────────────────────────────────────────────────────

    def initialize(self):
        """Open I2C bus and configure the board."""
        from smbus2 import SMBus

        self.i2c_address = int(str(self.output.i2c_location), 16)
        self.i2c_bus = self.output.i2c_bus

        # Read custom options
        self.force_60hz            = self.get_custom_option("force_60hz")
        self.invert_output         = self.get_custom_option("invert_output")
        self.eeprom_save_on_startup= self.get_custom_option("eeprom_save_on_startup")
        self.fan_manual_enable     = self.get_custom_option("fan_manual_enable")
        self.fan_manual_duty       = self.get_custom_option("fan_manual_duty")
        self.fan_force_off         = self.get_custom_option("fan_force_off")

        try:
            self.bus = SMBus(self.i2c_bus)
        except Exception as exc:
            self.logger.error(f"Could not open I2C bus {self.i2c_bus}: {exc}")
            return

        try:
            self._apply_config()
            self._apply_ch_mode()

            if self.fan_manual_enable:
                self._write_register(0x0B, self.fan_manual_duty)

            if self.eeprom_save_on_startup:
                self._trigger_eeprom_save()

            # Apply per-channel startup states
            self._apply_startup_states()

            self.output_setup = True
            self.logger.info(
                f"TRIAC controller initialised at I2C {self.i2c_address:#04x} "
                f"on bus {self.i2c_bus}."
            )
        except Exception as exc:
            self.logger.error(f"Initialisation failed: {exc}")

    def shutdown(self):
        """Apply shutdown states and close the I2C bus."""
        if not self.output_setup:
            return
        try:
            self._apply_shutdown_states()
        except Exception as exc:
            self.logger.error(f"Error during shutdown: {exc}")
        finally:
            if self.bus:
                self.bus.close()
                self.bus = None
            self.output_setup = False

    # ── Mycodo output interface ───────────────────────────────────────────────

    def output_switch(self, state, output_type=None, amount=None, output_channel=None):
        """
        Called by Mycodo to change an output.

        state        : 'on' | 'off'
        output_type  : 'value' (Mycodo sends level in percent)
        amount       : float 0–100 % (only when state='on' and type='value')
        output_channel: 0–5
        """
        if not self.output_setup:
            msg = "Output not set up — cannot switch."
            self.logger.error(msg)
            return msg

        if output_channel is None or output_channel not in range(6):
            msg = f"Invalid channel index: {output_channel}"
            self.logger.error(msg)
            return msg

        if state == "off" or amount is None or amount <= 0:
            level = 0
        else:
            level = _pct_to_level(float(amount))

        reg_addr = 0x01 + output_channel  # 0x01–0x06 for CH1–CH6
        try:
            self._write_register(reg_addr, level)
            self._level[output_channel] = level
            self.logger.debug(
                f"CH{output_channel + 1} → level {level}/128 "
                f"({amount if amount else 0:.1f} %)"
            )
        except Exception as exc:
            msg = f"I2C write error (CH{output_channel + 1}): {exc}"
            self.logger.error(msg)
            return msg

    def is_on(self, output_channel=None):
        if output_channel is None:
            return False
        return self._level[output_channel] > 0

    def is_setup(self):
        return self.output_setup

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _write_register(self, reg: int, value: int):
        """Write a single byte to a firmware register."""
        self.bus.write_byte_data(self.i2c_address, reg, value & 0xFF)

    def _read_register(self, reg: int) -> int:
        """Read a single byte from a firmware register."""
        return self.bus.read_byte_data(self.i2c_address, reg) & 0xFF

    def _build_config_byte(self, extra_bits: int = 0) -> int:
        """Assemble CONFIG register from current option flags."""
        cfg = extra_bits
        if self.force_60hz:
            cfg |= 0x01   # Bit0: force_60Hz
        if self.invert_output:
            cfg |= 0x02   # Bit1: invert
        if self.fan_manual_enable:
            cfg |= 0x20   # Bit5: fan_manual
        if self.fan_force_off:
            cfg |= 0x40   # Bit6: fan_force_off
        return cfg

    def _apply_config(self):
        """Write CONFIG register (0x07) to the board."""
        cfg = self._build_config_byte()
        self._write_register(0x07, cfg)
        self.logger.debug(f"CONFIG ← {cfg:#04x}")

    def _apply_ch_mode(self):
        """
        Build and write CH_MODE register (0x0C) from per-channel burst_fire
        options.  Bit n = 1 → channel (n+1) in burst-fire mode.
        """
        mask = 0
        for ch_idx in range(6):
            if self.get_custom_channel_option(ch_idx, "burst_fire"):
                mask |= (1 << ch_idx)
        mask &= 0x3F  # only bits 0–5 are valid
        self._write_register(0x0C, mask)
        self.logger.debug(
            f"CH_MODE ← {mask:#04x}  "
            f"(burst channels: "
            f"{[ch + 1 for ch in range(6) if mask & (1 << ch)] or 'none'})"
        )

    def _trigger_eeprom_save(self):
        """Pulse CONFIG Bit2 (ee_save) to persist current channel levels."""
        cfg = self._build_config_byte(extra_bits=0x04)  # Bit2 = ee_save
        self._write_register(0x07, cfg)
        # Give the ATtiny ~250 ms to complete the write cycle, then clear
        time.sleep(0.25)
        self._write_register(0x07, self._build_config_byte())
        self.logger.info("EEPROM save triggered.")

    def _apply_startup_states(self):
        """Apply per-channel startup state options."""
        channels = db_retrieve_table_daemon(
            OutputChannel,
            filter_by_output_id=self.output.unique_id,
        )
        for ch_obj in channels:
            ch_idx = ch_obj.channel
            state  = self.get_custom_channel_option(ch_idx, "state_startup")
            if state == "off":
                self._write_register(0x01 + ch_idx, 0)
                self._level[ch_idx] = 0
            elif state == "set":
                pct   = self.get_custom_channel_option(ch_idx, "startup_value")
                level = _pct_to_level(float(pct))
                self._write_register(0x01 + ch_idx, level)
                self._level[ch_idx] = level
            # "on" → leave whatever the EEPROM loaded at boot

    def _apply_shutdown_states(self):
        """Apply per-channel shutdown state options."""
        channels = db_retrieve_table_daemon(
            OutputChannel,
            filter_by_output_id=self.output.unique_id,
        )
        for ch_obj in channels:
            ch_idx = ch_obj.channel
            state  = self.get_custom_channel_option(ch_idx, "state_shutdown")
            if state == "off":
                self._write_register(0x01 + ch_idx, 0)
                self._level[ch_idx] = 0
            # "last" → do nothing — board holds the level until next ZC loss
            #          or power cycle

    # ── Diagnostic helpers (callable from Mycodo Functions / Conditionals) ────

    def read_status(self) -> dict:
        """
        Read the STATUS register and return a dict with parsed flags.
        Useful for Mycodo Conditional Functions.
        """
        if not self.output_setup:
            return {}
        sta = self._read_register(0x00)
        return {
            "zc_lock":  bool(sta & 0x80),
            "fan_on":   bool(sta & 0x04),
            "overtemp": bool(sta & 0x02),
            "hz_60":    bool(sta & 0x01),
            "raw":      sta,
        }

    def read_temperature_adc(self) -> int:
        """
        Return raw NTC ADC byte (0x09).
        Higher = colder: ~142→20°C, ~50→60°C, ~28→80°C, ~25→85°C.
        """
        if not self.output_setup:
            return -1
        return self._read_register(0x09)

    def read_zc_counter(self) -> int:
        """Return rolling zero-crossing counter (0x0A) as watchdog heartbeat."""
        if not self.output_setup:
            return -1
        return self._read_register(0x0A)

    def set_all_channels(self, pct: float):
        """
        Convenience: write all 6 channels to the same level in one burst.
        Uses the firmware's burst write register (0x08) for atomicity.
        pct: 0–100 %
        """
        if not self.output_setup:
            return
        level = _pct_to_level(pct)
        # Burst write: first byte selects 0x08, then 6 level bytes for CH1–CH6
        self.bus.write_i2c_block_data(
            self.i2c_address,
            0x08,                          # REG_BURST_CH
            [level] * 6,
        )
        self._level = [level] * 6
        self.logger.debug(f"Burst: all channels → {level}/128 ({pct:.1f} %)")

    def clear_fault(self):
        """
        Send CONFIG Bit4 (fault_clr) to attempt to clear an overtemp latch.
        The firmware will only honour this when the temperature has dropped
        to ≤ 75°C (ADC byte ≥ 33).
        """
        if not self.output_setup:
            return
        cfg = self._build_config_byte(extra_bits=0x10)  # Bit4 = fault_clr
        self._write_register(0x07, cfg)
        time.sleep(0.05)
        # Re-apply clean config (self-clearing bit)
        self._apply_config()
        self.logger.info("Fault-clear command sent.")
