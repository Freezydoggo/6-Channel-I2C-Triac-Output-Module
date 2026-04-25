/*
 * 6-Channel TRIAC Phase-Angle / Burst-Fire Controller
 * with PWM Fan Control
 *
 * Target:    ATtiny1616 @ 20 MHz (internal oscillator)
 * Toolchain: avr-gcc + avr-libc
 *
 * ── Pin assignment ───────────────────────────────────────────────────────────
 *
 *   PA1 (TWI SDA)   — I2C data to Raspberry Pi
 *   PA2 (TWI SCL)   — I2C clock from Raspberry Pi
 *   PA3             — Zero-crossing input (PC817 optocoupler, active-low)
 *   PA4             — I2C address bit 0  (pull-up; tie to GND to set bit)
 *   PA5             — I2C address bit 1  (pull-up; tie to GND to set bit)
 *   PA6             — ADC — NTC thermistor (10kΩ NTC to GND, 10kΩ to VCC)
 *   PB0             — CH1 gate drive (EL357N / MOC3021)
 *   PB1             — CH2 gate drive
 *   PB2             — CH3 gate drive
 *   PB3             — CH4 gate drive
 *   PC0             — CH5 gate drive
 *   PC1             — CH6 gate drive
 *   PC2 (TCB0 WO)   — Fan PWM output → N-channel MOSFET gate
 *   PB4             — GREEN status LED (anode via 330Ω to +5V)  [NEW]
 *   PC3             — RED   status LED (anode via 330Ω to +5V)  [NEW]
 *
 * ── Status LED behaviour ─────────────────────────────────────────────────────
 *
 *   State              GREEN          RED       Meaning
 *   ─────────────────  ─────────────  ────────  ───────────────────────────────
 *   Boot / no ZC       OFF            ON solid  Initialising, waiting for mains
 *   Healthy            ON solid       OFF       ZC locked, I2C active, no fault
 *   I2C idle (>5 s)    1 Hz blink     OFF       ZC ok but RPi silent/crashed
 *   Fault              OFF            ON solid  Overtemp OR ZC watchdog timeout
 *   EEPROM save        ON (1 flash)   ON (1 flash) Brief both-on confirms save
 *
 *   Rules:
 *   • RED is on at power-on (boot state) and turns off the instant GREEN
 *     turns on — no ambiguous period where both are off.
 *   • GREEN and RED are never both on except during the 1-flash EEPROM
 *     save confirmation (~210 ms).
 *   • Faults latch RED until the fault condition is resolved.
 *   • I2C activity is detected by a write counter incremented in the TWI
 *     ISR.  If no writes arrive for I2C_IDLE_OVERFLOWS TCA overflows
 *     (~5 s), the GREEN LED begins blinking at ~1 Hz to indicate the RPi
 *     is not communicating.  The first write after idle restores solid GREEN.
 *
 * ── LED circuit ──────────────────────────────────────────────────────────────
 *
 *   PB4 ──[330Ω]──[GREEN LED]── GND   (LED anode at pin side)
 *   PC3 ──[330Ω]──[RED   LED]── GND
 *   Pin HIGH = LED ON.  ~8.8 mA per LED at 5V (well within 40 mA pin limit).
 *
 *   PC2 ──[100Ω]── MOSFET gate (e.g. 2N7002 or IRLZ44N)
 *                  MOSFET drain  → fan negative terminal
 *                  MOSFET source → GND
 *   +5V → fan positive terminal
 *   1N4148 flyback diode across fan (cathode to +5V, anode to drain)
 *
 *   TCB0 runs in 8-bit PWM mode at 78 kHz (20 MHz / 256), well above
 *   audible range.  The MOSFET switches the fan's GND leg so the fan
 *   runs when PC2 is HIGH.
 *
 * ── Fan thermal curve ────────────────────────────────────────────────────────
 *
 *   NTC divider: 10kΩ fixed (VCC) + 10kΩ B3950 (GND).
 *   Hotter temperature → lower NTC resistance → lower ADC byte.
 *
 *   ADC byte thresholds (10-bit ADC, 4-sample accumulation, top 8 bits):
 *     ≥ 50  (<60°C)  — fan OFF
 *     48–29 (60–79°C)— fan ramps linearly from FAN_MIN_DUTY to 255
 *     ≤ 28  (≥80°C)  — fan FULL speed (duty = 255)
 *
 *   A minimum duty cycle (FAN_MIN_DUTY = 80/255 ≈ 31%) is applied when
 *   the fan first turns on to guarantee the motor overcomes static friction.
 *
 *   Hysteresis: fan turns OFF only when ADC byte rises back to ≥ 55
 *   (~57°C), preventing rapid on/off cycling near 60°C.
 *
 * ── I2C address selection ────────────────────────────────────────────────────
 *
 *   PA4 PA5  │  Address
 *   ─────────┼──────────
 *    Hi  Hi  │  0x40  (default — both floating)
 *    Lo  Hi  │  0x41  (PA4 to GND)
 *    Hi  Lo  │  0x42  (PA5 to GND)
 *    Lo  Lo  │  0x43  (both to GND)
 *
 * ── Register map (I2C) ───────────────────────────────────────────────────────
 *
 *   0x00  STATUS    R    Bit7=ZC_LOCK  Bit2=FAN_ON  Bit1=OVERTEMP  Bit0=60Hz
 *   0x01  CH1_LEVEL R/W  0=off … 128=full on
 *   0x02  CH2_LEVEL R/W
 *   0x03  CH3_LEVEL R/W
 *   0x04  CH4_LEVEL R/W
 *   0x05  CH5_LEVEL R/W
 *   0x06  CH6_LEVEL R/W
 *   0x07  CONFIG    R/W  Bit0=force_60Hz  Bit1=invert
 *                        Bit2=ee_save*    Bit3=ee_load*  Bit4=fault_clr*
 *                        Bit5=fan_manual  Bit6=fan_force_off
 *                        (* self-clearing after ~210 ms)
 *                        NOTE: global burst_mode removed — use CH_MODE (0x0C).
 *   0x08  BURST_CH  W    7-byte burst: sets CH1–CH6 levels in one transaction
 *   0x09  TEMP_RAW  R    ADC byte (higher = colder; ~50=60°C, ~28=80°C)
 *   0x0A  ZC_COUNT  R    Rolling 8-bit ZC counter (watchdog heartbeat)
 *   0x0B  FAN_DUTY  R/W  Current fan PWM duty 0–255.
 *                        Read: actual running duty set by firmware.
 *                        Write: manual duty override (requires CONFIG
 *                               Bit5 fan_manual = 1 to take effect).
 *   0x0C  CH_MODE   R/W  Per-channel firing mode bitmask (bits 0–5 used).
 *                        Bit n = 0 → channel (n+1) uses phase-angle control.
 *                        Bit n = 1 → channel (n+1) uses burst-fire control.
 *                        e.g. 0x07 = CH1+CH2+CH3 burst, CH4–CH6 phase-angle.
 *                        Writing this register resets burst accumulators for
 *                        channels transitioning from phase-angle → burst-fire.
 *
 * ── Fan manual override ───────────────────────────────────────────────────────
 *
 *   Set CONFIG Bit6 (fan_manual) to take control of the fan from the RPi.
 *   Write the desired duty (0–255) to FAN_DUTY.  The thermal curve is
 *   suspended while fan_manual is set.  Bit7 (fan_force_off) overrides
 *   both auto and manual modes to guarantee the fan is off (e.g. during
 *   quiet hours), but the overtemp latch still forces full speed regardless
 *   of either bit — safety always wins.
 *
 * ── EEPROM storage ───────────────────────────────────────────────────────────
 *
 *   Bytes 0–5: CH1–CH6 levels
 *   Byte  6:   CONFIG (volatile bits masked out)
 *   Byte  7:   XOR checksum with magic 0xA5
 *   Fan duty and thresholds are not saved (always determined at runtime
 *   from live temperature).
 *
 * ── ZC watchdog ──────────────────────────────────────────────────────────────
 *
 *   3 TCA overflows (~630 ms) without a ZC edge → all gates forced low,
 *   STA_ZC_LOCK cleared.  Fan continues running normally during ZC loss —
 *   the thermal situation does not change just because AC was lost.
 *
 * ── Over-temperature shutdown ────────────────────────────────────────────────
 *
 *   ADC byte ≤ 25 (≥85°C) → all TRIAC gates off, STA_OVERTEMP latches.
 *   Fan is forced to full speed (duty=255) regardless of all other settings.
 *   Latch clears via CONFIG Bit5 (fault_clr) only when temp ≤ ADC byte 33
 *   (~75°C, 10°C hysteresis).
 *
 * ── Timing ───────────────────────────────────────────────────────────────────
 *
 *   TCA0: free-running /64 prescaler → 3.2 µs/tick (phase-angle timing)
 *   TCB0: 8-bit PWM, CLK_PER (20 MHz) → 78 kHz PWM for fan MOSFET
 *
 * ── Safety ───────────────────────────────────────────────────────────────────
 *
 *   Mains-connected hardware.  Use proper isolation, fusing, creepage
 *   clearances, and an enclosure.  Never touch the HV side while powered.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <stdint.h>
#include <stdbool.h>

/* ── Compile-time configuration ─────────────────────────────────────────── */

#define I2C_BASE_ADDR            0x40u
#define NUM_CHANNELS             6u
#define LEVEL_MAX                128u

#define TICKS_50HZ               3125u
#define TICKS_60HZ               2604u
#define GATE_TICKS               31u      /* 100 µs gate pulse @ 3.2 µs/tick */

#define ZC_WATCHDOG_OVERFLOWS    3u       /* ~630 ms before ZC declared lost  */

/* ── LED configuration ───────────────────────────────────────────────────── */

/* Number of TCA overflows (~210 ms each) without an I2C write before
   the green LED starts blinking to indicate RPi is not communicating.
   24 overflows × 210 ms ≈ 5 seconds.                                        */
#define I2C_IDLE_OVERFLOWS       24u

/* Blink period for "I2C idle" green LED: toggle every N overflows.
   2 overflows × 210 ms = 420 ms half-period → ~1.2 Hz blink.               */
#define LED_BLINK_HALF_PERIOD    2u

/* ── NTC ADC thresholds (10-bit ADC, 4-sample acc, top 8 bits returned)
 *
 *   10kΩ B3950 NTC to GND, 10kΩ fixed to VCC (5V).
 *   Higher byte = colder.  Lower byte = hotter.
 *
 *   Byte  142 ≈  20°C      Byte  50 ≈  60°C
 *   Byte   88 ≈  40°C      Byte  28 ≈  80°C
 *   Byte   67 ≈  50°C      Byte  25 ≈  85°C (overtemp trip)
 *   Byte   58 ≈  55°C      Byte  33 ≈  75°C (overtemp clear hysteresis)
 * ─────────────────────────────────────────────────────────────────────── */

/* Overtemp shutdown threshold and hysteresis */
#define OVERTEMP_THRESHOLD       25u     /* ≤ this byte → shutdown (≥85°C)  */
#define OVERTEMP_CLEAR_THRESHOLD 33u     /* ≥ this byte → allow clear (≤75°C)*/

/* Fan thermal curve */
#define FAN_ON_BYTE              50u     /* ≤ this → fan activates  (~60°C)  */
#define FAN_FULL_BYTE            28u     /* ≤ this → duty=255       (~80°C)  */
#define FAN_OFF_HYST_BYTE        55u     /* ≥ this → fan turns off  (~57°C)  */
#define FAN_MIN_DUTY             80u     /* minimum duty when fan is running  */

/* ── EEPROM layout ───────────────────────────────────────────────────────── */

#define EE_CH_BASE    0u   /* bytes 0–5: CH1..CH6 levels  */
#define EE_CONFIG     6u   /* byte  6:   CONFIG            */
#define EE_CH_MODE    7u   /* byte  7:   CH_MODE           */
#define EE_CHECKSUM   8u   /* byte  8:   XOR checksum      */
#define EE_MAGIC      0xA5u

/* ── Register addresses ──────────────────────────────────────────────────── */

#define REG_STATUS    0x00u
#define REG_CH1       0x01u
#define REG_CH2       0x02u
#define REG_CH3       0x03u
#define REG_CH4       0x04u
#define REG_CH5       0x05u
#define REG_CH6       0x06u
#define REG_CONFIG    0x07u
#define REG_BURST_CH  0x08u
#define REG_TEMP_RAW  0x09u
#define REG_ZC_COUNT  0x0Au
#define REG_FAN_DUTY  0x0Bu
#define REG_CH_MODE   0x0Cu   /* per-channel mode: bit n=0→phase, bit n=1→burst */
#define NUM_REGS      13u

static const uint8_t CH_REG[NUM_CHANNELS] = {
    REG_CH1, REG_CH2, REG_CH3, REG_CH4, REG_CH5, REG_CH6
};

#define IS_CH_REG(r)  ((r) >= REG_CH1 && (r) <= REG_CH6)

/* Bitmask for channel ch (0-based) in REG_CH_MODE */
#define CH_MODE_BIT(ch)  ((uint8_t)(1u << (ch)))

/* True if channel ch is in burst-fire mode */
#define CH_IS_BURST(ch)  ((reg[REG_CH_MODE] & CH_MODE_BIT(ch)) != 0u)

_Static_assert(REG_CH6 - REG_CH1 + 1u == NUM_CHANNELS,
               "Channel registers must be contiguous and match NUM_CHANNELS");
_Static_assert(NUM_CHANNELS <= 8u,
               "CH_MODE register uses one byte — max 8 channels");

/* ── CONFIG bit masks ────────────────────────────────────────────────────── */

#define CFG_FORCE_60HZ    0x01u
#define CFG_INVERT        0x02u
/* Bit2 freed — was CFG_BURST_MODE (now per-channel via REG_CH_MODE)        */
#define CFG_EE_SAVE       0x04u
#define CFG_EE_LOAD       0x08u
#define CFG_FAULT_CLR     0x10u
#define CFG_FAN_MANUAL    0x20u  /* RPi controls fan duty via REG_FAN_DUTY   */
#define CFG_FAN_FORCE_OFF 0x40u  /* force fan off (overridden by overtemp)   */

#define CFG_VOLATILE_MASK (CFG_EE_SAVE | CFG_EE_LOAD | CFG_FAULT_CLR)

/* ── STATUS bit masks ────────────────────────────────────────────────────── */

#define STA_ZC_LOCK   0x80u
#define STA_FAN_ON    0x04u  /* fan is currently spinning                    */
#define STA_OVERTEMP  0x02u
#define STA_60HZ      0x01u

/* ── LED pin definitions ─────────────────────────────────────────────────── */

#define LED_GREEN_PORT  PORTB
#define LED_GREEN_PIN   PIN4_bm   /* PB4 — green status LED                  */
#define LED_RED_PORT    PORTC
#define LED_RED_PIN     PIN3_bm   /* PC3 — red  status LED                   */

#define LED_GREEN_ON()   (LED_GREEN_PORT.OUTSET = LED_GREEN_PIN)
#define LED_GREEN_OFF()  (LED_GREEN_PORT.OUTCLR = LED_GREEN_PIN)
#define LED_GREEN_TGL()  (LED_GREEN_PORT.OUTTGL = LED_GREEN_PIN)
#define LED_RED_ON()     (LED_RED_PORT.OUTSET   = LED_RED_PIN)
#define LED_RED_OFF()    (LED_RED_PORT.OUTCLR   = LED_RED_PIN)

typedef struct { volatile uint8_t *port; uint8_t pin; } ChPin;

static const ChPin CH_PINS[NUM_CHANNELS] = {
    { &PORTB.OUT, PIN0_bm },
    { &PORTB.OUT, PIN1_bm },
    { &PORTB.OUT, PIN2_bm },
    { &PORTB.OUT, PIN3_bm },
    { &PORTC.OUT, PIN0_bm },
    { &PORTC.OUT, PIN1_bm },
    /* PC2 is TCB0 WO — managed by peripheral, not direct GPIO             */
};

#define GATE_HIGH(ch)  (*CH_PINS[ch].port |=  CH_PINS[ch].pin)
#define GATE_LOW(ch)   (*CH_PINS[ch].port &= ~CH_PINS[ch].pin)

static inline void all_gates_off(void)
{
    PORTB.OUTCLR = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;
    PORTC.OUTCLR = PIN0_bm | PIN1_bm;
    /* PC2 (fan PWM) is NOT touched here — fan keeps running during ZC loss */
}

/* ── Global state ────────────────────────────────────────────────────────── */

static volatile uint8_t  reg[NUM_REGS];
static volatile uint8_t  i2c_reg_ptr;
static volatile bool     i2c_first_byte;

/* Phase-angle firing */
static volatile uint16_t fire_tick[NUM_CHANNELS];
static volatile bool     fire_pending[NUM_CHANNELS];
static volatile bool     gate_on[NUM_CHANNELS];
static volatile uint16_t gate_off_tick[NUM_CHANNELS];

/* Burst-fire */
static volatile uint8_t  burst_accum[NUM_CHANNELS];

/* Timing */
static volatile uint16_t half_period;
static volatile uint16_t tca_snapshot;

/* ZC watchdog */
static volatile uint8_t  zc_watchdog;
static volatile bool     zc_lost;

/* Fault state */
static volatile bool     overtemp_latched;

/* Fan state */
static volatile bool     fan_running;   /* true when fan duty > 0           */

/* ── LED state ───────────────────────────────────────────────────────────── */

/* Rolling count of I2C write transactions from the TWI ISR.
   Compared against i2c_write_prev in TCA overflow to detect idle.          */
static volatile uint8_t  i2c_write_count;

/* Snapshot taken each TCA overflow for idle detection */
static          uint8_t  i2c_write_prev;

/* Counts TCA overflows since the last I2C write was detected */
static          uint8_t  i2c_idle_overflows;

/* True when no I2C writes received for I2C_IDLE_OVERFLOWS overflows */
static volatile bool     i2c_idle;

/* Blink counter — incremented each TCA overflow, masked to create 1 Hz */
static          uint8_t  blink_counter;

/* One-shot flag set after EEPROM save completes; causes brief double-flash */
static volatile bool     eeprom_save_flash;

/* ── Fan PWM via TCB0 ────────────────────────────────────────────────────── */

/*
 * Set fan duty cycle.
 * duty: 0 = off, 255 = full on.
 * TCB0 in 8-bit PWM (PWMSINGLE) mode: CCMPH holds the duty byte.
 * The peripheral updates the compare register at the next period boundary
 * so there is no glitch risk calling this from any context.
 */
static inline void fan_set_duty(uint8_t duty)
{
    TCB0.CCMPH = duty;          /* duty byte (upper byte of CCMP in PWM mode) */
    fan_running  = (duty > 0u);
    reg[REG_FAN_DUTY] = duty;

    /* Update FAN_ON status bit */
    if (fan_running) {
        reg[REG_STATUS] |=  STA_FAN_ON;
    } else {
        reg[REG_STATUS] &= ~STA_FAN_ON;
    }
}

/*
 * Compute and apply fan duty from the current ADC byte using the
 * thermal curve defined by FAN_ON_BYTE / FAN_FULL_BYTE / FAN_MIN_DUTY.
 *
 * Called from the TCA overflow context (~200 ms cadence) so it runs
 * well away from any time-critical ZC or gate logic.
 */
static void fan_update_auto(uint8_t adc_byte)
{
    uint8_t cfg = reg[REG_CONFIG];

    /* Overtemp always wins — blast the fan regardless of other settings    */
    if (overtemp_latched) {
        fan_set_duty(255u);
        return;
    }

    /* RPi manual override: duty is whatever was written to REG_FAN_DUTY    */
    if (cfg & CFG_FAN_MANUAL) {
        fan_set_duty(reg[REG_FAN_DUTY]);
        return;
    }

    /* Force-off bit: RPi can suppress the fan (but NOT during overtemp)    */
    if (cfg & CFG_FAN_FORCE_OFF) {
        fan_set_duty(0u);
        return;
    }

    /* ── Automatic thermal curve ── */

    if (adc_byte >= FAN_OFF_HYST_BYTE) {
        /* Cold enough to turn off (with hysteresis)                        */
        fan_set_duty(0u);
        return;
    }

    if (adc_byte > FAN_ON_BYTE) {
        /* Between hysteresis and on-threshold: keep current state          */
        /* (dead band prevents chatter right around 60°C)                   */
        if (!fan_running) {
            fan_set_duty(0u);
        }
        /* If already running, leave it running to avoid unnecessary cycling */
        return;
    }

    if (adc_byte <= FAN_FULL_BYTE) {
        /* At or above 80°C — maximum speed                                 */
        fan_set_duty(255u);
        return;
    }

    /*
     * Linear ramp between FAN_ON_BYTE (60°C) and FAN_FULL_BYTE (80°C).
     *
     * adc_byte decreases as temperature increases, so:
     *   fraction = (FAN_ON_BYTE - adc_byte) / (FAN_ON_BYTE - FAN_FULL_BYTE)
     *   duty     = FAN_MIN_DUTY + fraction * (255 - FAN_MIN_DUTY)
     *
     * Using only 8-bit integer arithmetic with careful ordering to avoid
     * overflow: the numerator fits in 8 bits (≤22), denominator is 22,
     * so we scale up by 255 before dividing.
     */
    uint8_t  range    = (uint8_t)(FAN_ON_BYTE - FAN_FULL_BYTE);     /* 22   */
    uint8_t  delta    = (uint8_t)(FAN_ON_BYTE - adc_byte);          /* 0–22 */
    uint16_t span     = (uint16_t)(255u - FAN_MIN_DUTY);            /* 175  */
    uint8_t  ramp     = (uint8_t)((uint16_t)(delta * span) / range);
    uint8_t  duty     = (uint8_t)(FAN_MIN_DUTY + ramp);

    fan_set_duty(duty);
}

/* ── EEPROM ──────────────────────────────────────────────────────────────── */

static uint8_t eeprom_checksum(void)
{
    uint8_t csum = EE_MAGIC;
    for (uint8_t i = EE_CH_BASE; i < EE_CHECKSUM; i++)
        csum ^= eeprom_read_byte((const uint8_t *)(uintptr_t)i);
    return csum;
}

static void eeprom_save(void)
{
    for (uint8_t ch = 0u; ch < NUM_CHANNELS; ch++)
        eeprom_update_byte((uint8_t *)(uintptr_t)(EE_CH_BASE + ch),
                           reg[CH_REG[ch]]);
    eeprom_update_byte((uint8_t *)(uintptr_t)EE_CONFIG,
                       (uint8_t)(reg[REG_CONFIG] & ~CFG_VOLATILE_MASK));
    eeprom_update_byte((uint8_t *)(uintptr_t)EE_CH_MODE,
                       reg[REG_CH_MODE]);
    eeprom_update_byte((uint8_t *)(uintptr_t)EE_CHECKSUM,
                       eeprom_checksum());
}

static bool eeprom_load(void)
{
    if (eeprom_read_byte((const uint8_t *)(uintptr_t)EE_CHECKSUM)
            != eeprom_checksum())
        return false;

    for (uint8_t ch = 0u; ch < NUM_CHANNELS; ch++) {
        uint8_t lvl = eeprom_read_byte(
                          (const uint8_t *)(uintptr_t)(EE_CH_BASE + ch));
        if (lvl > LEVEL_MAX) lvl = LEVEL_MAX;
        reg[CH_REG[ch]] = lvl;
    }
    reg[REG_CONFIG] = (uint8_t)(
        eeprom_read_byte((const uint8_t *)(uintptr_t)EE_CONFIG)
        & ~CFG_VOLATILE_MASK);
    reg[REG_CH_MODE] = eeprom_read_byte(
                           (const uint8_t *)(uintptr_t)EE_CH_MODE);
    return true;
}

/* ── ADC (NTC) ───────────────────────────────────────────────────────────── */

static void adc_init(void)
{
    /* PA6 (AIN6), VDD reference, 4-sample accumulation, 625 kHz ADC clock */
    ADC0.CTRLA  = ADC_ENABLE_bm;
    ADC0.CTRLB  = ADC_SAMPNUM_ACC4_gc;
    ADC0.CTRLC  = ADC_PRESC_DIV32_gc | ADC_REFSEL_VDDREF_gc;
    ADC0.MUXPOS = ADC_MUXPOS_AIN6_gc;
}

static uint8_t adc_sample_byte(void)
{
    ADC0.COMMAND = ADC_STCONV_bm;
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    /* 4 × 10-bit → 12-bit result; return upper 8 bits */
    return (uint8_t)(ADC0.RES >> 4);
}

/*
 * Sample temperature, update REG_TEMP_RAW, check overtemp latch,
 * and update fan speed.  Called once per TCA overflow (~200 ms).
 */
static void temperature_tick(void)
{
    uint8_t t = adc_sample_byte();
    reg[REG_TEMP_RAW] = t;

    /* Check overtemp (remember: lower byte = hotter) */
    if (!overtemp_latched && t <= OVERTEMP_THRESHOLD) {
        overtemp_latched  = true;
        reg[REG_STATUS]  |= STA_OVERTEMP;
        all_gates_off();
        for (uint8_t ch = 0u; ch < NUM_CHANNELS; ch++) {
            fire_pending[ch] = false;
            gate_on[ch]      = false;
        }
    }

    /* Update fan — this always runs, even when overtemp is latched         */
    fan_update_auto(t);
}

/* ── Initialisation ──────────────────────────────────────────────────────── */

/*
 * Update both LEDs to reflect current system state.
 *
 * Called from the TCA overflow ISR (~210 ms cadence) — never from the
 * time-critical ZC ISR.  All decisions are based on stable global flags
 * that are only written from the TCA overflow or ZC ISR contexts.
 *
 * Priority (highest → lowest):
 *   1. EEPROM save flash — brief both-on, one overflow only
 *   2. Fault (overtemp or ZC lost) — RED solid, GREEN off
 *   3. Healthy + I2C idle — GREEN blink, RED off
 *   4. Healthy + I2C active — GREEN solid, RED off
 */
static void led_update(void)
{
    /* ── EEPROM save confirmation: both LEDs on for one overflow (~210 ms) ── */
    if (eeprom_save_flash) {
        eeprom_save_flash = false;
        LED_GREEN_ON();
        LED_RED_ON();
        return;
    }

    /* ── Fault state: RED solid, GREEN off ── */
    if (overtemp_latched || zc_lost) {
        LED_GREEN_OFF();
        LED_RED_ON();
        return;
    }

    /* ── Healthy: GREEN controls, RED off ── */
    LED_RED_OFF();

    if (i2c_idle) {
        /* Blink green at ~1 Hz to warn RPi is not communicating */
        blink_counter++;
        if (blink_counter >= LED_BLINK_HALF_PERIOD) {
            blink_counter = 0u;
            LED_GREEN_TGL();
        }
    } else {
        /* Solid green: ZC locked and I2C active */
        LED_GREEN_ON();
        blink_counter = 0u;  /* reset so blink starts cleanly if idle begins */
    }
}

static void clock_init(void)
{
    CPU_CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSC20M_gc;
    CPU_CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = 0;               /* no prescaler → 20 MHz           */
    while (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm);
}

static void gpio_init(void)
{
    /* Gate outputs — all low initially */
    PORTB.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;
    PORTB.OUTCLR = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;
    PORTC.DIRSET = PIN0_bm | PIN1_bm;
    PORTC.OUTCLR = PIN0_bm | PIN1_bm;

    /* PC2: TCB0 WO (fan PWM) — direction set by peripheral after tcb_init  */

    /* Status LEDs: PB4 = GREEN, PC3 = RED
       RED on immediately at boot; GREEN off until healthy state confirmed.  */
    LED_GREEN_PORT.DIRSET = LED_GREEN_PIN;
    LED_RED_PORT.DIRSET   = LED_RED_PIN;
    LED_GREEN_OFF();
    LED_RED_ON();   /* boot state: RED solid until first healthy LED update  */

    /* ZC input PA3: pull-up, falling-edge interrupt */
    PORTA.DIRCLR   = PIN3_bm;
    PORTA.PIN3CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;

    /* Address select PA4/PA5: inputs with pull-ups */
    PORTA.DIRCLR   = PIN4_bm | PIN5_bm;
    PORTA.PIN4CTRL = PORT_PULLUPEN_bm;
    PORTA.PIN5CTRL = PORT_PULLUPEN_bm;

    /* NTC input PA6: plain input, no pull-up */
    PORTA.DIRCLR   = PIN6_bm;
    PORTA.PIN6CTRL = 0u;

    /* TWI PA1/PA2 */
    PORTA.PIN1CTRL = PORT_PULLUPEN_bm;
    PORTA.PIN2CTRL = PORT_PULLUPEN_bm;
}

static uint8_t read_i2c_address(void)
{
    uint8_t pa   = PORTA.IN;
    uint8_t bits = (uint8_t)(
        ((~pa & PIN4_bm) ? 1u : 0u) |
        ((~pa & PIN5_bm) ? 2u : 0u));
    return (uint8_t)(I2C_BASE_ADDR + bits);
}

static void tca_init(void)
{
    /* TCA0 free-running, /64 prescaler → 3.2 µs/tick, overflow every ~210ms */
    TCA0.SINGLE.CTRLA   = TCA_SINGLE_CLKSEL_DIV64_gc;
    TCA0.SINGLE.CTRLB   = TCA_SINGLE_WGMODE_NORMAL_gc;
    TCA0.SINGLE.PER     = 0xFFFFu;
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.CTRLA  |= TCA_SINGLE_ENABLE_bm;
}

static void tcb_fan_init(void)
{
    /*
     * TCB0 in 8-bit PWM (PWMSINGLE) mode.
     *
     * CCMPL (low byte) sets the period — fixed at 254 for 8-bit operation.
     * CCMPH (high byte) sets the duty — written dynamically by fan_set_duty().
     *
     * Clock source: CLK_PER (20 MHz, no prescaler)
     * PWM frequency: 20,000,000 / 256 = 78,125 Hz  (well above audible range)
     *
     * PC2 is the TCB0 WO (waveform output) pin on ATtiny1616.
     */
    TCB0.CTRLB  = TCB_CNTMODE_PWM8_gc | TCB_CCMPEN_bm;
    TCB0.CCMPL  = 254u;      /* period byte: gives 255 steps of duty         */
    TCB0.CCMPH  = 0u;        /* duty byte: start at 0 (fan off)              */
    TCB0.CTRLA  = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;

    /* Route TCB0 WO to PC2 */
    PORTC.DIRSET = PIN2_bm;
}

static void twi_slave_init(uint8_t addr)
{
    TWI0.SADDR  = (uint8_t)(addr << 1);
    TWI0.SCTRLA = TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm | TWI_ENABLE_bm;
    TWI0.SCTRLB = 0u;
}

/* ── Register / firing helpers ───────────────────────────────────────────── */

static void update_status(void)
{
    uint8_t s = (uint8_t)(reg[REG_STATUS] & (STA_OVERTEMP | STA_FAN_ON));
    if (!zc_lost && half_period > 0u) s |= STA_ZC_LOCK;
    if (half_period > 0u && half_period < 2900u) s |= STA_60HZ;
    reg[REG_STATUS] = s;
}

static void recompute_fire_ticks(void)
{
    uint16_t hp  = half_period;
    bool     inv = (reg[REG_CONFIG] & CFG_INVERT) != 0u;

    for (uint8_t ch = 0u; ch < NUM_CHANNELS; ch++) {
        uint8_t raw = reg[CH_REG[ch]];
        if (raw == 0u) {
            fire_tick[ch] = 0xFFFFu;
        } else if (raw >= LEVEL_MAX) {
            fire_tick[ch] = 0u;
        } else {
            uint8_t lvl   = inv ? (uint8_t)(LEVEL_MAX - raw) : raw;
            fire_tick[ch] = (uint16_t)(
                ((uint32_t)(LEVEL_MAX - lvl) * hp) / LEVEL_MAX);
        }
    }
}

static bool burst_should_fire(uint8_t ch)
{
    uint8_t lvl = reg[CH_REG[ch]];
    if (lvl == 0u)        return false;
    if (lvl >= LEVEL_MAX) return true;
    if (reg[REG_CONFIG] & CFG_INVERT) lvl = (uint8_t)(LEVEL_MAX - lvl);
    burst_accum[ch] = (uint8_t)(burst_accum[ch] + lvl);
    if (burst_accum[ch] >= LEVEL_MAX) {
        burst_accum[ch] = (uint8_t)(burst_accum[ch] - LEVEL_MAX);
        return true;
    }
    return false;
}

/* ── Zero-crossing ISR ───────────────────────────────────────────────────── */

ISR(PORTA_PORT_vect)
{
    PORTA.INTFLAGS = PIN3_bm;

    uint16_t now   = TCA0.SINGLE.CNT;
    uint16_t delta = (uint16_t)(now - tca_snapshot);
    tca_snapshot   = now;

    if (reg[REG_ZC_COUNT] > 2u) {
        half_period = (uint16_t)(
            half_period - (half_period >> 3) + (delta >> 3));
    } else {
        half_period = (reg[REG_CONFIG] & CFG_FORCE_60HZ)
                    ? TICKS_60HZ : TICKS_50HZ;
    }

    reg[REG_ZC_COUNT]++;
    zc_watchdog = 0u;
    zc_lost     = false;
    update_status();

    if (overtemp_latched) return;

    /*
     * Per-channel mode dispatch.
     *
     * On each zero crossing, recompute phase-angle fire ticks once (they
     * depend on half_period which just updated) then arm each channel
     * according to its individual CH_MODE bit:
     *
     *   Burst-fire channel: decide NOW whether this half-cycle fires.
     *     burst_should_fire() advances the Bresenham accumulator and
     *     returns true → fire gate immediately; false → keep gate low.
     *
     *   Phase-angle channel: set fire_pending so poll_gates() fires the
     *     gate at the calculated delay after this ZC edge.
     */
    recompute_fire_ticks();

    for (uint8_t ch = 0u; ch < NUM_CHANNELS; ch++) {
        uint8_t lvl = reg[CH_REG[ch]];

        if (lvl == 0u) {
            /* Level 0 always means off in both modes */
            GATE_LOW(ch);
            fire_pending[ch] = false;
            gate_on[ch]      = false;
            continue;
        }

        if (CH_IS_BURST(ch)) {
            /* ── Burst-fire mode for this channel ── */
            if (burst_should_fire(ch)) {
                GATE_HIGH(ch);
                gate_on[ch]       = true;
                gate_off_tick[ch] = (uint16_t)(now + GATE_TICKS);
                fire_pending[ch]  = false;
            } else {
                GATE_LOW(ch);
                gate_on[ch]      = false;
                fire_pending[ch] = false;
            }
        } else {
            /* ── Phase-angle mode for this channel ── */
            if (lvl >= LEVEL_MAX) {
                /* Full-on: fire immediately at ZC */
                GATE_HIGH(ch);
                fire_pending[ch] = false;
                gate_on[ch]      = false;
                gate_off_tick[ch]= (uint16_t)(now + GATE_TICKS);
            } else {
                /* Delayed fire: poll_gates() handles the actual firing */
                fire_pending[ch] = true;
                gate_on[ch]      = false;
            }
        }
    }
}

/* ── TCA overflow ISR (~210 ms) ──────────────────────────────────────────── */

ISR(TCA0_OVF_vect)
{
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;

    /* ── ZC watchdog ── */
    if (!zc_lost) {
        zc_watchdog++;
        if (zc_watchdog >= ZC_WATCHDOG_OVERFLOWS) {
            zc_lost     = true;
            half_period = 0u;
            all_gates_off();
            for (uint8_t ch = 0u; ch < NUM_CHANNELS; ch++) {
                fire_pending[ch] = false;
                gate_on[ch]      = false;
            }
            update_status();
            /* Fan keeps running — ZC loss doesn't affect thermal need      */
        }
    }

    /* ── Temperature sample + fan update ── */
    temperature_tick();

    /* ── Deferred CONFIG actions ── */
    uint8_t cfg = reg[REG_CONFIG];

    if (cfg & CFG_EE_SAVE) {
        eeprom_save();
        reg[REG_CONFIG]  &= ~CFG_EE_SAVE;
        eeprom_save_flash = true;   /* triggers brief both-LEDs-on next cycle */
    }
    if (cfg & CFG_EE_LOAD) {
        eeprom_load();
        recompute_fire_ticks();
        reg[REG_CONFIG] &= ~CFG_EE_LOAD;
    }
    if (cfg & CFG_FAULT_CLR) {
        if (overtemp_latched) {
            uint8_t t = adc_sample_byte();
            /* Lower threshold = hotter, so we need byte >= clear threshold */
            if (t >= OVERTEMP_CLEAR_THRESHOLD) {
                overtemp_latched  = false;
                reg[REG_STATUS]  &= ~STA_OVERTEMP;
            }
        }
        reg[REG_CONFIG] &= ~CFG_FAULT_CLR;
    }

    /* ── I2C idle detection ── */
    if (i2c_write_count != i2c_write_prev) {
        /* New write activity since last overflow */
        i2c_write_prev      = i2c_write_count;
        i2c_idle_overflows  = 0u;
        i2c_idle            = false;
    } else {
        if (i2c_idle_overflows < I2C_IDLE_OVERFLOWS) {
            i2c_idle_overflows++;
            if (i2c_idle_overflows >= I2C_IDLE_OVERFLOWS) {
                i2c_idle = true;
            }
        }
    }

    /* ── LED update ── */
    led_update();
}

/* ── Main-loop gate-pulse management ─────────────────────────────────────── */

static void poll_gates(void)
{
    if (overtemp_latched || zc_lost) return;

    uint16_t now = TCA0.SINGLE.CNT;
    uint16_t zc  = tca_snapshot;

    for (uint8_t ch = 0u; ch < NUM_CHANNELS; ch++) {
        if (gate_on[ch]) {
            if ((uint16_t)(now - gate_off_tick[ch]) < 0x8000u) {
                GATE_LOW(ch);
                gate_on[ch] = false;
            }
        } else if (fire_pending[ch]) {
            if ((uint16_t)(now - zc) >= fire_tick[ch]) {
                GATE_HIGH(ch);
                gate_on[ch]       = true;
                gate_off_tick[ch] = (uint16_t)(now + GATE_TICKS);
                fire_pending[ch]  = false;
            }
        }
    }
}

/* ── TWI (I2C) Slave ISR ─────────────────────────────────────────────────── */

ISR(TWI0_TWIS_vect)
{
    uint8_t status = TWI0.SSTATUS;

    if (status & TWI_APIF_bm) {
        if (status & TWI_AP_bm) {
            i2c_first_byte = true;
            TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
        } else {
            TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
        }
        return;
    }

    if (status & TWI_DIF_bm) {
        if (status & TWI_DIR_bm) {
            /* Master read */
            uint8_t r   = (i2c_reg_ptr < NUM_REGS) ? reg[i2c_reg_ptr] : 0xFFu;
            TWI0.SDATA  = r;
            i2c_reg_ptr = (uint8_t)((i2c_reg_ptr + 1u) % NUM_REGS);
            TWI0.SCTRLB = (TWI0.SSTATUS & TWI_RXACK_bm)
                        ? TWI_SCMD_COMPTRANS_gc
                        : TWI_SCMD_RESPONSE_gc;
        } else {
            /* Master write */
            uint8_t data = TWI0.SDATA;
            if (i2c_first_byte) {
                i2c_reg_ptr    = (uint8_t)(data % NUM_REGS);
                i2c_first_byte = false;
            } else {
                /* Count every data byte written — used by LED idle detector */
                i2c_write_count++;
                switch (i2c_reg_ptr) {

                case REG_STATUS:
                case REG_TEMP_RAW:
                case REG_ZC_COUNT:
                    /* Read-only */
                    break;

                case REG_BURST_CH:
                    i2c_reg_ptr = REG_CH1;
                    TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
                    return;

                case REG_CONFIG: {
                    reg[REG_CONFIG] = data;
                    /* No burst-mode accumulator reset here — mode is now
                       per-channel and managed by REG_CH_MODE writes.       */
                    break;
                }

                case REG_CH_MODE: {
                    /*
                     * Reset burst accumulators for every channel that is
                     * transitioning from phase-angle → burst-fire so its
                     * first burst decision starts from a clean state.
                     * Channels already in burst mode, or transitioning out
                     * of it, do not need a reset.
                     */
                    uint8_t old_mode = reg[REG_CH_MODE];
                    uint8_t new_mode = data & 0x3Fu; /* mask unused bits 6-7 */
                    uint8_t newly_burst = (uint8_t)(new_mode & ~old_mode);
                    for (uint8_t ch = 0u; ch < NUM_CHANNELS; ch++) {
                        if (newly_burst & CH_MODE_BIT(ch))
                            burst_accum[ch] = 0u;
                    }
                    reg[REG_CH_MODE] = new_mode;
                    break;
                }

                case REG_FAN_DUTY:
                    /*
                     * Only takes effect if CFG_FAN_MANUAL is set.
                     * Store the value; fan_update_auto() picks it up on
                     * the next TCA overflow (~210 ms).  For faster response
                     * the RPi can write CONFIG and FAN_DUTY in the same
                     * I2C transaction.
                     */
                    reg[REG_FAN_DUTY] = data;
                    break;

                default:
                    if (IS_CH_REG(i2c_reg_ptr)) {
                        uint8_t ch  = (uint8_t)(i2c_reg_ptr - REG_CH1);
                        uint8_t lvl = (data > LEVEL_MAX)
                                    ? (uint8_t)LEVEL_MAX : data;
                        reg[i2c_reg_ptr] = lvl;
                        /* Reset burst accumulator regardless of current mode
                           so a mode switch immediately after a level write
                           always starts clean.                              */
                        burst_accum[ch] = 0u;
                        /* Recompute phase-angle tick for this channel if it
                           is currently in phase-angle mode.                 */
                        if (!CH_IS_BURST(ch))
                            recompute_fire_ticks();
                    }
                    break;
                }
                i2c_reg_ptr = (uint8_t)((i2c_reg_ptr + 1u) % NUM_REGS);
            }
            TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
        }
    }

    if (status & TWI_PIEF_bm) {
        TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
    }
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    clock_init();
    gpio_init();
    adc_init();
    tcb_fan_init();

    uint8_t i2c_addr = read_i2c_address();

    tca_init();
    twi_slave_init(i2c_addr);

    /* Initialise register file and state */
    for (uint8_t i = 0u; i < NUM_REGS; i++) reg[i] = 0u;
    half_period        = TICKS_50HZ;
    i2c_first_byte     = true;
    zc_lost            = false;
    zc_watchdog        = 0u;
    overtemp_latched   = false;
    fan_running        = false;
    i2c_write_count    = 0u;
    i2c_write_prev     = 0u;
    i2c_idle_overflows = 0u;
    i2c_idle           = false;
    blink_counter      = 0u;
    eeprom_save_flash  = false;

    for (uint8_t ch = 0u; ch < NUM_CHANNELS; ch++) {
        burst_accum[ch]  = 0u;
        fire_pending[ch] = false;
        gate_on[ch]      = false;
    }

    eeprom_load();
    recompute_fire_ticks();

    /*
     * Take one temperature sample before enabling interrupts so the fan
     * starts in the correct state immediately rather than waiting up to
     * 210 ms for the first TCA overflow.
     */
    {
        uint8_t t = adc_sample_byte();
        reg[REG_TEMP_RAW] = t;
        fan_update_auto(t);
    }

    sei();
    set_sleep_mode(SLEEP_MODE_IDLE);

    while (1) {
        poll_gates();
        sleep_enable();
        sleep_cpu();
        sleep_disable();
    }
}
