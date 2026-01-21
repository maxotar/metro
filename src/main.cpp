#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/cpufunc.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "i2c.h"

/* ============================================================================
   ATtiny1616 Pin Usage (20-pin VQFN)
   ============================================================================
   Pin    | Function    | Usage          | Direction | Notes
   -------+-------------+----------------+-----------+-------------------------
   VDD  4 | Power       | Power supply   | -         | Connect to 3.3V/5V
   GND  3 | Ground      | Ground         | -         | Common ground
   PA0 19 | UPDI        | Programming    | -         | Reserved for UPDI
   PA1 20 | -           | FREE           | -         | Available
   PA2  1 | -           | FREE           | -         | Available
   PA3  2 | -           | FREE           | -         | Available
   PA4  5 | LED_STATUS  | LED STATUS     | -         | Active low
   PA5  6 | LED_LSB     | LED bit 0      | Output    | Active low
   PA6  7 | LED_BIT1    | LED bit 1      | Output    | Active low
   PA7  8 | LED_BIT2    | LED bit 2      | Output    | Active low
   PB0 14 | TWI0 SCL    | I2C Clock      | I2C       | 4.7kΩ pullup
   PB1 13 | TWI0 SDA    | I2C Data       | I2C       | 4.7kΩ pullup
   PB2 12 | XTAL1       | FREE           | -         | Future: external crystal
   PB3 11 | XTAL2       | FREE           | -         | Future: external crystal
   PB4 10 | LED_MSB     | LED bit 4 (MSB)| Output    | Active low
   PB5  9 | LED_BIT3    | LED bit 3      | Output    | Active low
   PC0 15 | -           | FREE           | -         | Available
   PC1 16 | BUTTON1     | Decrease BPM   | Input     | Internal Pullup, BOTH EDGES
   PC2 17 | BUTTON2     | Play/Pause     | Input     | Internal Pullup, BOTH EDGES
   PC3 18 | BUTTON3     | Increase BPM   | Input     | Internal Pullup, BOTH EDGES

   Used: 12 pins (3 buttons + 6 LEDs + 2 I2C + 1 UPDI)
   Free: 6 pins (PA1-PA3, PB2-PB3, PC0)
   ============================================================================ */

typedef enum
{
  BTN_IDLE,
  BTN_PRESSED
} ButtonState;

typedef struct
{
  uint8_t pinMask;
  volatile uint8_t *pinCtrl;
  uint8_t taskIndex;
  void (*onPress)(void);
  volatile ButtonState state;
} Button;

typedef struct
{
  uint16_t timeDue;
  uint8_t flags;
  void (*callback)(void *ctx);
  void *context;
} Task;

static void task_play(void *ctx);
static void task_pause(void *ctx);
static void task_haptic_standby(void *ctx);
static void task_hide_BPM(void *ctx);
static void task_hide_status(void *ctx);
static void task_debounce(void *ctx);
static void action_dec(void);
static void action_inc(void);
static void action_play(void);

enum TaskIndex
{
  TASK_BTN_DEC_POLL,
  TASK_BTN_PLAY_POLL,
  TASK_BTN_INC_POLL,
  TASK_PLAY,
  TASK_HAPTIC_STANDBY,
  TASK_HIDE_BPM,
  TASK_HIDE_STATUS,
  TASK_PAUSE,
  NUM_TASKS
};

static Button buttons[] = {
    {PIN1_bm, &PORTC.PIN1CTRL, TASK_BTN_DEC_POLL, action_dec, BTN_IDLE},
    {PIN2_bm, &PORTC.PIN2CTRL, TASK_BTN_PLAY_POLL, action_play, BTN_IDLE},
    {PIN3_bm, &PORTC.PIN3CTRL, TASK_BTN_INC_POLL, action_inc, BTN_IDLE}};

static Task tasks[NUM_TASKS] = {
    {0, 0, task_debounce, &buttons[0]}, // TASK_BTN_DEC_POLL
    {0, 0, task_debounce, &buttons[1]}, // TASK_BTN_PLAY_POLL
    {0, 0, task_debounce, &buttons[2]}, // TASK_BTN_INC_POLL
    {0, 0, task_play, NULL},            // TASK_PLAY
    {0, 0, task_haptic_standby, NULL},  // TASK_HAPTIC_STANDBY
    {0, 0, task_hide_BPM, NULL},        // TASK_HIDE_BPM
    {0, 0, task_hide_status, NULL},     // TASK_HIDE_STATUS
    {0, 0, task_pause, NULL}            // TASK_PAUSE
};

// Macros
#define NUM_BUTTONS (sizeof(buttons) / sizeof(Button))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define TASK_PENDING (1 << 0)

// Status LED
#define LED_STATUS_PIN PIN4_bm // PA4 - Status LED (active low)

// BPM Display LEDs (5 bits, scaled by 5)
#define LED_MSB_PIN PIN4_bm  // PB4 - MSB (bit 4)
#define LED_BIT3_PIN PIN5_bm // PB5 - bit 3
#define LED_BIT2_PIN PIN7_bm // PA7 - bit 2
#define LED_BIT1_PIN PIN6_bm // PA6 - bit 1
#define LED_LSB_PIN PIN5_bm  // PA5 - LSB (bit 0)

// Timing constants (in RTC ticks at 1024Hz, 1 tick = 1ms)
#define DEBOUNCE_DELAY 30
#define LED_ON_DURATION 500
#define HAPTIC_DURATION 100
#define BPM_INCREMENT 5
#define BPM_MIN 5
#define BPM_MAX 155

// DRV2605L Register Addresses
#define DRV2605L_ADDR 0x5A
#define DRV2605L_REG_STATUS 0x00
#define DRV2605L_REG_MODE 0x01
#define DRV2605L_REG_RTPIN 0x02
#define DRV2605L_REG_LIBRARY 0x03
#define DRV2605L_REG_WAVESEQ1 0x04
#define DRV2605L_REG_WAVESEQ2 0x05
#define DRV2605L_REG_WAVESEQ3 0x06
#define DRV2605L_REG_WAVESEQ4 0x07
#define DRV2605L_REG_WAVESEQ5 0x08
#define DRV2605L_REG_WAVESEQ6 0x09
#define DRV2605L_REG_WAVESEQ7 0x0A
#define DRV2605L_REG_WAVESEQ8 0x0B
#define DRV2605L_REG_GO 0x0C
#define DRV2605L_REG_OVERDRIVE 0x0D
#define DRV2605L_REG_SUSTAINPOS 0x0E
#define DRV2605L_REG_SUSTAINNEG 0x0F
#define DRV2605L_REG_BREAK 0x10
#define DRV2605L_REG_AUDIOCTRL 0x11
#define DRV2605L_REG_CONTROL1 0x1B
#define DRV2605L_REG_CONTROL2 0x1C
#define DRV2605L_REG_CONTROL3 0x1D
#define DRV2605L_REG_CONTROL4 0x1E
#define DRV2605L_REG_CONTROL5 0x1F
#define DRV2605L_REG_FEEDBACK 0x1A
#define DRV2605L_REG_RATED_VOLT 0x16
#define DRV2605L_REG_CLAMP_VOLT 0x17
#define DRV2605L_REG_CALIB_COMP 0x18
#define DRV2605L_REG_CALIB_BEMF 0x19

uint8_t bpm = 100;

static void init_clock(void)
{
  // Prescaler DIV4 for 4MHz (16MHz / 4)
  // PDIV = 1 (divide by 2^(PDIV+1) = 2^2 = 4), PEN = 1 (enable)
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, (1 << CLKCTRL_PDIV_gp) | CLKCTRL_PEN_bm);
}

static bool try_init_haptic_driver(void)
{
  // -----------------------------------------------------------------------------
  // DRV2605L + VLV101040A LRA initialization
  //
  // Motor: VLV101040A
  //  - Type: LRA, Z-axis
  //  - Size: 10 × 10 × 4 mm
  //  - Rated voltage: 2.5 Vrms
  //  - Resonant frequency: ~170 Hz
  //  - Usable bandwidth: ~140–300 Hz
  //  - Fast response: ~10 ms rise, ~40 ms fall (unbraked)
  //
  // Usage:
  //  - Wearable haptic metronome
  //  - Very short, strong "tick" pulses (~12 ms)
  //  - Fixed frequency, open-loop, RTP drive
  //  - Aggressive braking to suppress ring-down
  // -----------------------------------------------------------------------------

  // --- Exit standby / reset internal state ---
  // Required before writing most configuration registers
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x00);

  // --- Select Real-Time Playback (RTP) ---
  // WaveSeq = 0x7F routes drive amplitude from RTPIN register
  // This bypasses waveform libraries and gives direct control
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_WAVESEQ1, 0x7F);
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_WAVESEQ2, 0x00);

  // --- Set LRA resonant frequency (~170 Hz) ---
  // CONTROL1 = Drive Time
  // DriveTime = (1 / (2 × f)) × 10000 − 1
  // For 170 Hz → ~28–29 → 0x1C
  //
  // This sets the waveform frequency for *all* RTP pulses
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_CONTROL1, 0x1C);

  // --- Clamp voltage (attack strength) ---
  // Raised above rated voltage for very short pulses
  // ~3.6 V peak equivalent
  //
  // Safe because:
  //  - Pulse width is only ~12 ms
  //  - Duty cycle is extremely low (metronome use)
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_CLAMP_VOLT, 0xA5);

  // --- Feedback / braking configuration ---
  // Bit 7 = 1 → LRA mode
  // Brake factor = 8× → fast stop, minimal ring-down
  // Loop gain = medium → stable, predictable response
  //
  // This is critical for making the vibration feel like a "tap"
  // instead of a buzz when mounted against skin.
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_FEEDBACK, 0xBC);

  // --- Disable auto-resonance & force open-loop ---
  // Required because this LRA has unusually wide bandwidth
  // Auto-resonance causes frequency hunting and inconsistent pulses
  //
  // Bit 5 = 1 → disable auto-resonance
  // Bit 1 = 1 → bidirectional drive
  // Bit 0 = 1 → open-loop
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_CONTROL3, 0x23);

  // --- Optional: low-power idle ---
  // Device will be woken and put into RTP mode per pulse
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x40);

  return true; // We assume success; power-cycle recovery is acceptable
}

static void init_haptic_driver(void)
{
  // Allow DRV2605L internal POR to settle
  _delay_us(250);

  while (1)
  {
    i2c_bus_recovery();
    init_i2c();

    if (try_init_haptic_driver())
    {
      i2c_shutdown();
      break;
    }

    i2c_shutdown();
    _delay_ms(10);
  }
}

static inline void rtc_wait_ready(void)
{
  while (RTC.STATUS > 0)
    ;
}

static void init_rtc(void)
{
  // Wait for any previous power-on synchronization to finish
  rtc_wait_ready();

  // Disable RTC before changing configuration
  RTC.CTRLA = 0;
  rtc_wait_ready(); // MUST wait after disabling before changing CLKSEL

  // Select internal 1kHz clock source
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;
  rtc_wait_ready(); // MUST wait after changing clock source

  // Clear any pending interrupt flags
  RTC.INTFLAGS = RTC_OVF_bm | RTC_CMP_bm;

  // Set initial compare value for first metronome tick
  RTC.CMP = 100;
  rtc_wait_ready(); // MUST wait after setting compare value

  // Enable RTC with DIV1 prescaler, run in standby mode
  RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RTCEN_bm | RTC_RUNSTDBY_bm;
  rtc_wait_ready(); // MUST wait after enabling RTC

  // Enable compare match interrupt
  RTC.INTCTRL = RTC_CMP_bm;

  // Final wait to ensure RTC is fully operational before returning
  rtc_wait_ready();
}

static void init_unused_pins_low_power(void)
{
  PORTA.OUTCLR = PIN1_bm | PIN2_bm | PIN3_bm;
  PORTA.DIRSET = PIN1_bm | PIN2_bm | PIN3_bm;
  PORTA.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;

  PORTB.OUTCLR = PIN2_bm | PIN3_bm;
  PORTB.DIRSET = PIN2_bm | PIN3_bm;
  PORTB.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;

  PORTC.OUTCLR = PIN0_bm;
  PORTC.DIRSET = PIN0_bm;
  PORTC.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
}

static void init_buttons(void)
{
  uint8_t allButtonPins = 0;

  for (uint8_t i = 0; i < NUM_BUTTONS; i++)
  {
    allButtonPins |= buttons[i].pinMask;
    *(buttons[i].pinCtrl) = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
  }

  PORTC.DIRCLR = allButtonPins;
}

static void init_leds(void)
{
  // Configure LED pins as outputs on PORTA (PA5, PA6, PA7)
  PORTA.DIRSET = LED_STATUS_PIN | LED_LSB_PIN | LED_BIT1_PIN | LED_BIT2_PIN;
  PORTA.OUTSET = LED_STATUS_PIN | LED_LSB_PIN | LED_BIT1_PIN | LED_BIT2_PIN; // Start HIGH (LEDs off - active low)

  // Configure LED pins as outputs on PORTB (PB4, PB5)
  PORTB.DIRSET = LED_MSB_PIN | LED_BIT3_PIN;
  PORTB.OUTSET = LED_MSB_PIN | LED_BIT3_PIN; // Start HIGH (LEDs off - active low)
}

static void haptic_trigger(void)
{
  // -----------------------------------------------------------------------------
  // Generate one haptic "tick" using RTP drive
  //
  // Dynamics summary:
  //  - The LRA is a resonant mechanical system (~170 Hz)
  //  - Maximum perceived sharpness comes from:
  //      1) Hitting resonance hard (high initial acceleration)
  //      2) Letting it ring for ~2 cycles
  //      3) Actively braking to suppress tail vibration
  //
  // For a 170 Hz LRA:
  //  - Period ≈ 5.9 ms
  //  - 12 ms ≈ 2 cycles → strong but non-buzzy impulse
  //
  // This function assumes:
  //  - Frequency, clamp voltage, brake strength are already configured at init
  //  - Auto-resonance is disabled (open-loop operation)
  // -----------------------------------------------------------------------------

  // Bring up I2C just-in-time to minimize idle power
  init_i2c();

  // --- Wake the driver ---
  // MODE = 0x00 exits standby and clears internal state
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x00);

  // Short delay ensures the analog drive stage is fully awake
  _delay_us(250);

  // --- Enter Real-Time Playback (RTP) mode ---
  // In RTP mode:
  //  - CONTROL1 defines the vibration frequency (≈170 Hz)
  //  - RTPIN defines instantaneous drive amplitude
  //
  // This bypasses waveform libraries and gives cycle-level control
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x05);
  _delay_ms(1);

  // --- Apply drive ---
  // Full-scale RTP produces the strongest initial acceleration,
  // which dominates perceived intensity for short pulses
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_RTPIN, 127);

  // --- Let the LRA ring ---
  // ~12 ms ≈ 2 cycles at 170 Hz
  //
  // Shorter:
  //   <10 ms → sharper but weaker
  // Longer:
  //   >14 ms → stronger but starts to feel buzzy
  //
  // For a plastic watch case pressed against skin,
  // ~11–13 ms is the perceptual sweet spot.
  _delay_ms(12);

  // --- Hard stop / active brake ---
  // Setting RTPIN to zero engages braking (configured at init)
  // This kills ring-down and makes the pulse feel like a "tap"
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_RTPIN, 0);

  // --- Return to standby ---
  // Reduces idle current between metronome ticks
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x40);

  // Shut down I2C until next use
  i2c_shutdown();
}

static void show_BPM(void)
{
  // Scale BPM by 5: display_value = BPM / 5
  uint8_t scaled = bpm / 5;

  // Update PORTA LEDs (bits 0-2) - active low
  (scaled & 0x01) ? (PORTA.OUTCLR = LED_LSB_PIN) : (PORTA.OUTSET = LED_LSB_PIN);
  (scaled & 0x02) ? (PORTA.OUTCLR = LED_BIT1_PIN) : (PORTA.OUTSET = LED_BIT1_PIN);
  (scaled & 0x04) ? (PORTA.OUTCLR = LED_BIT2_PIN) : (PORTA.OUTSET = LED_BIT2_PIN);

  // Update PORTB LEDs (bits 3-4) - active low
  (scaled & 0x08) ? (PORTB.OUTCLR = LED_BIT3_PIN) : (PORTB.OUTSET = LED_BIT3_PIN);
  (scaled & 0x10) ? (PORTB.OUTCLR = LED_MSB_PIN) : (PORTB.OUTSET = LED_MSB_PIN);
}

static void show_status(void)
{
  PORTA.OUTCLR = LED_STATUS_PIN; // Active low
}

static void schedule_task(uint8_t task, uint16_t delay)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    tasks[task].timeDue = RTC.CNT + delay;
    tasks[task].flags |= TASK_PENDING;
  }
}

static void cancel_task(uint8_t task)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    tasks[task].flags &= ~TASK_PENDING;
  }
}

static void task_hide_status(void *ctx)
{
  (void)ctx;
  PORTA.OUTSET = LED_STATUS_PIN;
}

static void task_hide_BPM(void *ctx)
{
  (void)ctx;
  PORTA.OUTSET = LED_LSB_PIN | LED_BIT1_PIN | LED_BIT2_PIN;
  PORTB.OUTSET = LED_MSB_PIN | LED_BIT3_PIN;
}

static void task_haptic_standby(void *ctx)
{
  (void)ctx;
  init_i2c();
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x40); // Enter standby mode
  i2c_shutdown();
}

static void task_play(void *ctx)
{
  (void)ctx;
  show_status();
  schedule_task(TASK_HIDE_STATUS, HAPTIC_DURATION);
  // schedule_task(TASK_HAPTIC_STANDBY, HAPTIC_DURATION);

  uint16_t ticks = (60UL * 1024) / bpm;
  schedule_task(TASK_PLAY, ticks);

  haptic_trigger();
}

static void task_pause(void *ctx)
{
  (void)ctx;
  cancel_task(TASK_PLAY);
  task_hide_BPM(NULL);
  task_hide_status(NULL);
  task_haptic_standby(NULL);
}

static void task_debounce(void *ctx)
{
  Button *btn = (Button *)ctx;
  bool physical_pressed = !(PORTC.IN & btn->pinMask);

  if (physical_pressed)
  {
    if (btn->state == BTN_IDLE)
    {
      btn->state = BTN_PRESSED;
      if (btn->onPress)
        btn->onPress();
    }
    schedule_task(btn->taskIndex, DEBOUNCE_DELAY);
  }
  else
  {
    btn->state = BTN_IDLE;
    PORTC.INTFLAGS = btn->pinMask;
    *(btn->pinCtrl) = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
  }
}

static void action_dec(void)
{
  bpm = MAX(bpm - BPM_INCREMENT, BPM_MIN);
  show_BPM();
  schedule_task(TASK_HIDE_BPM, LED_ON_DURATION);
}

static void action_inc(void)
{
  bpm = MIN(bpm + BPM_INCREMENT, BPM_MAX);
  show_BPM();
  schedule_task(TASK_HIDE_BPM, LED_ON_DURATION);
}

static void action_play(void)
{
  bool isPlaying = (tasks[TASK_PLAY].flags & TASK_PENDING);
  schedule_task(isPlaying ? TASK_PAUSE : TASK_PLAY, 0);
}

static bool process_tasks(void)
{
  bool ranAny = false;
  uint16_t now;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    now = RTC.CNT;
  }

  for (uint8_t i = 0; i < NUM_TASKS; i++)
  {
    bool run = false;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      if ((tasks[i].flags & TASK_PENDING) &&
          (int16_t)(tasks[i].timeDue - now) <= 0)
      {
        tasks[i].flags &= ~TASK_PENDING;
        run = true;
      }
    }

    if (run)
    {
      tasks[i].callback(tasks[i].context);
      ranAny = true;
    }
  }

  return ranAny;
}

static bool is_any_task_pending(void)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    for (uint8_t i = 0; i < NUM_TASKS; i++)
    {
      if (tasks[i].flags & TASK_PENDING)
      {
        return true;
      }
    }
  }
  return false;
}

static void prepare_next_wakeup(void)
{
  uint16_t now;
  uint16_t next = 0;
  bool found = false;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    now = RTC.CNT;
    for (uint8_t i = 0; i < NUM_TASKS; i++)
    {
      if (tasks[i].flags & TASK_PENDING)
      {
        if (!found || (int16_t)(tasks[i].timeDue - now) < (int16_t)(next - now))
        {
          next = tasks[i].timeDue;
          found = true;
        }
      }
    }

    if (found)
    {
      RTC.CMP = next;

      while (RTC.STATUS & RTC_CMPBUSY_bm)
        ;

      RTC.INTFLAGS = RTC_CMP_bm;
      RTC.INTCTRL = RTC_CMP_bm;
    }
    else
    {
      RTC.INTCTRL = 0;
    }
  }
}

ISR(RTC_CNT_vect)
{
  RTC.INTFLAGS = RTC_CMP_bm;
}

ISR(PORTC_PORT_vect)
{
  uint8_t flags = PORTC.INTFLAGS;
  PORTC.INTFLAGS = flags;

  for (uint8_t i = 0; i < NUM_BUTTONS; i++)
  {
    if (flags & buttons[i].pinMask)
    {
      *(buttons[i].pinCtrl) = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;
      schedule_task(buttons[i].taskIndex, DEBOUNCE_DELAY);
    }
  }
}

int main(void)
{
  init_clock();
  init_rtc();
  init_unused_pins_low_power();
  init_buttons();
  init_leds();
  init_haptic_driver();

  sei();

  while (1)
  {
    while (process_tasks())
      ;

    prepare_next_wakeup();

    set_sleep_mode(is_any_task_pending()
                       ? SLEEP_MODE_STANDBY
                       : SLEEP_MODE_PWR_DOWN);

    cli();
    if (process_tasks())
    {
      sei();
      continue;
    }

    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
  }

  return 0;
}
