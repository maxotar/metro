#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/cpufunc.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
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
   PA2  1 | SW2         | Play/Pause     | Input     | Internal pullup, FALLING
   PA3  2 | -           | FREE           | -         | EXTCLK capable
   PA4  5 | LED_STATUS  | Beat indicator | Output    | Active low
   PA5  6 | LED5 (MSB)  | LED bit 4      | Output    | Active low
   PA6  7 | LED4        | LED bit 3      | Output    | Active low
   PA7  8 | LED3        | LED bit 2      | Output    | Active low
   PB0 14 | TWI0 SCL    | I2C Clock      | I2C       | 4.7kΩ pullup
   PB1 13 | TWI0 SDA    | I2C Data       | I2C       | 4.7kΩ pullup
   PB2 12 | SW3         | Increase BPM   | Input     | Internal pullup, FALLING
   PB3 11 | -           | FREE           | -         | Available
   PB4 10 | LED1 (LSB)  | LED bit 0      | Output    | Active low
   PB5  9 | LED2        | LED bit 1      | Output    | Active low
   PC0 15 | -           | FREE           | -         | Available
   PC1 16 | -           | FREE           | -         | Available
   PC2 17 | SW1         | Decrease BPM   | Input     | Internal pullup, FALLING
   PC3 18 | -           | FREE           | -         | Available

   Used: 12 pins (3 buttons + 5 BPM LEDs + 1 status LED + 2 I2C + 1 UPDI)
   Free:  6 pins (PA1, PA3, PB3, PC0, PC1, PC3)
   Power: 2 pins (VDD, GND)
   Total: 20
   ============================================================================ */

typedef struct
{
  uint8_t pinMask;
  volatile PORT_t *port;
  volatile uint8_t *pinCtrl;
  uint8_t taskIndex;
  void (*onPress)(void);
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
static void task_hide_bpm(void *ctx);
static void task_hide_status(void *ctx);
static void task_debounce(void *ctx);
static void action_dec(void);
static void action_inc(void);
static void action_play_pause(void);

enum TaskIndex
{
  TASK_BTN_DEC_DEBOUNCE,
  TASK_BTN_PLAY_DEBOUNCE,
  TASK_BTN_INC_DEBOUNCE,
  TASK_PLAY,
  TASK_HAPTIC_STANDBY,
  TASK_HIDE_BPM,
  TASK_HIDE_STATUS,
  TASK_PAUSE,
  NUM_TASKS
};

static Button buttons[] = {
    {PIN2_bm, &PORTC, &PORTC.PIN2CTRL, TASK_BTN_DEC_DEBOUNCE, action_dec},         // SW1 - PC2 - Decrease BPM
    {PIN2_bm, &PORTA, &PORTA.PIN2CTRL, TASK_BTN_PLAY_DEBOUNCE, action_play_pause}, // SW2 - PA2 - Play/Pause
    {PIN2_bm, &PORTB, &PORTB.PIN2CTRL, TASK_BTN_INC_DEBOUNCE, action_inc},         // SW3 - PB2 - Increase BPM
};

static Task tasks[NUM_TASKS] = {
    {0, 0, task_debounce, &buttons[0]}, // TASK_BTN_DEC_DEBOUNCE
    {0, 0, task_debounce, &buttons[1]}, // TASK_BTN_PLAY_DEBOUNCE
    {0, 0, task_debounce, &buttons[2]}, // TASK_BTN_INC_DEBOUNCE
    {0, 0, task_play, 0},               // TASK_PLAY
    {0, 0, task_haptic_standby, 0},     // TASK_HAPTIC_STANDBY
    {0, 0, task_hide_bpm, 0},           // TASK_HIDE_BPM
    {0, 0, task_hide_status, 0},        // TASK_HIDE_STATUS
    {0, 0, task_pause, 0}               // TASK_PAUSE
};

// Macros
#define NUM_BUTTONS (sizeof(buttons) / sizeof(Button))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define TASK_PENDING (1 << 0)

// BPM Display LEDs (5 bits, scaled by 5) - all active low
// LED5 is MSB (bit 4), LED1 is LSB (bit 0)
#define LED_STATUS_PIN PIN4_bm // PA4 - beat flash indicator
#define LED5_PIN PIN5_bm       // PA5 - bit 4 (MSB)
#define LED4_PIN PIN6_bm       // PA6 - bit 3
#define LED3_PIN PIN7_bm       // PA7 - bit 2
#define LED2_PIN PIN5_bm       // PB5 - bit 1
#define LED1_PIN PIN4_bm       // PB4 - bit 0 (LSB)

// Timing constants (in RTC ticks at 1024Hz, 1 tick ≈ 1ms)
#define DEBOUNCE_DELAY 30
#define BEAT_FLASH_MS 100
#define HAPTIC_DRIVE_MS 12 // ~2 cycles at 170 Hz (see haptic_trigger for derivation)
#define LED_ON_DURATION 500
#define BPM_INCREMENT 5
#define BPM_MIN 5
#define BPM_MAX 155

// DRV2605L I2C address and registers used in this firmware
#define DRV2605L_ADDR 0x5A
#define DRV2605L_REG_MODE 0x01
#define DRV2605L_REG_RTPIN 0x02
#define DRV2605L_REG_CONTROL1 0x1B
#define DRV2605L_REG_CONTROL3 0x1D
#define DRV2605L_REG_FEEDBACK 0x1A
#define DRV2605L_REG_CLAMP_VOLT 0x17

// DRV2605L operating modes (MODE register values)
#define DRV2605L_MODE_NORMAL 0x00  // Active - registers writable, drive stage enabled
#define DRV2605L_MODE_RTP 0x05     // Real-Time Playback - amplitude set by RTPIN register
#define DRV2605L_MODE_STANDBY 0x40 // Low-power standby - drive disabled, registers retained

static uint8_t bpm = 100;

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
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, DRV2605L_MODE_NORMAL))
    return false;

  // --- Set LRA resonant frequency (~170 Hz) ---
  // CONTROL1 = Drive Time
  // DriveTime = (1 / (2 × f)) × 10000 − 1
  // For 170 Hz → ~28–29 → 0x1C
  //
  // This sets the waveform frequency for *all* RTP pulses
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_CONTROL1, 0x1C))
    return false;

  // --- Clamp voltage (attack strength) ---
  // Raised above rated voltage for very short pulses
  // ~3.6 V peak equivalent
  //
  // Safe because:
  //  - Pulse width is only ~12 ms
  //  - Duty cycle is extremely low (metronome use)
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_CLAMP_VOLT, 0xA5))
    return false;

  // --- Feedback / braking configuration ---
  // Bit 7 = 1 → LRA mode
  // Brake factor = 8× → fast stop, minimal ring-down
  // Loop gain = medium → stable, predictable response
  //
  // This is critical for making the vibration feel like a "tap"
  // instead of a buzz when mounted against skin.
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_FEEDBACK, 0xBC))
    return false;

  // --- Disable auto-resonance & force open-loop ---
  // Required because this LRA has unusually wide bandwidth
  // Auto-resonance causes frequency hunting and inconsistent pulses
  //
  // Bit 5 = 1 → disable auto-resonance
  // Bit 1 = 1 → bidirectional drive
  // Bit 0 = 1 → open-loop
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_CONTROL3, 0x23))
    return false;

  // --- Enter standby ---
  // haptic_trigger() exits standby and enters RTP mode for each pulse.
  // Registers are retained in standby, so init only needs to run once.
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, DRV2605L_MODE_STANDBY))
    return false;

  return true;
}

static void init_haptic_driver(void)
{
  // Allow DRV2605L internal POR to settle
  _delay_us(250);

  // Retry indefinitely - no error recovery path on a wearable, only recourse is battery pull
  while (1)
  {
    i2c_bus_recovery(); // clears stuck bus and calls init_i2c()

    if (try_init_haptic_driver())
    {
      i2c_shutdown();
      return;
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
  // PA1, PA3 are free (PA2=SW2, PA4=status LED, PA5-PA7=BPM LEDs)
  PORTA.OUTCLR = PIN1_bm | PIN3_bm;
  PORTA.DIRSET = PIN1_bm | PIN3_bm;
  PORTA.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;

  // PB3 is free (PB2 is now SW3)
  PORTB.OUTCLR = PIN3_bm;
  PORTB.DIRSET = PIN3_bm;
  PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;

  // PC0, PC1, PC3 are free (PC2 is now SW1)
  PORTC.OUTCLR = PIN0_bm | PIN1_bm | PIN3_bm;
  PORTC.DIRSET = PIN0_bm | PIN1_bm | PIN3_bm;
  PORTC.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTC.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTC.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
}

static void init_buttons(void)
{
  for (uint8_t i = 0; i < NUM_BUTTONS; i++)
  {
    buttons[i].port->DIRCLR = buttons[i].pinMask;
    *(buttons[i].pinCtrl) = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
  }
}

static void init_leds(void)
{
  // Configure LED pins as outputs on PORTA (PA4=status, PA5=LED5 MSB, PA6=LED4, PA7=LED3)
  PORTA.DIRSET = LED_STATUS_PIN | LED5_PIN | LED4_PIN | LED3_PIN;
  PORTA.OUTSET = LED_STATUS_PIN | LED5_PIN | LED4_PIN | LED3_PIN; // Start HIGH (LEDs off - active low)

  // Configure LED pins as outputs on PORTB (PB4=LED1 LSB, PB5=LED2)
  PORTB.DIRSET = LED1_PIN | LED2_PIN;
  PORTB.OUTSET = LED1_PIN | LED2_PIN; // Start HIGH (LEDs off - active low)
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
  // MODE_NORMAL exits standby and clears internal state
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, DRV2605L_MODE_NORMAL);

  // Short delay ensures the analog drive stage is fully awake
  _delay_us(250);

  // --- Enter Real-Time Playback (RTP) mode ---
  // In RTP mode:
  //  - CONTROL1 defines the vibration frequency (≈170 Hz)
  //  - RTPIN defines instantaneous drive amplitude
  //
  // This bypasses waveform libraries and gives cycle-level control
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, DRV2605L_MODE_RTP);
  _delay_ms(1);

  // --- Apply drive ---
  // Full-scale RTP produces the strongest initial acceleration,
  // which dominates perceived intensity for short pulses
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_RTPIN, 127);

  // Shut down I2C - the DRV2605L drives the LRA autonomously from the RTPIN
  // register value, no bus activity needed during the drive window.
  i2c_shutdown();

  // Schedule RTPIN=0 (brake) + standby after ~2 LRA cycles at 170 Hz.
  // The CPU is free to sleep in STANDBY mode until this fires.
  schedule_task(TASK_HAPTIC_STANDBY, HAPTIC_DRIVE_MS);
}

static void show_bpm(void)
{
  // Scale BPM to a 5-bit display value: 0–31 represents BPM_MIN–BPM_MAX in steps of 5
  uint8_t scaled = bpm / 5;

  // LED1 (bit 0) and LED2 (bit 1) on PORTB - active low
  if (scaled & 0x01)
    PORTB.OUTCLR = LED1_PIN;
  else
    PORTB.OUTSET = LED1_PIN;
  if (scaled & 0x02)
    PORTB.OUTCLR = LED2_PIN;
  else
    PORTB.OUTSET = LED2_PIN;

  // LED3 (bit 2), LED4 (bit 3), LED5/MSB (bit 4) on PORTA - active low
  if (scaled & 0x04)
    PORTA.OUTCLR = LED3_PIN;
  else
    PORTA.OUTSET = LED3_PIN;
  if (scaled & 0x08)
    PORTA.OUTCLR = LED4_PIN;
  else
    PORTA.OUTSET = LED4_PIN;
  if (scaled & 0x10)
    PORTA.OUTCLR = LED5_PIN;
  else
    PORTA.OUTSET = LED5_PIN;
}

static void show_status(void)
{
  PORTA.OUTCLR = LED_STATUS_PIN; // Active low
}

static void task_hide_status(void *ctx)
{
  (void)ctx;
  PORTA.OUTSET = LED_STATUS_PIN;
}

static void task_hide_bpm(void *ctx)
{
  (void)ctx;
  PORTB.OUTSET = LED1_PIN | LED2_PIN;
  PORTA.OUTSET = LED3_PIN | LED4_PIN | LED5_PIN;
}

static void task_haptic_standby(void *ctx)
{
  (void)ctx;
  init_i2c();
  // RTPIN=0 engages braking - kills ring-down for a clean "tap" feel
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_RTPIN, 0);
  // Return to low-power standby
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, DRV2605L_MODE_STANDBY);
  i2c_shutdown();
}

static void task_play(void *ctx)
{
  (void)ctx;
  show_status();
  schedule_task(TASK_HIDE_STATUS, BEAT_FLASH_MS);

  uint16_t ticks = (60UL * 1024) / bpm;
  schedule_task(TASK_PLAY, ticks);

  haptic_trigger();
}

static void task_pause(void *ctx)
{
  (void)ctx;
  cancel_task(TASK_PLAY);
  cancel_task(TASK_HAPTIC_STANDBY);
  task_hide_bpm(0);
  task_hide_status(0);
  task_haptic_standby(0); // brake and standby unconditionally - harmless if motor already stopped
}

static void task_debounce(void *ctx)
{
  Button *btn = (Button *)ctx;
  // Debounce period elapsed - clear any accumulated flag and re-enable falling-edge interrupt
  btn->port->INTFLAGS = btn->pinMask;
  *(btn->pinCtrl) = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
}

static void action_dec(void)
{
  bpm = MAX(bpm - BPM_INCREMENT, BPM_MIN);
  show_bpm();
  schedule_task(TASK_HIDE_BPM, LED_ON_DURATION);
}

static void action_inc(void)
{
  bpm = MIN(bpm + BPM_INCREMENT, BPM_MAX);
  show_bpm();
  schedule_task(TASK_HIDE_BPM, LED_ON_DURATION);
}

static void action_play_pause(void)
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
  }

  if (found)
  {
    RTC.CMP = next;

    // Wait for CMP sync outside the atomic block - can take up to 2 ms at 1kHz
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

static void handle_button_isr(volatile PORT_t *port)
{
  uint8_t flags = port->INTFLAGS;
  port->INTFLAGS = flags;

  for (uint8_t i = 0; i < NUM_BUTTONS; i++)
  {
    if (buttons[i].port == port && (flags & buttons[i].pinMask))
    {
      *(buttons[i].pinCtrl) = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;
      if (buttons[i].onPress)
        buttons[i].onPress();
      schedule_task(buttons[i].taskIndex, DEBOUNCE_DELAY);
    }
  }
}

ISR(RTC_CNT_vect)
{
  RTC.INTFLAGS = RTC_CMP_bm;
}

ISR(PORTA_PORT_vect)
{
  handle_button_isr(&PORTA);
}

ISR(PORTB_PORT_vect)
{
  handle_button_isr(&PORTB);
}

ISR(PORTC_PORT_vect)
{
  handle_button_isr(&PORTC);
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
