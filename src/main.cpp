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
   PC1 16 | BUTTON1     | Decrease BPM   | Input     | Pullup, falling edge int
   PC2 17 | BUTTON2     | Play/Pause     | Input     | Pullup, falling edge int ASYNC
   PC3 18 | BUTTON3     | Increase BPM   | Input     | Pullup, falling edge int

   Used: 12 pins (3 buttons + 6 LEDs + 2 I2C + 1 UPDI)
   Free: 6 pins (PA1-PA3, PB2-PB3, PC0)
   ============================================================================ */

// Button pins on Port C
#define BUTTON1_PIN PIN1_bm // PC1 - Decrease BPM
#define BUTTON2_PIN PIN2_bm // PC2 - Play/Pause
#define BUTTON3_PIN PIN3_bm // PC3 - Increase BPM

// Status LED
#define LED_STATUS_PIN PIN4_bm // PA4 - Status LED (active low)

// BPM Display LEDs (5 bits, scaled by 5)
#define LED_MSB_PIN PIN4_bm  // PB4 - MSB (bit 4)
#define LED_BIT3_PIN PIN5_bm // PB5 - bit 3
#define LED_BIT2_PIN PIN7_bm // PA7 - bit 2
#define LED_BIT1_PIN PIN6_bm // PA6 - bit 1
#define LED_LSB_PIN PIN5_bm  // PA5 - LSB (bit 0)

// Timing constants (in RTC ticks at 1024Hz, 1 tick = 1ms)
#define DEBOUNCE_DELAY 20
#define LED_ON_DURATION 500
#define HAPTIC_DURATION 75 // strong click waveform duration + safety margin
#define BPM_INCREMENT 5
#define BPM_MIN 5
#define BPM_MAX 155

// DRV2605L I2C address and registers
#define DRV2605L_ADDR 0x5A
#define DRV2605L_REG_STATUS 0x00
#define DRV2605L_REG_MODE 0x01
#define DRV2605L_REG_RTPIN 0x02
#define DRV2605L_REG_LIBRARY 0x03
#define DRV2605L_REG_WAVESEQ1 0x04
#define DRV2605L_REG_GO 0x0C
#define DRV2605L_REG_FEEDBACK 0x1A
#define DRV2605L_REG_CONTROL1 0x1B
#define DRV2605L_REG_CONTROL3 0x1D

typedef struct
{
  volatile uint16_t timeDue; // Time when task should execute (in RTC ticks)
  volatile bool isPending;   // Set by ISR, cleared after execution
  void (*callback)(void);
} Task;

#define NUM_TASKS 6
enum TaskIndex
{
  TASK_OUTPUT_ON = 0,
  TASK_OUTPUT_OFF,
  TASK_LED_OFF,
  TASK_BUTTON1_DEBOUNCE,
  TASK_BUTTON2_DEBOUNCE,
  TASK_BUTTON3_DEBOUNCE,
};

Task taskList[NUM_TASKS];

// BPM control (0-155 range)
volatile uint8_t bpm = 60;       // Default BPM
volatile bool isPlaying = false; // Play/pause state

static void outputOnTask(void);
static void outputOffTask(void);
static void ledOffTask(void);
static void button1DebounceTask(void);
static void button2DebounceTask(void);
static void button3DebounceTask(void);

static void debugLedOn(void)
{
  // delete before final code commits
  PORTC.DIRSET = PIN0_bm;
  PORTC.OUTCLR = PIN0_bm;
}

static void debugLedOff(void)
{
  // delete before final code commits
  PORTC.DIRSET = PIN0_bm;
  PORTC.OUTSET = PIN0_bm;
}

static void init_tasks(void)
{
  taskList[TASK_OUTPUT_ON].callback = outputOnTask;
  taskList[TASK_OUTPUT_ON].isPending = false;

  taskList[TASK_OUTPUT_OFF].callback = outputOffTask;
  taskList[TASK_OUTPUT_OFF].isPending = false;

  taskList[TASK_LED_OFF].callback = ledOffTask;
  taskList[TASK_LED_OFF].isPending = false;

  taskList[TASK_BUTTON1_DEBOUNCE].callback = button1DebounceTask;
  taskList[TASK_BUTTON1_DEBOUNCE].isPending = false;

  taskList[TASK_BUTTON2_DEBOUNCE].callback = button2DebounceTask;
  taskList[TASK_BUTTON2_DEBOUNCE].isPending = false;

  taskList[TASK_BUTTON3_DEBOUNCE].callback = button3DebounceTask;
  taskList[TASK_BUTTON3_DEBOUNCE].isPending = false;
}

static void init_clock(void)
{
  // Enable prescaler with div 4 for 4MHz (16MHz / 4)
  // PDIV = 0x1 (div 4), PEN = 1 (enable prescaler)
  // Requires Configuration Change Protection (CCP) write
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, (0x1 << 1) | 0x01);
}

static void init_haptic_driver(void)
{
  // Step 1: Wait 250µs after powerup before I2C commands
  _delay_us(250);

  init_i2c();

  // Step 3: Exit standby mode - set MODE to internal trigger (0x00)
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x00);

  // Step 4-5: Skip auto-calibration (using open-loop mode)

  // Step 6: Select library (Library 6 for LRA strong click)
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_LIBRARY, 0x06);

  // Step 7: Configure control registers for LRA open-loop mode

  // Set feedback control for LRA mode
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_FEEDBACK, 0x80); // LRA mode (bit 7 = 1)

  // Set Control3: LRA mode, open loop, strong braking
  // Bit 7 = 1 (LRA), Bit 4 = 1 (supply compensation disable for open loop)
  // Bits 2-0 = 0 (open loop, loop gain = 0)
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_CONTROL3, 0x90);

  // Set Control1: Drive time for 170Hz LRA
  // Drive time = (1 / (2 * 170Hz)) = 2.94ms ≈ 3ms
  // Drive time register = (time_ms - 0.5) / 0.1 = (3 - 0.5) / 0.1 = 25 = 0x19
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_CONTROL1, 0x93); // Drive time + startup boost

  // Load waveform sequencer: Strong Click 100% (effect 1)
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_WAVESEQ1, 0x01);     // Strong click
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_WAVESEQ1 + 1, 0x00); // End sequence

  // Step 8: Put device in standby mode for low power
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x40); // Set STANDBY bit

  i2c_shutdown();
}

static void init_rtc(void)
{
  while (RTC.STATUS > 0)
    ;

  RTC.CTRLA = 0; // Disable RTC before configuration (required for CLKSEL change)

  while (RTC.STATUS > 0)
    ;

  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc; // Select 1.024kHz internal oscillator (must be done while RTC disabled)

  while (RTC.STATUS > 0)
    ;

  RTC.INTFLAGS = RTC_OVF_bm | RTC_CMP_bm;

  while (RTC.STATUS > 0)
    ;

  RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RTCEN_bm | RTC_RUNSTDBY_bm; // DIV1 for 1024Hz (1.024kHz / 1 = 1.024kHz, ~0.977ms per tick)

  while (RTC.STATUS > 0)
    ;

  RTC.INTCTRL = RTC_CMP_bm; // Enable compare interrupt
}

static void init_buttons(void)
{
  // Configure PC1, PC2, PC3 as inputs with pull-ups
  PORTC.DIRCLR = BUTTON1_PIN | BUTTON2_PIN | BUTTON3_PIN;
  PORTC.PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;
  PORTC.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
  PORTC.PIN3CTRL = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;
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

static void enterSleep(void)
{
  cli();           // Disable interrupts
  sleep_enable();  // Enable sleep mode
  sei();           // Re-enable interrupts
  sleep_cpu();     // Sleep with interrupts enabled (atomic operation)
  sleep_disable(); // Disable sleep mode after waking
}

static void scheduleTask(uint8_t taskIndex, uint16_t delayTicks)
{
  taskList[taskIndex].timeDue = RTC.CNT + delayTicks;
  taskList[taskIndex].isPending = true;
}

static void processTasks(void)
{
  // this is critical to wait for RTC to be ready
  while (RTC.STATUS > 0)
    ;

  uint16_t now = RTC.CNT;

  for (uint8_t i = 0; i < NUM_TASKS; i++)
  {
    bool shouldRun = false;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      if (taskList[i].isPending && (int16_t)(taskList[i].timeDue - now) <= 0)
      {
        taskList[i].isPending = false;
        shouldRun = true;
      }
    }

    if (shouldRun && taskList[i].callback != 0)
    {
      taskList[i].callback();
    }
  }
}

static void prepareNextWakeup(void)
{
  uint16_t nextTime = 0xFFFF;
  bool foundTask = false;

  for (uint8_t i = 0; i < NUM_TASKS; i++)
  {
    if (taskList[i].isPending)
    {
      if (!foundTask || (int16_t)(taskList[i].timeDue - nextTime) < 0)
      {
        nextTime = taskList[i].timeDue;
        foundTask = true;
      }
    }
  }

  if (foundTask)
  {
    uint16_t now = RTC.CNT;

    if ((int16_t)(nextTime - now) <= 0)
    {
      RTC.CMP = now + 1; // Deadline passed; wake up on next tick.
    }
    else
    {
      RTC.CMP = nextTime;
    }
  }
}

static void haptic_trigger(void)
{
  init_i2c();

  // Exit standby mode if device was in standby (Step 3 from datasheet)
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x00);

  // Send GO command to trigger waveform playback (Step 5 from datasheet)
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_GO, 0x01);

  i2c_shutdown();
}

static void statusLedOn(void)
{
  PORTA.OUTCLR = LED_STATUS_PIN; // Active low
}

static void statusLedOff(void)
{
  PORTA.OUTSET = LED_STATUS_PIN;
}

static void outputOnTask(void)
{
  if (!isPlaying)
    return;

  haptic_trigger();

  statusLedOn();

  scheduleTask(TASK_OUTPUT_OFF, HAPTIC_DURATION);

  uint16_t ticksPerBeat = (60UL * 1024) / bpm;
  scheduleTask(TASK_OUTPUT_ON, ticksPerBeat);
}

static void outputOffTask(void)
{
  // Put DRV2605L into standby mode after waveform completes (Step 7 from datasheet)
  init_i2c();
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x40); // Set STANDBY bit
  i2c_shutdown();

  statusLedOff();
}

static void ledOnTask(void)
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

static void ledOffTask(void)
{
  // Turn off all LEDs (active low - set HIGH)
  PORTA.OUTSET = LED_LSB_PIN | LED_BIT1_PIN | LED_BIT2_PIN;
  PORTB.OUTSET = LED_MSB_PIN | LED_BIT3_PIN;
}

static void button1DebounceTask(void)
{
  if (!(PORTC.IN & BUTTON1_PIN))
  {
    if (bpm >= BPM_MIN + BPM_INCREMENT)
    {
      bpm -= BPM_INCREMENT;
    }
    else
    {
      bpm = BPM_MIN;
    }

    ledOnTask();
    scheduleTask(TASK_LED_OFF, LED_ON_DURATION);
    PORTC.PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
  }
}

static void button2DebounceTask(void)
{
  if (!(PORTC.IN & BUTTON2_PIN))
  {
    isPlaying = !isPlaying;
    if (isPlaying)
    {
      outputOnTask();
      ledOnTask();
      scheduleTask(TASK_LED_OFF, LED_ON_DURATION);
      PORTC.PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
      PORTC.PIN3CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
    }
    else
    {
      for (uint8_t i = 0; i < NUM_TASKS; i++)
      {
        taskList[i].isPending = false;
      }
      outputOffTask();
      ledOffTask();
      PORTC.PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;
      PORTC.PIN3CTRL = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;
    }
    PORTC.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
  }
}

static void button3DebounceTask(void)
{
  if (!(PORTC.IN & BUTTON3_PIN))
  {
    if (bpm <= BPM_MAX - BPM_INCREMENT)
    {
      bpm += BPM_INCREMENT;
    }
    else
    {
      bpm = BPM_MAX;
    }

    ledOnTask();
    scheduleTask(TASK_LED_OFF, LED_ON_DURATION);
    PORTC.PIN3CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
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

  if ((flags & BUTTON1_PIN) && !(PORTC.IN & BUTTON1_PIN) && !taskList[TASK_BUTTON1_DEBOUNCE].isPending && isPlaying)
  {
    PORTC.PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;
    scheduleTask(TASK_BUTTON1_DEBOUNCE, DEBOUNCE_DELAY);
  }
  if ((flags & BUTTON2_PIN) && !(PORTC.IN & BUTTON2_PIN) && !taskList[TASK_BUTTON2_DEBOUNCE].isPending)
  {
    PORTC.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;
    scheduleTask(TASK_BUTTON2_DEBOUNCE, DEBOUNCE_DELAY);
  }
  if ((flags & BUTTON3_PIN) && !(PORTC.IN & BUTTON3_PIN) && !taskList[TASK_BUTTON3_DEBOUNCE].isPending && isPlaying)
  {
    PORTC.PIN3CTRL = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;
    scheduleTask(TASK_BUTTON3_DEBOUNCE, DEBOUNCE_DELAY);
  }
}

int main(void)
{
  init_clock();
  init_rtc();
  init_buttons();
  init_leds();
  init_tasks();
  init_haptic_driver();

  sei();

  while (1)
  {
    processTasks();
    prepareNextWakeup();
    set_sleep_mode((isPlaying || taskList[TASK_BUTTON2_DEBOUNCE].isPending)
                       ? SLEEP_MODE_STANDBY
                       : SLEEP_MODE_PWR_DOWN);
    enterSleep();
  }

  return 0;
}