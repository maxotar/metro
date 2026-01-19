#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
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

// Macros
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

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

static void init_clock(void)
{
  // Prescaler DIV4 for 4MHz (16MHz / 4)
  // PDIV = 1 (divide by 2^(PDIV+1) = 2^2 = 4), PEN = 1 (enable)
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, (1 << CLKCTRL_PDIV_gp) | CLKCTRL_PEN_bm);
}

static bool try_init_haptic_driver_erm(void)
{
  // Exit standby mode - set MODE to internal trigger (0x00)
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x00))
    return false;

  // Small delay for mode change to take effect
  _delay_us(100);

  // Select library (Library 1 for ERM - TS2200 motors)
  // Library options: 1=A (2-3V), 2=B (3V), 3=C (3-5V), 4=D (4-5V), 5=E (5V)
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_LIBRARY, 0x01))
    return false;

  // Configure feedback control for ERM mode
  // Bit 7 = 0 (ERM mode), Bit 6 = 0 (4x brake factor), Bit 1-0 = medium gain
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_FEEDBACK, 0x00))
    return false;

  // Set Control3 - ERM mode, open loop
  // Bit 7 = 0 (ERM mode on N_ERM_LRA pin)
  // Bit 5 = 0 (analog input)
  // Bit 4 = 1 (supply compensation disable for open loop)
  // Bit 3 = 0 (data format: signed)
  // Bits 2-0 = 0 (loop gain = 0 for open loop)
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_CONTROL3, 0x10))
    return false;

  // Set Control1 - Drive time for ERM (typically default is fine)
  // Bit 7-5 = 100 (startup boost)
  // Bit 4-0 = 10011 (drive time - can use default for ERM)
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_CONTROL1, 0x93))
    return false;

  // Set Control2 - Unidirectional for ERM
  // Bit 3 = 0 (unidirectional input for ERM)
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_CONTROL2, 0x00))
    return false;

  // Load default waveform: Strong Click 100% (effect 1)
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_WAVESEQ1, 0x01))
    return false;
  // End sequence
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_WAVESEQ1 + 1, 0x00))
    return false;

  // Enter standby mode for low power consumption
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x40))
    return false;

  return true;
}

static bool try_init_haptic_driver(void)
{
  // Exit standby mode - set MODE to internal trigger (0x00)
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x00))
    return false;

  // Small delay for mode change to take effect
  _delay_us(100);

  // Select library (Library 6 for LRA strong click)
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_LIBRARY, 0x06))
    return false;

  // Configure feedback control for LRA mode
  // Bit 7 = 1 (LRA mode), Bit 6 = 0 (4x brake factor)
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_FEEDBACK, 0x80))
    return false;

  // Set Control3 - LRA mode, open loop
  // Bit 7 = 1 (LRA mode), Bit 4 = 1 (supply comp disable), Bits 2-0 = 0 (open loop)
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_CONTROL3, 0x90))
    return false;

  // Set Control1 - Drive time for 170Hz LRA + startup boost
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_CONTROL1, 0x93))
    return false;

  // Load default waveform: Strong Click 100% (effect 1)
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_WAVESEQ1, 0x01))
    return false;

  // End sequence
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_WAVESEQ1 + 1, 0x00))
    return false;

  // Enter standby mode for low power
  if (!i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x40))
    return false;

  return true;
}

static void init_haptic_driver(void)
{
  // Wait for DRV2605L power-on settling
  _delay_us(250);

  // Keep trying until initialization succeeds
  while (1)
  {
    // Recover bus and initialize I2C
    i2c_bus_recovery();
    init_i2c();

    // Try to configure the device
    if (try_init_haptic_driver_erm())
    {
      // Success! Clean up and exit
      i2c_shutdown();
      break;
    }

    // Failed - clean up before retry
    i2c_shutdown();

    // Wait before retry to avoid hammering the bus
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

static void init_all_pins_low_power(void)
{
  // Set all pins as outputs driving low
  PORTA.DIRSET = 0xFF;
  PORTA.OUTCLR = 0xFF;
  PORTB.DIRSET = 0xFF;
  PORTB.OUTCLR = 0xFF;
  PORTC.DIRSET = 0xFF;
  PORTC.OUTCLR = 0xFF;

  // Disable input buffers on all pins
  for (uint8_t i = 0; i < 8; i++)
  {
    (&PORTA.PIN0CTRL)[i] = PORT_ISC_INPUT_DISABLE_gc;
    (&PORTB.PIN0CTRL)[i] = PORT_ISC_INPUT_DISABLE_gc;
    (&PORTC.PIN0CTRL)[i] = PORT_ISC_INPUT_DISABLE_gc;
  }
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
  wdt_reset();
  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_OFF_gc); // Disable the Watchdog before sleep

  while (WDT.STATUS & WDT_SYNCBUSY_bm)
    ;

  cli();
  sleep_enable();
  sei();
  sleep_cpu();
  sleep_disable();

  /* * WDT Period Options (Based on 1.024kHz internal WDT clock):
   * WDT_PERIOD_OFF_gc      - Watchdog Disabled
   * WDT_PERIOD_8CLK_gc     - 8ms timeout
   * WDT_PERIOD_16CLK_gc    - 16ms timeout
   * WDT_PERIOD_32CLK_gc    - 32ms timeout
   * WDT_PERIOD_64CLK_gc    - 64ms timeout
   * WDT_PERIOD_128CLK_gc   - 128ms timeout
   * WDT_PERIOD_256CLK_gc   - 256ms timeout
   * WDT_PERIOD_512CLK_gc   - 512ms timeout
   * WDT_PERIOD_1KCLK_gc    - 1024ms (~1.0s)
   * WDT_PERIOD_2KCLK_gc    - 2048ms (~2.0s)
   * WDT_PERIOD_4KCLK_gc    - 4096ms (~4.1s)
   * WDT_PERIOD_8KCLK_gc    - 8192ms (~8.2s)
   */
  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_128CLK_gc);

  while (WDT.STATUS & WDT_SYNCBUSY_bm)
    ;
}

static void scheduleTask(uint8_t taskIndex, uint16_t delayTicks)
{
  taskList[taskIndex].timeDue = RTC.CNT + delayTicks;
  taskList[taskIndex].isPending = true;
}

static void processTasks(void)
{
  rtc_wait_ready(); // this is critical to wait for RTC to be ready

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

    if (shouldRun)
    {
      taskList[i].callback();
    }
  }
}

static void prepareNextWakeup(void)
{
  uint16_t nextTime = 0xFFFF;
  bool foundTask = false;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
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
  }

  if (foundTask)
  {
    rtc_wait_ready();
    uint16_t now = RTC.CNT;

    if ((int16_t)(nextTime - now) <= 0)
    {
      RTC.CMP = now + 1;
    }
    else
    {
      RTC.CMP = nextTime;
    }

    rtc_wait_ready();
  }
}

static void haptic_trigger(void)
{
  init_i2c();
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x00);
  _delay_us(250);                                         // Wait for device to exit standby before sending GO command
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_GO, 0x01); // Send GO command to trigger waveform playback
  i2c_shutdown();
}

static void haptic_standby(void)
{
  init_i2c();
  i2c_write_reg_u8(DRV2605L_ADDR, DRV2605L_REG_MODE, 0x40); // Enter standby mode
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
  haptic_standby();

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
    bpm = MAX(bpm - BPM_INCREMENT, BPM_MIN);

    ledOnTask();
    scheduleTask(TASK_LED_OFF, LED_ON_DURATION);
  }
  PORTC.PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
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
  }
  PORTC.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
}

static void button3DebounceTask(void)
{
  if (!(PORTC.IN & BUTTON3_PIN))
  {
    bpm = MIN(bpm + BPM_INCREMENT, BPM_MAX);

    ledOnTask();
    scheduleTask(TASK_LED_OFF, LED_ON_DURATION);
  }
  PORTC.PIN3CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
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
  RSTCTRL.RSTFR = 0xFF;

  init_clock();
  init_rtc();
  init_all_pins_low_power();
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
