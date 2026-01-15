#include <Arduino.h>
#include <Wire.h>

#define DRV2605_ADDR 0x5A
#define DRV2605_MODE 0x01
#define DRV2605_LIBRARY 0x03
#define DRV2605_WAVESEQ1 0x04
#define DRV2605_GO 0x0C

#define LED_BUILTIN 10

// 80 times per minute = once every 750ms
#define HAPTIC_INTERVAL 750
#define LED_ON_TIME 50

unsigned long lastHapticTime = 0;
unsigned long ledOnTime = 0;
bool ledState = false;

void writeRegister(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(DRV2605_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();

  // Exit standby
  writeRegister(DRV2605_MODE, 0x00);
  // Select library 1
  writeRegister(DRV2605_LIBRARY, 1);
  // Set mode to internal trigger
  writeRegister(DRV2605_MODE, 0x00);
}

// the loop function runs over and over again forever
void loop()
{
  unsigned long currentTime = millis();

  // Check if it's time to trigger haptic and LED
  if (currentTime - lastHapticTime >= HAPTIC_INTERVAL)
  {
    lastHapticTime = currentTime;

    // Turn LED on
    digitalWrite(LED_BUILTIN, HIGH);
    ledOnTime = currentTime;
    ledState = true;

    // Trigger haptic: 1 = Strong Click
    writeRegister(DRV2605_WAVESEQ1, 1);
    writeRegister(DRV2605_GO, 1);
  }

  // Check if it's time to turn LED off
  if (ledState && (currentTime - ledOnTime >= LED_ON_TIME))
  {
    digitalWrite(LED_BUILTIN, LOW);
    ledState = false;
  }
}