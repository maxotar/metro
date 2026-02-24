#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

// ATtiny1616 TWI0 pins: PB0=SCL, PB1=SDA
#define I2C_PORT PORTB
#define I2C_SDA_PIN_bm PIN1_bm
#define I2C_SCL_PIN_bm PIN0_bm

// I2C bus speed - Standard mode (100kHz)
#define I2C_SCL_HZ 100000UL
#define I2C_RISE_TIME_NS 1000L // Rise time in nanoseconds for 100kHz

// Initialize I2C hardware
void init_i2c(void);

// Shutdown I2C and release pins as inputs (external pullups hold lines idle-high)
void i2c_shutdown(void);

// Write multiple bytes to I2C device
bool i2c_write_bytes(uint8_t addr7, const uint8_t *data, uint8_t len);

// Write a single byte to a register
bool i2c_write_reg_u8(uint8_t addr7, uint8_t reg, uint8_t value);

// Read a single byte from a register
bool i2c_read_reg_u8(uint8_t addr7, uint8_t reg, uint8_t *value);

// Recover stuck I2C bus by generating clock pulses
void i2c_bus_recovery(void);

#endif // I2C_H
