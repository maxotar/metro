#include "i2c.h"
#include <avr/io.h>

// F_CPU must be defined before including this file
#ifndef F_CPU
#error "F_CPU must be defined before including i2c.cpp"
#endif

static bool twi_wait(uint8_t mask)
{
    // Simple timeout to avoid lockups if the bus is held.
    for (uint16_t i = 0; i < 60000; i++)
    {
        if (TWI0.MSTATUS & mask)
        {
            return true;
        }
    }
    return false;
}

void init_i2c(void)
{
    // Configure pins as inputs (external pullups used)
    I2C_PORT.DIRCLR = I2C_SDA_PIN_bm | I2C_SCL_PIN_bm;

    // Disable before configuration.
    TWI0.MCTRLA = 0;

    // Baud rate with rise time compensation:
    // MBAUD = ((F_CPU/FREQ) - (F_CPU*T_RISE/1e9) - 10) / 2
    const uint32_t baud = ((F_CPU / I2C_SCL_HZ) - ((F_CPU * I2C_RISE_TIME_NS) / 1000000000UL) - 10) / 2;
    TWI0.MBAUD = (uint8_t)((baud > 255UL) ? 255UL : baud);

    // Force bus state to IDLE.
    TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;

    // Enable TWI master.
    TWI0.MCTRLA = TWI_ENABLE_bm;
}

void i2c_shutdown(void)
{
    // Disable TWI master.
    TWI0.MCTRLA = 0;
    TWI0.MCTRLB = 0;

    // Release pins and disable pullups to minimize leakage.
    I2C_PORT.DIRCLR = I2C_SDA_PIN_bm | I2C_SCL_PIN_bm;
    I2C_PORT.PIN0CTRL &= (uint8_t)~PORT_PULLUPEN_bm; // SCL
    I2C_PORT.PIN1CTRL &= (uint8_t)~PORT_PULLUPEN_bm; // SDA
}

bool i2c_write_reg_u8(uint8_t addr7, uint8_t reg, uint8_t value)
{
    // Address + write
    TWI0.MADDR = (uint8_t)(addr7 << 1);
    if (!twi_wait(TWI_WIF_bm))
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        return false;
    }
    // Check for bus errors or arbitration lost
    if (TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm))
    {
        while (!(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc))
            ; // Wait for IDLE
        return false;
    }
    if (TWI0.MSTATUS & TWI_RXACK_bm)
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        while (!(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc))
            ; // Wait for IDLE
        return false;
    }

    // Register
    TWI0.MDATA = reg;
    if (!twi_wait(TWI_WIF_bm))
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        while (!(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc))
            ;
        return false;
    }
    if (TWI0.MSTATUS & TWI_RXACK_bm)
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        while (!(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc))
            ;
        return false;
    }

    // Value
    TWI0.MDATA = value;
    if (!twi_wait(TWI_WIF_bm))
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        while (!(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc))
            ;
        return false;
    }
    if (TWI0.MSTATUS & TWI_RXACK_bm)
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        while (!(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc))
            ;
        return false;
    }

    // Stop
    TWI0.MCTRLB = TWI_MCMD_STOP_gc;
    while (!(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc))
        ; // Wait for IDLE
    return true;
}
