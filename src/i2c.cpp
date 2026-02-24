#include "i2c.h"
#include <avr/io.h>
#include <util/delay.h>

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

static bool twi_wait_idle(void)
{
    for (uint16_t i = 0; i < 60000; i++)
    {
        if (TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc)
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
    TWI0.MCTRLA = 0;
    TWI0.MCTRLB = 0;

    // Release pins as inputs - external pullups hold lines idle-high.
    // This leaves the bus in a valid idle state and avoids ~1.4 mA of
    // continuous pullup current that would flow if lines were driven low.
    I2C_PORT.DIRCLR = I2C_SDA_PIN_bm | I2C_SCL_PIN_bm;
}

bool i2c_write_bytes(uint8_t addr7, const uint8_t *data, uint8_t len)
{
    TWI0.MADDR = (uint8_t)(addr7 << 1);
    if (!twi_wait(TWI_WIF_bm))
        return false;
    if (TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_RXACK_bm))
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        twi_wait_idle();
        return false;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        TWI0.MDATA = data[i];
        if (!twi_wait(TWI_WIF_bm))
        {
            TWI0.MCTRLB = TWI_MCMD_STOP_gc;
            twi_wait_idle();
            return false;
        }
        if (TWI0.MSTATUS & TWI_RXACK_bm)
        {
            TWI0.MCTRLB = TWI_MCMD_STOP_gc;
            twi_wait_idle();
            return false;
        }
    }

    TWI0.MCTRLB = TWI_MCMD_STOP_gc;
    twi_wait_idle();
    return true;
}

bool i2c_write_reg_u8(uint8_t addr7, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_write_bytes(addr7, data, 2);
}

bool i2c_read_reg_u8(uint8_t addr7, uint8_t reg, uint8_t *value)
{
    // Write register address
    TWI0.MADDR = (uint8_t)(addr7 << 1);
    if (!twi_wait(TWI_WIF_bm))
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        twi_wait_idle();
        return false;
    }
    if (TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_RXACK_bm))
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        twi_wait_idle();
        return false;
    }

    TWI0.MDATA = reg;
    if (!twi_wait(TWI_WIF_bm))
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        twi_wait_idle();
        return false;
    }
    if (TWI0.MSTATUS & TWI_RXACK_bm)
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        twi_wait_idle();
        return false;
    }

    // Repeated start + read
    TWI0.MADDR = (uint8_t)((addr7 << 1) | 0x01);
    if (!twi_wait(TWI_RIF_bm))
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        twi_wait_idle();
        return false;
    }
    if (TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_RXACK_bm))
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        twi_wait_idle();
        return false;
    }

    // Read byte and NACK
    *value = TWI0.MDATA;
    TWI0.MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc;
    twi_wait_idle();
    return true;
}

void i2c_bus_recovery(void)
{
    // Disable TWI
    TWI0.MCTRLA = 0;

    // Configure SCL as output
    I2C_PORT.DIRSET = I2C_SCL_PIN_bm;

    // Generate up to 9 clock pulses
    for (uint8_t i = 0; i < 9; i++)
    {
        I2C_PORT.OUTCLR = I2C_SCL_PIN_bm;
        _delay_us(5);
        I2C_PORT.OUTSET = I2C_SCL_PIN_bm;
        _delay_us(5);

        // Check if SDA released
        if (I2C_PORT.IN & I2C_SDA_PIN_bm)
        {
            break;
        }
    }

    // Reconfigure as input and reinitialize
    I2C_PORT.DIRCLR = I2C_SCL_PIN_bm;
    init_i2c();
}
