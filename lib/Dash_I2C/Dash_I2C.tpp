/**
 * @file Dash_I2C.tpp
 * @author ChiHo Ngan, Red Bird Racing 
 * @brief DashLcd implementation.
 * @version 1.0
 * @date 2026-07-24
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "Dash_I2C.hpp"
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>

namespace
{
    constexpr uint8_t EN_BIT = 0b00000100; ///< PCF8574 enable bit
    constexpr uint8_t RS_BIT = 0b00000001; ///< Register select: 0 = command, 1 = data
}

void DashLcd::begin(uint8_t lcd_cols, uint8_t lines, uint8_t dotsize)
{
    cols = lcd_cols;
    displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
    if (lines > 1)
    {
        displayfunction |= LCD_2LINE;
    }
    numlines = lines;

    if ((dotsize != 0) && (lines == 1))
    {
        displayfunction |= LCD_5x10DOTS;
    }
    _delay_ms(50);
    expanderWriteBlocking(backlightval);
    _delay_ms(100);
    writeNibbleBlocking(0x03 << 4);
    _delay_us(4500);
    writeNibbleBlocking(0x03 << 4);
    _delay_us(4500);
    writeNibbleBlocking(0x03 << 4);
    _delay_us(150);
    writeNibbleBlocking(0x02 << 4);
    sendBlocking(LCD_FUNCTIONSET | displayfunction, 0);
    displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    sendBlocking(LCD_DISPLAYCONTROL | displaycontrol, 0);
    clear();
    displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    sendBlocking(LCD_ENTRYMODESET | displaymode, 0);
    sendBlocking(LCD_RETURNHOME, 0);
    _delay_us(2000);
}

void DashLcd::init()
{
    initPriv();
}

void DashLcd::initPriv()
{
    displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
    begin(cols, rows);
}

void DashLcd::backlight()
{
    backlightval = LCD_BACKLIGHT;
    expanderWriteBlocking(0);
}

void DashLcd::clear()
{
    sendBlocking(LCD_CLEARDISPLAY, 0);
    _delay_us(2000);
}

void DashLcd::createChar(uint8_t location, const uint8_t *charmap)
{
    location &= 0x7;
    sendBlocking(LCD_SETCGRAMADDR | (location << 3), 0);
    for (uint8_t i = 0; i < 8; ++i)
    {
        write(pgm_read_byte(&charmap[i]));
    }
    setCursor(0, 0); // leave CGRAM addressing
}

void DashLcd::setCursor(uint8_t col, uint8_t row)
{
    static constexpr uint8_t row_offsets[4] = {0x00, 0x40, 0x14, 0x54};
    if (row >= numlines)
    {
        row = numlines - 1; 
    }
    if (row > 3)
    {
        row = 3;
    }
    sendBlocking(LCD_SETDDRAMADDR | (col + row_offsets[row]), 0);
}

size_t DashLcd::print(const char *str)
{
    size_t n = 0;
    while (*str)
    {
        write(static_cast<uint8_t>(*str));
        ++str;
        ++n;
    }
    return n;
}


size_t DashLcd::print(const __FlashStringHelper *str)
{
    PGM_P p = reinterpret_cast<PGM_P>(str);
    size_t n = 0;
    for (uint8_t c; (c = pgm_read_byte(p + n)) != 0; ++n)
    {
        write(c);
    }
    return n;
}

size_t DashLcd::printQueued(const __FlashStringHelper *str)
{
    PGM_P p = reinterpret_cast<PGM_P>(str);
    size_t n = 0;
    for (uint8_t c; (c = pgm_read_byte(p + n)) != 0; ++n)
    {
        writeQueued(c);
    }
    return n;
}

size_t DashLcd::write(uint8_t value)
{
    sendBlocking(value, RS_BIT);
    return 1;
}

bool DashLcd::setCursorQueued(uint8_t col, uint8_t row)
{
    static constexpr uint8_t row_offsets[4] = {0x00, 0x40, 0x14, 0x54};
    if (row >= numlines)
    {
        row = numlines - 1;
    }
    if (row > 3)
    {
        row = 3;
    }
    return sendQueued(LCD_SETDDRAMADDR | (col + row_offsets[row]), 0);
}

size_t DashLcd::printQueued(const char *str)
{
    size_t n = 0;
    while (*str)
    {
        if (!writeQueued(static_cast<uint8_t>(*str)))
        {
            break; 
        }
        ++str;
        ++n;
    }
    return n;
}
size_t DashLcd::print(uint16_t value, uint8_t base)
{
    char buf[6];
    uint8_t i = sizeof(buf);
    buf[--i] = '\0';
    if (value == 0)
    {
        buf[--i] = '0';
    }
    else
    {
        while (value != 0 && i > 0)
        {
            uint8_t digit = value % base;
            buf[--i] = (digit < 10) ? static_cast<char>('0' + digit) : static_cast<char>('A' + digit - 10);
            value /= base;
        }
    }
    return print(&buf[i]);
}

size_t DashLcd::printQueued(uint16_t value, uint8_t base)
{
    char buf[6];
    uint8_t i = sizeof(buf);
    buf[--i] = '\0';
    if (value == 0)
    {
        buf[--i] = '0';
    }
    else
    {
        while (value != 0 && i > 0)
        {
            uint8_t digit = value % base;
            buf[--i] = (digit < 10) ? static_cast<char>('0' + digit) : static_cast<char>('A' + digit - 10);
            value /= base;
        }
    }
    return printQueued(&buf[i]);
}

bool DashLcd::writeQueued(uint8_t value)
{
    return sendQueued(value, RS_BIT);
}

/**
 * @brief Encode one HD44780 byte as 4 expander bytes: high nibble with E high, E low,
 * then the low nibble the same way.
 * @details Sent as one I2C transaction. At 100 kHz each byte takes ~90 us, which
 * covers the E pulse and command settle times without delays.
 */
void DashLcd::fillSend(uint8_t *out, uint8_t value, uint8_t mode) const
{
    uint8_t hi = static_cast<uint8_t>((value & 0xF0) | mode | backlightval);
    uint8_t lo = static_cast<uint8_t>((value << 4) | mode | backlightval);
    out[0] = hi | EN_BIT;
    out[1] = hi;
    out[2] = lo | EN_BIT;
    out[3] = lo;
}

/**
 * @brief Queue `data` and wait until the bus has sent it.
 * @details pump() is only called to start an idle bus. Calling it in the wait loop
 * would trip the I2C driver's watchdog.
 */
void DashLcd::pushWait(const uint8_t *data, uint8_t len)
{
    I2cTransaction tx = I2cTransaction::makeWrite(addr, len, data);
    i2c.pump();
    while (!i2c.push(tx))
    {
    }
    i2c.pump();
    while (!i2c.queueEmpty())
    {
    }
}

void DashLcd::sendBlocking(uint8_t value, uint8_t mode)
{
    fillSend(blockingTxBuffer, value, mode);
    pushWait(blockingTxBuffer, 4);
}

/** @brief Send a single nibble. Only used by the begin() reset sequence. */
void DashLcd::writeNibbleBlocking(uint8_t nibble_with_mode)
{
    expanderWriteBlocking(nibble_with_mode);
    expanderWriteBlocking(nibble_with_mode | EN_BIT);
    _delay_us(1);                                                            // E pulse > 450 ns
    expanderWriteBlocking(nibble_with_mode & static_cast<uint8_t>(~EN_BIT)); // E low latches
    _delay_us(50);                                                           // command settle > 37 us
}

void DashLcd::expanderWriteBlocking(uint8_t data)
{
    blockingTxBuffer[0] = static_cast<uint8_t>(data | backlightval);
    pushWait(blockingTxBuffer, 1);
}

bool DashLcd::sendQueued(uint8_t value, uint8_t mode)
{
    if (dryRun)
    {
        dryCrc = _crc16_update(_crc16_update(dryCrc, mode), value);
        return true;
    }
    // The queue holds at most PRIORITY_SIZE - 1 entries, so this slot is never in flight.
    uint8_t *slot = queuedTxBuffer[queuedTxIndex & QUEUED_TX_MASK];
    fillSend(slot, value, mode);
    I2cTransaction tx = I2cTransaction::makeWrite(addr, 4, slot);
    // Queue full: wait for the ISR to free a slot.
    if (!i2c.push(tx))
    {
        i2c.pump();
        while (!i2c.push(tx))
        {
        }
    }
    ++queuedTxIndex;
    return true;
}
