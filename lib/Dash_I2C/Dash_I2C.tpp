/**
 * @file Dash_I2C.tpp
 * @author ChiHo Ngan, Red Bird Racing 
 * @brief Implementation of I2C drivers made by Carson on Dash
 * @version 1.0
 * @date 2026-07-24
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "Dash_I2C.hpp"
#include <util/delay.h>
#include <avr/pgmspace.h>

namespace
{
    constexpr uint8_t EN_BIT = 0b00000100; /**< Enable bit on the PCF8574 expander */
    constexpr uint8_t RS_BIT = 0b00000001; /**< Register-select bit: 0 = command, 1 = data */
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

    // for some 1-line displays a taller font can be selected
    if ((dotsize != 0) && (lines == 1))
    {
        displayfunction |= LCD_5x10DOTS;
    }
    _delay_ms(50);
    expanderWriteBlocking(backlightval); // put the expander port into a known state (like hd44780_I2Cexp::ioinit)
    _delay_ms(100);                      // power-on settle: hd44780::begin() waits delay(100) here
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
        write(pgm_read_byte(&charmap[i])); // charmap lives in flash
    }
    // hd44780::createChar() restores the DDRAM address afterwards so the chip is
    // not left addressing CGRAM. This driver is write-only and cannot read the
    // previous address back, so return to a known (0,0) instead.
    setCursor(0, 0);
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
    char buf[6]; // enough for a 16-bit value in decimal ("65535") or hex ("FFFF")
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

void DashLcd::sendBlocking(uint8_t value, uint8_t mode)
{
    uint8_t highnib = value & 0xF0;
    uint8_t lownib = static_cast<uint8_t>(value << 4) & 0xF0;
    writeNibbleBlocking(highnib | mode);
    writeNibbleBlocking(lownib | mode);
}

void DashLcd::writeNibbleBlocking(uint8_t nibble_with_mode)
{
    expanderWriteBlocking(nibble_with_mode);                              // data lines valid, EN low
    expanderWriteBlocking(nibble_with_mode | EN_BIT);                     // EN high
    _delay_us(1);                                                         // enable pulse must be held >450ns
    expanderWriteBlocking(nibble_with_mode & static_cast<uint8_t>(~EN_BIT)); // EN low, latches nibble
    _delay_us(50);                                                        // commands need >37us to settle
}

void DashLcd::expanderWriteBlocking(uint8_t data)
{
    blockingTxBuffer = static_cast<uint8_t>((data | backlightval) & 0xFF);
    I2cTransaction tx = I2cTransaction::makeWrite(addr, 1, &blockingTxBuffer);

    while (!i2c.pushPriority(tx))
    {
        i2c.pump();
    }
    i2c.pump();
    while (!i2c.priorityEmpty())
    {
        i2c.pump();
    }
}

bool DashLcd::sendQueued(uint8_t value, uint8_t mode)
{
    uint8_t highnib = value & 0xF0;
    uint8_t lownib = static_cast<uint8_t>(value << 4) & 0xF0;
    if (!writeNibbleQueued(highnib | mode))
    {
        return false;
    }
    return writeNibbleQueued(lownib | mode);
}

bool DashLcd::writeNibbleQueued(uint8_t nibble_with_mode)
{
    // Matches hd44780_I2Cexp::write4bits(): two expander writes per nibble - E
    // raised together with the data/RS lines, then E lowered to latch. That lib
    // notes this "violates the spec but seems to work reliably"; here each write
    // is also a separate ~200us (100kHz) I2C transaction, so the enable pulse is
    // far wider than the >450ns minimum and no _delay_us() is needed.
    if (!expanderWriteQueued(nibble_with_mode | EN_BIT))
    {
        return false;
    }
    return expanderWriteQueued(nibble_with_mode & static_cast<uint8_t>(~EN_BIT));
}

bool DashLcd::expanderWriteQueued(uint8_t data)
{
    uint8_t slot = queuedTxIndex & QUEUED_TX_MASK;
    queuedTxBuffer[slot] = static_cast<uint8_t>((data | backlightval) & 0xFF);
    I2cTransaction tx = I2cTransaction::makeWrite(addr, 1, &queuedTxBuffer[slot]);

    // Backpressure, not drop: a page redraw enqueues far more than the ring
    // buffer holds, so pump the bus until a slot frees instead of silently
    // losing the rest of the frame. The priority queue holds at most
    // PRIORITY_SIZE-1 entries while queuedTxBuffer has PRIORITY_SIZE slots, so
    // the slot we just wrote is never one that is still in flight.
    while (!i2c.pushPriority(tx))
    {
        i2c.pump();
    }
    ++queuedTxIndex;
    return true;
}
