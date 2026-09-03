/**
 * @file Dash_I2C.hpp
 * @author ChiHo Ngan, Red Bird Racing 
 * @brief LCD driver built directly on top of the
 *        custom I2C.hpp / I2C.tpp driver. This replaces the Arduino LiquidCrystal_I2C
 *        library: no Arduino.h, no Wire.h, and no Arduino Print class involved.
 * @version 1.0
 * @date 2026-07-24
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#ifndef DASH_I2C_HPP
#define DASH_I2C_HPP

#include <stdint.h>
#include <stddef.h>
#include "I2C.hpp"

// PRIORITY_SIZE: the priority queue is a true ring-buffer FIFO (I2C.hpp accepts a
//                push any time there is room), so it is what backs BOTH the
//                blocking API (init sequence, page setup labels, clear) AND the
//                *Queued API (setCursorQueued/printQueued/writeQueued). Each printed
//                character costs 4 I2C write transactions (2 nibbles x 2
//                transactions/nibble). It does NOT need to hold a whole redraw:
//                expanderWriteQueued() applies backpressure (pumps the bus until a
//                slot frees) instead of dropping, so a smaller ring just pumps more
//                often. 16 entries = 4 chars in flight, a good RAM/throughput
//                balance on a 2KB part. Each entry is sizeof(I2cTransaction)=4 B, so
//                this queue costs PRIORITY_SIZE*4 B of RAM. Must remain a power of 2
// RECURRING_SIZE: only used by pushRecurring(), which Dash_I2C never calls. Pinned
//                 to the minimum I2C.hpp allows (2) so it costs just 8 B.
constexpr uint16_t DASH_I2C_BITRATE_KBPS = 100;
constexpr uint8_t DASH_I2C_PRIORITY_SIZE = 16;
constexpr uint8_t DASH_I2C_RECURRING_SIZE = 2;
constexpr uint8_t DASH_I2C_WATCHDOG_MAX_COUNT = 16;

using DashI2CDriver = I2C<DASH_I2C_BITRATE_KBPS, DASH_I2C_PRIORITY_SIZE, DASH_I2C_RECURRING_SIZE, DASH_I2C_WATCHDOG_MAX_COUNT>;
extern DashI2CDriver i2c;
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTDECREMENT 0x00

#define LCD_DISPLAYON 0x04
#define LCD_CURSOROFF 0x00
#define LCD_BLINKOFF 0x00

#define LCD_4BITMODE 0x00
#define LCD_1LINE 0x00
#define LCD_2LINE 0x08
#define LCD_5x8DOTS 0x00
#define LCD_5x10DOTS 0x04

#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

/**
 * @brief driver over a I2C expander, built on the I2C driver
 * template in I2C.hpp/I2C.tpp. Drop-in replacement for Arduino's LiquidCrystal_I2C
 * for the subset of the API this project actually uses.
 */
class DashLcd
{
public:
    DashLcd(uint8_t lcd_addr, uint8_t lcd_cols, uint8_t lcd_rows)
        : addr(lcd_addr), cols(lcd_cols), rows(lcd_rows), backlightval(LCD_NOBACKLIGHT)
    {
    }

    void begin(uint8_t cols, uint8_t lines, uint8_t dotsize = LCD_5x8DOTS);
    void init();
    void backlight();
    void clear();
    void createChar(uint8_t location, const uint8_t *charmap); // charmap is 8 bytes in PROGMEM
    void setCursor(uint8_t col, uint8_t row);
    size_t print(const char *str);
    size_t print(uint16_t value, uint8_t base);
    size_t write(uint8_t value);
    bool setCursorQueued(uint8_t col, uint8_t row);
    size_t printQueued(const char *str);
    size_t printQueued(uint16_t value, uint8_t base);
    bool writeQueued(uint8_t value);

private:
    void initPriv();
    void sendBlocking(uint8_t value, uint8_t mode);
    void writeNibbleBlocking(uint8_t nibble_with_mode);
    void expanderWriteBlocking(uint8_t data);
    bool sendQueued(uint8_t value, uint8_t mode);
    bool writeNibbleQueued(uint8_t nibble_with_mode);
    bool expanderWriteQueued(uint8_t data);

    uint8_t addr;
    uint8_t cols;
    uint8_t rows;
    uint8_t displayfunction = 0;
    uint8_t displaycontrol = 0;
    uint8_t displaymode = 0;
    uint8_t numlines = 0;
    uint8_t backlightval;

    uint8_t blockingTxBuffer = 0;
    uint8_t queuedTxBuffer[DASH_I2C_PRIORITY_SIZE] = {};
    uint8_t queuedTxIndex = 0;
    static constexpr uint8_t QUEUED_TX_MASK = DASH_I2C_PRIORITY_SIZE - 1;
};

using LiquidCrystal_I2C = DashLcd;

#include "Dash_I2C.tpp"

#endif // DASH_I2C_HPP
