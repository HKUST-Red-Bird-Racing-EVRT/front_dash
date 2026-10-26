/**
 * @file Dash_I2C.hpp
 * @author ChiHo Ngan, Red Bird Racing
 * @brief HD44780 LCD driver over a PCF8574 I2C expander, built on I2C.hpp.
 * @details Replaces LiquidCrystal_I2C for the subset of the API this project uses.
 * @version 1.0
 * @date 2026-07-24
 *
 * @copyright Copyright (c) 2026
 */

#ifndef DASH_I2C_HPP
#define DASH_I2C_HPP

#include <stdint.h>
#include <stddef.h>
#include "I2C.hpp"

class __FlashStringHelper;

constexpr uint16_t DASH_I2C_BITRATE_KBPS = 100;      ///< I2C bus speed
constexpr uint8_t DASH_I2C_PRIORITY_SIZE = 8;        ///< I2C queue length, power of 2. One entry per LCD byte.
constexpr uint8_t DASH_I2C_RECURRING_SIZE = 2;
constexpr uint8_t DASH_I2C_WATCHDOG_MAX_COUNT = 16;  ///< pump() calls without activity before bus recovery

using DashI2CDriver = I2C<DASH_I2C_BITRATE_KBPS, DASH_I2C_PRIORITY_SIZE, DASH_I2C_WATCHDOG_MAX_COUNT>;
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
 * @brief HD44780 LCD over a PCF8574 expander.
 * @details Blocking calls wait until the byte is on the bus. *Queued calls push to
 * the I2C queue and wait only when it is full.
 */
class DashLcd
{
public:
    /**
     * @param lcd_addr 7-bit I2C address of the expander.
     * @param lcd_cols Number of columns.
     * @param lcd_rows Number of rows.
     */
    DashLcd(uint8_t lcd_addr, uint8_t lcd_cols, uint8_t lcd_rows)
        : addr(lcd_addr), cols(lcd_cols), rows(lcd_rows), backlightval(LCD_NOBACKLIGHT)
    {
    }

    /**
     * @brief Run the HD44780 4-bit init sequence and clear the display.
     * @param cols Number of columns.
     * @param lines Number of rows.
     * @param dotsize LCD_5x8DOTS or LCD_5x10DOTS (1-line displays only).
     */
    void begin(uint8_t cols, uint8_t lines, uint8_t dotsize = LCD_5x8DOTS);
    /** @brief begin() with the constructor's size. */
    void init();
    /** @brief Turn the backlight on. */
    void backlight();
    /** @brief Clear the display and home the cursor. */
    void clear();
    /**
     * @brief Load a custom glyph. Leaves the cursor at (0,0).
     * @param location CGRAM slot 0-7.
     * @param charmap 8 bytes in PROGMEM.
     */
    void createChar(uint8_t location, const uint8_t *charmap);
    /** @brief Move the cursor. Row is clamped to the display. */
    void setCursor(uint8_t col, uint8_t row);
    /** @brief Print a RAM string. @return Characters written. */
    size_t print(const char *str);
    /** @brief Print an F() string. @return Characters written. */
    size_t print(const __FlashStringHelper *str);
    /** @brief Print a number without leading zeros. @param base 10 or 16. @return Characters written. */
    size_t print(uint16_t value, uint8_t base);
    /** @brief Write one character or glyph slot. @return 1. */
    size_t write(uint8_t value);

    /** @brief Queued setCursor(). @return true. */
    bool setCursorQueued(uint8_t col, uint8_t row);
    /** @brief Queued print(). @return Characters queued. */
    size_t printQueued(const char *str);
    /** @brief Queued print() of an F() string. @return Characters queued. */
    size_t printQueued(const __FlashStringHelper *str);
    /** @brief Queued print() of a number. @return Characters queued. */
    size_t printQueued(uint16_t value, uint8_t base);
    /** @brief Queued write(). @return true. */
    bool writeQueued(uint8_t value);

    /** @brief Start a dry run: *Queued calls only update a CRC, nothing is sent. */
    void beginDryRun()
    {
        dryRun = true;
        dryCrc = 0xFFFF;
    }
    /** @brief End the dry run. @return CRC of everything queued since beginDryRun(). */
    uint16_t endDryRun()
    {
        dryRun = false;
        return dryCrc;
    }

private:
    void initPriv();
    void sendBlocking(uint8_t value, uint8_t mode);
    void writeNibbleBlocking(uint8_t nibble_with_mode);
    void expanderWriteBlocking(uint8_t data);
    bool sendQueued(uint8_t value, uint8_t mode);
    void fillSend(uint8_t *out, uint8_t value, uint8_t mode) const;
    void pushWait(const uint8_t *data, uint8_t len);

    uint8_t addr;
    uint8_t cols;
    uint8_t rows;
    uint8_t displayfunction = 0;
    uint8_t displaycontrol = 0;
    uint8_t displaymode = 0;
    uint8_t numlines = 0;
    uint8_t backlightval;

    uint8_t blockingTxBuffer[4] = {};                       ///< Data for the blocking transaction
    uint8_t queuedTxBuffer[DASH_I2C_PRIORITY_SIZE][4] = {}; ///< One slot per queued transaction
    uint8_t queuedTxIndex = 0;
    bool dryRun = false;
    uint16_t dryCrc = 0;
    static constexpr uint8_t QUEUED_TX_MASK = DASH_I2C_PRIORITY_SIZE - 1;
};

using LiquidCrystal_I2C = DashLcd;

#include "Dash_I2C.tpp"

#endif // DASH_I2C_HPP
