/**
 * @file Page.tpp
 * @author ChiHo
 * @brief Implementation of Page abstract base class and derived classes
 * @version 1.0
 * @date 2026-05-28
 * @see Page.hpp
 */
#include <Arduino.h>
#include "Page.hpp"
#include <LiquidCrystal_I2C.h>
#include <mcp2515.h>

// === Page Abstract Base Class ===
// Pure virtual methods must be implemented by derived classes.

// ============================================================================
// === DriverPage Implementation ===
// ============================================================================

/**
 * @brief Constructor for DriverMenuPage.
 */
DriverPage::DriverPage(LiquidCrystal_I2C& lcd, DashState& state)
    : lcd(lcd), state(state)
{
}

/**
 * @brief Setup the driver menu page.
 * Clears the LCD and initializes the dashboard layout.
 */
void DriverPage::setup()
{
    #define char_locked 0
    #define char_deg 1

	// Custom Char
    byte byte_char_locked[8] = {
        0b01110,
        0b10001,
        0b10001,
        0b11111,
        0b11011,
        0b11011,
        0b11011,
        0b11111
    };

    lcd.setCursor(4, 0);
	lcd.print(" kmh");
	lcd.setCursor(16, 0);
	lcd.print(" rpm");
	lcd.setCursor(0, 1);
	lcd.print("Throttle: ");
	lcd.setCursor(19, 1);
	lcd.print("%");
	lcd.setCursor(0, 2);
	lcd.print("MCU Warn/Err: 0x");
	lcd.setCursor(0, 3);
	lcd.print("Odometer:         km");
    lcd.createChar(char_locked, byte_char_locked);
}

/**
 * @brief Update the driver menu page.
 * Called repeatedly to refresh vehicle data on display.
 * TODO: Implement with actual vehicle state data
 * - Update speed
 * - Update RPM
 * - Update throttle position
 * - Update odometer
 * - Update status indicators
 */
void DriverPage::update()
{
    // TODO: Implement vehicle data display
}

// ============================================================================
// === VCUDebugPage Implementation ===
// ============================================================================

/**
 * @brief Constructor for VCUDebugPage.
 */
VCUPage::VCUPage(LiquidCrystal_I2C& lcd, DashState& state)
    : lcd(lcd), state(state)
{
}

/**
 * @brief Setup the VCU debug page.
 * Displays VCU (Vehicle Control Unit) debug information.
 */
void VCUPage::setup()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("VCU Debug");
    lcd.setCursor(0, 1);
    lcd.print("Motor RPM:");
    lcd.setCursor(0, 2);
    lcd.print("Torque:");
    lcd.setCursor(0, 3);
    lcd.print("Status:");
}

/**
 * @brief Update the VCU debug page.
 * Called repeatedly to refresh VCU debug data on display.
 * TODO: Implement with actual VCU diagnostic data
 * - Display motor RPM values
 * - Display torque values
 * - Display VCU status codes
 * - Display fault information
 */
void VCUPage::update()
{

}

// ============================================================================
// === BMSDebugPage Implementation ===
// ============================================================================

/**
 * @brief Constructor for BMSDebugPage.
 */
BMSPage::BMSPage(LiquidCrystal_I2C& lcd, DashState& state)
    : lcd(lcd), state(state)
{
}

/**
 * @brief Setup the BMS debug page.
 * Displays BMS (Battery Management System) debug information.
 */
void BMSPage::setup()
{
	byte degCelsius[8] = { // degree celsius char
        0b01000,
        0b10100,
        0b01000,
        0b00011,
        0b00100,
        0b00100,
        0b00100,
        0b00011
    };
	lcd.createChar(char_deg, degCelsius);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("BMS Debug");
    lcd.setCursor(0, 1);
    lcd.print("Voltage:");
    lcd.setCursor(0, 2);
    lcd.print("Current:");
    lcd.setCursor(0, 3);
    lcd.print("Status:");

}

/**
 * @brief Update the BMS debug page.
 * Called repeatedly to refresh BMS debug data on display.
 * TODO: Implement with actual BMS diagnostic data
 * - Display battery voltage
 * - Display battery current
 * - Display BMS status
 * - Display cell temperatures
 * - Display fault information
 */
void BMSPage::update()
{
    // TODO: Implement BMS debug data display
}

// ============================================================================
// === ReservedPage Implementation ===
// ============================================================================

/**
 * @brief Constructor for ReservedPage.
 */
ReservedPage::ReservedPage(LiquidCrystal_I2C& lcd, DashState& state)
    : lcd(lcd), state(state)
{
}

/**
 * @brief Setup the reserved page.
 * Placeholder for future page implementation.
 */
void ReservedPage::setup()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Reserved");
    lcd.setCursor(0, 1);
    lcd.print("Page");
    lcd.setCursor(0, 2);
    lcd.print("(Future Use)");
}

/**
 * @brief Update the reserved page.
 * Placeholder for future page updates.
 */
void ReservedPage::update()
{
    // TODO: Implement reserved page functionality
}
