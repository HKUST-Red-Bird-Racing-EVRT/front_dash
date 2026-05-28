/**
 * @file Page.tpp
 * @author Red Bird Racing
 * @brief Implementation of Page abstract base class and derived classes
 * @version 1.0
 * @date 2026-05-23
 * @see Page.hpp
 */

#include "Page.hpp"
#include <LiquidCrystal_I2C.h>



#define char_locked 0
#define char_deg 1

// Custom Char
byte degCelsius[8] = { // degree celsius char
	0b01000,
	0b10100,
	0b01000,
	0b00011,
	0b00100,
	0b00100,
	0b00100,
	0b00011};

byte byte_char_locked[8] = {
	0b01110,
	0b10001,
	0b10001,
	0b11111,
	0b11011,
	0b11011,
	0b11011,
	0b11111};

// === Page Abstract Base Class ===
// Pure virtual methods must be implemented by derived classes.

// === DashboardPage Implementation ===

/**
 * @brief Constructor for DashboardPage.
 * Initializes the dashboard with reference to LCD display.
 * @param led reference to LiquidCrystal_I2C display object.
 */
DashboardPage::DashboardPage(LiquidCrystal_I2C& lcd, DashState& state) : lcd(lcd)
{
    
}

/**
 * @brief Setup the dashboard page.
 * Clears the LCD and initializes the dashboard layout.
 */
void DashboardPage::setup()
{
    lcd.clear();
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
    lcd.createChar(char_deg, degCelsius);
}

/**
 * @brief Update the dashboard page.
 * Called repeatedly to refresh vehicle data on display.
 * TODO: Implement with actual vehicle state data
 * - Update speed
 * - Update RPM
 * - Update throttle position
 * - Update odometer
 * - Update status indicators
 */
void DashboardPage::update()
{
    
}
