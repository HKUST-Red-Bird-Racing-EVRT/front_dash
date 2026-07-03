/**
 * @file Page.tpp
 * @author ChiHo
 * @brief Implementation of Page abstract base class and derived classes
 * @version 1.0
 * @date 2026-05-28
 * @see Page.hpp
 */
#include "namespace.h"
#include <Arduino.h>
#include "Page.hpp"
#include <LiquidCrystal_I2C.h>
#include <mcp2515.h>



// Current page pointer
extern Page* currentPage;


int16_t torque_val, motor_rpm = 0;
uint16_t motor_warn, motor_error = 0; // Variable to store motor stuff
uint32_t odometer_integral = 0; // Variable to store integral of RPM for odometer calculation

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
   lcd.setCursor(0, 0);
	uint8_t speed = abs(motor_rpm) / rpm_calc::RPM_TO_KMH_DIVISOR;
	char speed_str[5];
	speed_str[4] = '\0';
	for (int i = 3; i >= 1; --i)
	{
	    speed_str[i] = (speed % 10) + '0';
		speed /= 10;
	}
	speed_str[0] = (motor_rpm >= 0) ? '+' : '-';
	lcd.print(speed_str);
    lcd.setCursor(16, 2);
	lcd.print("00");
	lcd.setCursor(16, 2);
	lcd.print(motor_warn, HEX);
	lcd.setCursor(18, 2);
	lcd.print("00");
	lcd.setCursor(18, 2);
	lcd.print(motor_error, HEX);
			// drive mode
	lcd.setCursor(9, 0);
	switch (car.pedal.status.bits.car_status)
	{
	    case CarStatus::Init:
		{
		    lcd.write(char_locked);
			break;
		}
	    case CarStatus::Startin:
		{
			lcd.print("S");
			break;
		}
		case CarStatus::Bussin:
		{
			lcd.print("B");
			break;
		}
		case CarStatus::Drive:
		{
			lcd.print("D");
			break;
		}
}
    lcd.setCursor(11, 0);
	uint16_t rpm = (uint32_t)abs(motor_rpm) * rpm_calc::MAX_MOTOR_RPM / rpm_calc::MAX_MOTOR_RPM_READING;
	char rpm_str[6];
	rpm_str[5] = '\0';
	for (int i = 4; i >= 1; --i)
	{
		rpm_str[i] = (rpm % 10) + '0';
		rpm /= 10;
	}
	if (motor_rpm >= 0)
	{
		rpm_str[0] = '+';
	}
	else
	{
		rpm_str[0] = '-';
	}
	lcd.print(rpm_str);
}


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
