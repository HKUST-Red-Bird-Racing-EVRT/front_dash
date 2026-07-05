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
#include "DashState.hpp"





// Current page pointer
extern Page* currentPage;


int16_t torque_val, motor_rpm = 0;
uint16_t motor_warn, motor_error = 0; // Variable to store motor stuff
uint32_t odometer_integral = 0; // Variable to store integral of RPM for odometer calculation
constexpr uint8_t ODO_NUM_DIGITS = 6;
constexpr uint8_t ODO_DECIMAL_PLACES = rpm_calc::NUM_DECIMAL_PLACE;
constexpr uint8_t ODO_STR_LENGTH = ODO_NUM_DIGITS + 1 + 1; // digits + decimal point + null terminator
constexpr uint8_t ODO_POS_OFFSET = ODO_STR_LENGTH + 1 + 2; // odometer string + " km" right align padding
constexpr uint8_t STR_START_POS = 21 - ODO_POS_OFFSET;	   // right align the odometer reading
extern const byte CHAR_LOCKED;
extern const byte CHAR_DEG;
extern const byte CHAR_PWR;
void DriverPage::updatepwr(){
	lcd.setCursor(9,1);
	for (int bmsval=0; bmsval<10;bmsval++){
	    if (telembms.bms_data[2]/10 == bmsval){
			for(int j=0; j<bmsval;j++){
				lcd.write(CHAR_PWR);
				lcd.setCursor(9+j,1);
			}
	}
    }
	lcd.setCursor(14,3);
	if(telembms.bms_data[2]/100 != 0){
		lcd.print("0");
		lcd.setCursor(15,3);
		lcd.write(telembms.bms_data[2]);
	}
}

void DriverPage::updaterpm(){
	lcd.setCursor(11, 0);
	uint16_t rpm = (uint32_t)abs(motor_rpm) * rpm_calc::MAX_MOTOR_RPM / rpm_calc::MAX_MOTOR_RPM_READING;    // motor rpm
	char rpm_str[6];
	rpm_str[5] = '\0';
	for (uint8_t i = 4; i >= 1; --i)
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

void DriverPage::update_car_spd(){
	lcd.setCursor(4,0);
	uint8_t speed = abs(motor_rpm) / rpm_calc::RPM_TO_KMH_DIVISOR;
	char speed_str[5];
	speed_str[4] = '\0';
	for (uint8_t i = 3; i >= 1; --i)
	{
		speed_str[i] = (speed % 10) + '0';
		speed /= 10;
	}
	speed_str[0] = (motor_rpm >= 0) ? '+' : '-';
	lcd.print(speed_str);
}

void DriverPage::updateodo(){
	lcd.setCursor(0, 3);
	uint32_t odometer = odometer_integral / rpm_calc::RPM_INTEGRAL_TO_KM_DIVISOR;
	char odometer_str[ODO_STR_LENGTH];
	odometer_str[ODO_STR_LENGTH - 1] = '\0';
	for (int8_t i = ODO_STR_LENGTH - 2; i >= 0; --i)
	{
		if (i == ODO_NUM_DIGITS - ODO_DECIMAL_PLACES)
		{
			odometer_str[i] = '.';
		}
		else
		{
			odometer_str[i] = (odometer % 10) + '0';
			odometer /= 10;
		}
	}
	lcd.print(odometer_str);
}


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
    lcd.setCursor(0, 0);
	lcd.print("kmh:");
	lcd.setCursor(0, 1);
	lcd.print("rpm:");
	lcd.setCursor(0, 2);
	lcd.setCursor(7, 3);
	lcd.print("km");
	lcd.setCursor(13, 3);
	lcd.print("B");
	lcd.setCursor(17,3);
	lcd.print("%");
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
	updatepwr();
	lcd.setCursor(9, 0);
    switch (car.pedal.status.bits.car_status)
	{
	    case CarStatus::Init:
		{
		    lcd.write(CHAR_LOCKED);
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
			break;}
	}
	updateodo();
	update_car_spd();
	updaterpm();
	updatepwr();
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
    lcd.setCursor(0, 1);
    lcd.print("ERROR");
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
    lcd.setCursor(16, 2);
	lcd.print("00");
	lcd.setCursor(16, 2);
	lcd.print(motor_warn, HEX);
	lcd.setCursor(18, 2);
	lcd.print("00");
	lcd.setCursor(18, 2);
	lcd.print(motor_error, HEX);
	char vcu_text[5]="    ";
	vcu_text[3] = (car.pedal.faults.byte % 16 > 9) ? (car.pedal.faults.byte % 16 - 10 + 'A') : (car.pedal.faults.byte % 16 + '0');
	vcu_text[2] = (car.pedal.faults.byte/16 % 16 > 9) ? (car.pedal.faults.byte/16 % 16 - 10 + 'A') : (car.pedal.faults.byte/16 % 16 + '0');
	vcu_text[1] = (car.pedal.status.byte % 16 > 9) ? (car.pedal.status.byte % 16 - 10 + 'A') : (car.pedal.status.byte % 16 + '0');
	vcu_text[0] = (car.pedal.status.byte/16 % 16 > 9) ? (car.pedal.status.byte/16 % 16 - 10 + 'A') : (car.pedal.status.byte/16 % 16 + '0');
	lcd.setCursor(0,1);
	lcd.print(vcu_text);
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

    lcd.clear();
	lcd.setCursor(7,0);
	lcd.print("BMS");
    lcd.setCursor(0, 1);
    lcd.print("ERROR:");
    lcd.setCursor(0, 3);
    lcd.print("MAX:");
	lcd.setCursor(7,3);
	lcd.print("V");
	lcd.setCursor(9,3);
	lcd.print("MIN:");
	lcd.setCursor(16,3);
	lcd.print("V");
    lcd.setCursor(0, 3);
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
	char adc[17] = "                ";
	adc[16] = '\0';
	uint16_t values[4] = {car.pedal.apps_5v, car.pedal.apps_3v3, car.pedal.brake, car.pedal.hall_sensor};
	for (uint8_t i = 0; i < 4; ++i)
	{
		for (uint8_t j = 3; j > 0; --j)
		{
			adc[4 * i + j - 1] = (values[i] % 16 > 9) ? (values[i] % 16 - 10 + 'A') : (values[i] % 16 + '0');
			values[i] /= 16;
			if (!values[i])
				break;
		}
	}
    // temp
	lcd.setCursor(0, 2);
	lcd.print(adc);

}

// ============================================================================
// === defaultPage Implementation ===
// ============================================================================

/**
 * @brief Constructor for defaultPage.
 */
DefaultPage::DefaultPage(LiquidCrystal_I2C& lcd, DashState& state)
    : lcd(lcd), state(state)
{
}

/**
 * @brief Setup the default page.
 */
void DefaultPage::setup()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("BRAKE:");
    lcd.setCursor(0, 1);
    lcd.print("BUTTON:");
    lcd.setCursor(0, 2);
    lcd.print("HV:");
}

/**
 * @brief Update the reserved page.
 * Placeholder for future page updates.
 */
void DefaultPage::update()
{
	if(car.pedal.faults.bits.fault_active){
		lcd.setCursor(6,2);
		lcd.print("Pedal/Brake error");
	}
	else{lcd.setCursor(6,2);lcd.print("no Error");}
	if(telembms.bms_data[6] ==car.pedal.status.bits.hv_ready){
		lcd.setCursor(3,2);
		lcd.print("Y");
	}
	else{
		lcd.setCursor(3,2);
		lcd.print("N");
	}
}
