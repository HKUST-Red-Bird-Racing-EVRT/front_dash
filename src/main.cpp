/**
 * @file main.cpp
 * @brief This file contains the main functions for DASH of gen6 car
 * @note This code is intended for testing purposes only.
 */

#include <Arduino.h>
#include <mcp2515.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include "pinMap.h"
#include "CarState.hpp"

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
#define char_locked 0

can_frame rx_frame;
int message_count = 0;
uint8_t pot_buffer[8] = {0}; // Buffer for merged POT data (0x730 + 0x750)
bool pot_730_ready = false;
bool pot_750_ready = false;

int16_t torque_val, motor_rpm = 0;
uint16_t motor_warn, motor_error = 0; // Variable to store motor stuff
uint32_t odometer_integral = 0;		  // Variable to store integral of RPM for odometer calculation

namespace lcd_update
{
	constexpr uint8_t cycle_rate = 4; // lcd hz
	constexpr uint8_t update_items = 5;
	constexpr uint8_t update_count = cycle_rate * update_items;

	static_assert((1000 / update_count) * update_count == 1000, "update_count must be a factor of 1000 to avoid truncation.");

	constexpr uint8_t update_interval_ms = 1000 / update_count;
}
namespace rpm_calc
{
	constexpr uint32_t ipow(uint32_t base, unsigned exp)
	{
		return exp == 0 ? 1u : base * ipow(base, exp - 1);
	}

	constexpr uint8_t GEAR_RATIO_NUMERATOR = 50;   /**< Gear ratio numerator of the drivetrain. */
	constexpr uint8_t GEAR_RATIO_DENOMINATOR = 13; /**< Gear ratio denominator of the drivetrain. */

	// === Calculation for RPM threshold ===
	constexpr uint16_t WHEEL_DIAMETER_MM = 455;		  /**< Wheel diameter in millimeters (actual recorded). */
	constexpr uint16_t MAX_MOTOR_RPM = 7000;		  /**< Maximum motor RPM. */
	constexpr uint16_t MAX_MOTOR_RPM_READING = 32767; /**< Maximum motor RPM reading from CAN (2^15 - 1 for signed 16-bit). */
	constexpr uint16_t MAX_TORQUE_VAL = 32767;		  /**< Maximum torque value for motor controller. */

	constexpr uint32_t MM_PER_KM = (uint32_t)1000 * 1000;					 /**< Millimeters per kilometer. */
	constexpr uint8_t MINUTES_PER_HOUR = 60;								 /**< Minutes per hour. */
	constexpr uint16_t SECONDS_PER_HOUR = 3600;								 /**< Seconds per hour. */
	constexpr uint8_t NUM_DECIMAL_PLACE = 5;								 /**< Number of decimal places (added 4 to become meter) */
	constexpr uint32_t FIXED_POINT_MULTIPLIER = ipow(10, NUM_DECIMAL_PLACE); /**< Multiplier to adjust for decimal places */
	constexpr double PI_ = 3.1415926535897932384626433832795;				 /**< Value of pi, unnamed to avoid clashing with Arduino.h's definition. */

	/** Divide by this constant to get  */
	constexpr uint16_t RPM_TO_KMH_DIVISOR = (double)MAX_TORQUE_VAL / MAX_MOTOR_RPM /
												WHEEL_DIAMETER_MM / PI_ * MM_PER_KM / MINUTES_PER_HOUR / GEAR_RATIO_DENOMINATOR * GEAR_RATIO_NUMERATOR +
											0.5f;
	constexpr uint32_t RPM_INTEGRAL_TO_KM_DIVISOR = (double)MAX_TORQUE_VAL / MAX_MOTOR_RPM /
														WHEEL_DIAMETER_MM / PI_ * MM_PER_KM / MINUTES_PER_HOUR / GEAR_RATIO_DENOMINATOR * GEAR_RATIO_NUMERATOR *
														lcd_update::update_count * SECONDS_PER_HOUR / FIXED_POINT_MULTIPLIER +
													0.5f;
}

// Define pin assignment using actual Arduino pin numbers
// SPI Pins for CAN Controllers (MCP2515)
#define NUM_MCP 2		///< @brief Number of MCP2515 controllers
#define CAN0_CS PIN_PB2 ///< @brief Chip Select pin for CAN Controller 0 (PB2/SS)
#define CAN1_CS PIN_PB1 ///< @brief Chip Select pin for CAN Controller 1 (PB1)

// Rotary Encoder Pins
// #define ENC_A PIN_PC0 ///< @brief Pin for Rotary Encoder A output (PC0 - Analog 0)
// #define ENC_B PIN_PC1 ///< @brief Pin for Rotary Encoder B output (PC1 - Analog 1)
bool test_mode = true;
int test_encA = 1, test_encB = 0;
int lastStateA, lastStateB;
int lastChange = 0;

// I2C Communication Pins
#define SDA_PIN PIN_PC4 ///< @brief SDA pin for I2C communication (PC4 - Analog 4)
#define SCL_PIN PIN_PC5 ///< @brief SCL pin for I2C communication (PC5 - Analog 5)

// General Purpose Input/Output (GPIO) Pins
#define GPIO_1 PIN_PC2 ///< @brief General Purpose I/O Pin 1 (PC2 - Analog 2)
#define GPIO_2 PIN_PC3 ///< @brief General Purpose I/O Pin 2 (PC3 - Analog 3)
#define GPIO_3 PIN_PD7 ///< @brief General Purpose I/O Pin 3 (PD7 - Digital 7)
#define GPIO_4 PIN_PD2 ///< @brief General Purpose I/O Pin 4 (PD2 - Digital 2)
#define GPIO_5 PIN_PD3 ///< @brief General Purpose I/O Pin 5 (PD3 - Digital 3)
#define GPIO_6 PIN_PD4 ///< @brief General Purpose I/O Pin 6 (PD4 - Digital 4)
#define GPIO_7 PIN_PD5 ///< @brief General Purpose I/O Pin 7 (PD5 - Digital 5)
#define GPIO_8 PIN_PD6 ///< @brief General Purpose I/O Pin 8 (PD6 - Digital 6)

/**
 * @brief An array containing all the GPIO pins to be tested.
 */
const int gpioPins[] = {
	GPIO_1, GPIO_2, GPIO_3, GPIO_4,
	GPIO_5, GPIO_6, GPIO_7, GPIO_8};
const int numGpioPins = sizeof(gpioPins) / sizeof(gpioPins[0]);
/**
 * @brief MCP2515 functions and PINS
 */
static constexpr canid_t VCU_READ = 0xFF; /**< Motor read CAN ID */
MCP2515 can_vcu(CAN0_CS);
MCP2515 can_ssru(CAN1_CS);

/** edit later for implementation of CAN of dash
 *
 * MCP2515 mcp2515_motor(CS_CAN_MOTOR);
 *MCP2515 mcp2515_BMS(CS_CAN_BMS);
 *MCP2515 mcp2515_();
 *#define mcp2515_motor mcp2515_DL
 *#define mcp2515_BMS mcp2515_DL
 *constexpr uint8_t NUM_MCP = 3;
 *MCP2515 MCPS[NUM_MCP] = {mcp2515_motor, mcp2515_BMS, mcp2515_DL};
 *
 *
 *
 */
/**
 * @brief Arduino setup function.
 * @details It initializes serial communication
 */
LiquidCrystal_I2C lcd(0x27, 20, 4);
// update ticks
uint32_t lastLcdTick = 0;
uint32_t lastCanReadTick = 0;
uint8_t write_counter = 0;
MCP2515 cans[NUM_MCP] = {can_vcu, can_ssru};

CarState car;

void setup()
{
	randomSeed(analogRead(GPIO_1)); // Seed random number generator with noise from an unconnected analog pin for better randomness
	Serial.begin(115200);
	lcd.begin(20, 4);
	lcd.clear();
	lcd.init();
	lcd.backlight();
	lcd.setCursor(0, 0);
	lcd.print("Dash Init ");
	lcd.createChar(char_locked, byte_char_locked);
	for (int i = 0; i < 10; ++i)
	{
		delay(random(20, 100));
		lcd.print(".");
	}
	lcd.setCursor(0, 1);
	delay(300);
	lcd.print("LCD ");
	for (int i = 0; i < 13; ++i)
	{
		delay(random(20, 50));
		lcd.print(".");
	}
	delay(250);
	lcd.print(" OK");
	lcd.setCursor(0, 2);
	delay(random(50, 150));
	lcd.print("CAN ");
	for (int i = 0; i < NUM_MCP; ++i)
	{
		cans[i].reset();
		delay(random(20, 50));
		lcd.print(".");
		delay(random(20, 50));
		lcd.print(".");
		cans[i].setBitrate(CAN_500KBPS, MCP_20MHZ);
		delay(random(20, 50));
		lcd.print(".");
		delay(random(20, 50));
		lcd.print(".");
		cans[i].setNormalMode();
		delay(random(20, 50));
		lcd.print(".");
		delay(random(20, 50));
		lcd.print(".");
	}
	delay(random(20, 50));
	lcd.print(".");
	delay(random(50, 200));
	lcd.print(" OK");
	lcd.setCursor(0, 3);
	delay(500);
	lcd.setCursor(0, 0);
	lcd.print("LCD ............. OK");
	lcd.setCursor(0, 1);
	lcd.print("CAN ............. OK");
	lcd.setCursor(0, 2);
	lcd.print("Serial              ");
	lcd.setCursor(7, 2);
	for (int i = 0; i < 10; ++i)
	{
		delay(random(10, 30));
		lcd.print(".");
	}
	while (!Serial)
	{
		;
	}
	delay(random(100, 200));
	lcd.print(" OK");
	delay(random(100, 200));
	lcd.setCursor(0, 3);
	lcd.print("Dash Ready! Race!");
	delay(2000);
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
}
void loop()
{
	/*    int encA, encB;

	if (test_mode) {
		encA = test_encA;  // test values
		encB = test_encB;
		} else {
			encA = digitalRead(PIN_PC0);  // actual pins
			encB = digitalRead(PIN_PC1);
		}
		lcd.setCursor(0, 3);
		lcd.print(millis());
		*/
	MCP2515::ERROR read_state = can_vcu.readMessage(&rx_frame);
	if (read_state == MCP2515::ERROR_OK)
	{
		// if (test_encA==1){

		uint8_t output = 0b10100000;
		switch (rx_frame.can_id) // 20 20 200 ms counttime
		{
		case 0x700: // vcu pedals
			car.pedal.status.byte = rx_frame.data[5];
			car.pedal.faults.byte = rx_frame.data[6];
			break;
		case 0x701: // vcu motors
			torque_val = (rx_frame.data[1] << 8) | rx_frame.data[0];
			motor_rpm = (rx_frame.data[3] << 8) | rx_frame.data[2];
			motor_warn = (rx_frame.data[5] << 8) | rx_frame.data[4];
			motor_error = (rx_frame.data[7] << 8) | rx_frame.data[6];
			output += 1;
			break;
		case 0x710: // vcu bms
			output += 2;
			break;
		case 0x730: // front pot
			memcpy(&pot_buffer[0], rx_frame.data, 4);
			pot_730_ready = true;
			break;
		case 0x740: // front
			output += 5;
			break;
		case 0x750: // rear pot
			memcpy(&pot_buffer[4], rx_frame.data, 4);
			pot_750_ready = true;
			break;
		case 0x760: // cooling
			output += 8;
			break;
		default:
			output += 15; // not supposed to happen
		}
		if (pot_730_ready && pot_750_ready)
		{
			output += 3;
			Serial.write(output);
			Serial.write(pot_buffer, 8);
			pot_730_ready = false;
			pot_750_ready = false;
		}
		else
		{
			Serial.write(output);
			Serial.write(rx_frame.data, 8);
		}
		/*

		lcd.setCursor(0, 0);
		lcd.print("ID:        ");
		lcd.setCursor(3, 0);
		lcd.print(rx_frame.can_id, HEX);
		lcd.setCursor(0, 2);
		for (uint8_t i = 0; i < rx_frame.can_dlc && i < 8; i++){
			if (rx_frame.data[i] < 0x10)
			lcd.print("0");
			lcd.print(rx_frame.data[i], HEX);
			}}
			//    else if (test_encB==1){
				lcd.clear();
				lcd.setCursor(1,1);
				lcd.print("encoder works");
				lcd.setCursor(0, 0);
				lcd.print("RPM: ");
				lcd.print(motor_rpm,HEX);
				delay(100);
				lcd.clear();
				}
				//**  if (test_mode) {
					//
					//    if (millis() - lastChange >= 5000) {
						//      if (test_encA == 1 && test_encB == 0) {
							//        test_encA = 0;
							//      test_encB = 1;  // Switch to second path
			//} else {
			  //  test_encA = 1;
			  //test_encB = 0;  // Switch to first path
			  //}
			  //lastChange = millis();}
			  */
	}
	if (millis() - lastLcdTick >= lcd_update::update_interval_ms)
	{
		static uint8_t lcd_update_state = 0;
		odometer_integral += abs(motor_rpm);
		switch (lcd_update_state)
		{
		case 0:
		{
			// speed
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
			break;
		}
		case 1:
		{
			// motor rpm
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
			break;
		}
		case 2:
		{
			// throttle percentage
			lcd.setCursor(14, 1);
			uint8_t throttle_percent = (uint32_t)(abs(torque_val) + 162) * 100 / 32500;
			char throttle_str[5];
			throttle_str[4] = '\0';
			if (torque_val >= 0)
			{
				throttle_str[0] = '+';
			}
			else
			{
				throttle_str[0] = '-';
			}
			for (int i = 3; i >= 1; --i)
			{
				throttle_str[i] = (throttle_percent % 10) + '0';
				throttle_percent /= 10;
			}
			lcd.print(throttle_str);
			break;
		}
		case 3:
		{
			// motor warn/error
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
			break;
		}
		case 4:
		{
			// odometer
			constexpr uint8_t ODO_NUM_DIGITS = 6;
			constexpr uint8_t ODO_DECIMAL_PLACES = rpm_calc::NUM_DECIMAL_PLACE;
			constexpr uint8_t ODO_STR_LENGTH = ODO_NUM_DIGITS + 1 + 1; // digits + decimal point + null terminator
			constexpr uint8_t ODO_POS_OFFSET = ODO_STR_LENGTH + 1 + 2; // odometer string + " km" right align padding
			constexpr uint8_t STR_START_POS = 21 - ODO_POS_OFFSET;	   // right align the odometer reading
			lcd.setCursor(STR_START_POS, 3);
			uint32_t odometer = odometer_integral / rpm_calc::RPM_INTEGRAL_TO_KM_DIVISOR;
			char odometer_str[ODO_STR_LENGTH];
			odometer_str[ODO_STR_LENGTH - 1] = '\0';
			for (int i = ODO_STR_LENGTH - 2; i >= 0; --i)
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
			break;
		}
		}
		lcd_update_state = (lcd_update_state + 1) % (lcd_update::update_items);
		lastLcdTick = millis();
	}
}