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

can_frame rx_frame;
int message_count = 0;
uint8_t pot_buffer[8] = {0}; // Buffer for merged POT data (0x730 + 0x750)
bool pot_730_ready = false;
bool pot_750_ready = false;

int16_t torque_val, motor_rpm = 0;
uint16_t motor_warn, motor_error = 0; // Variable to store motor stuff

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
void setup()
{
	Serial.begin(115200);
	lcd.begin(20, 4);
	lcd.clear();
	lcd.init();
	lcd.backlight();
	lcd.setCursor(0, 0);
	lcd.print("Dash Init ...");
	lcd.setCursor(0, 1);
	lcd.print("LCD ... OK");
	lcd.setCursor(0, 2);
	lcd.print("CAN ... ");
	for (int i = 0; i < NUM_MCP; ++i)
	{
		cans[i].reset();
		cans[i].setBitrate(CAN_500KBPS, MCP_20MHZ);
		cans[i].setNormalMode();
	}
	lcd.print("OK");
	lcd.setCursor(0, 3);
	lcd.print("Serial ... ");
	while (!Serial)
	{
		;
	}
	lcd.print("OK");
	delay(2000);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Dash Ready!");
	delay(2000);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Throttle: ");
	lcd.setCursor(0, 1);
	lcd.print("Motor RPM: ");
	lcd.setCursor(15, 1);
	lcd.print("RPM");
	lcd.setCursor(0, 2);
	lcd.print("Motor Warn: 0x");
	lcd.setCursor(0, 3);
	lcd.print("Motor Error: 0x");
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
	if (millis() - lastLcdTick >= 75)
	{
		static uint8_t lcd_update_state = 0;
		switch (lcd_update_state)
		{
		case 0:
			lcd.setCursor(10, 0);
			if (torque_val < 0)
			{
				lcd.print("-   ");
			}
			else
			{
				lcd.print("+   ");
			}
			lcd.setCursor(11, 0);
			lcd.print(abs(torque_val) / 327);
			lcd.setCursor(14, 0);
			lcd.print("%");
			break;
		case 1:
			lcd.setCursor(11, 1);
			lcd.print((int32_t)motor_rpm * 6000 / 32767);
			break;
		case 2:
			lcd.setCursor(14, 2);
			lcd.print("00");
			lcd.setCursor(14, 2);
			lcd.print(motor_warn, HEX);
			break;
		case 3:
			lcd.setCursor(15, 3);
			lcd.print("00");
			lcd.setCursor(15, 3);
			lcd.print(motor_error, HEX);
			break;
		}
		lcd_update_state = (lcd_update_state + 1) % 3;
		lastLcdTick = millis();
	}
}