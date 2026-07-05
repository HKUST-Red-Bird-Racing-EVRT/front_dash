/**
 * @file main.cpp
 * @brief This file contains the main functions for DASH of gen6 car
 * Pages are switched via rotary encoder (INT0_vect ISR).
 */

#include <Arduino.h>
#include <mcp2515.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include "pinMap.h"
#include "DashState.hpp"
#include "Page.hpp"
#include "Structs.h"
#include "I2C.hpp"
#include "namespace.h"

can_frame rx_frame;
int message_count = 0;
uint8_t pot_buffer[8] = {0}; // Buffer for merged POT data (0x730 + 0x750)
bool pot_730_ready = false;
bool pot_750_ready = false;
bool hasStarted = false;


uint16_t carstate = 0;

LiquidCrystal_I2C lcd(0x27, 20, 4);

// Page instances
DashState dashState;
DriverPage driverPage(lcd, dashState);
VCUPage vcuPage(lcd, dashState);
BMSPage bmsPage(lcd, dashState);
DefaultPage defaultPage(lcd, dashState);



// Define pin assignment using actual Arduino pin numbers
// SPI Pins for CAN Controllers (MCP2515)
#define NUM_MCP 2		///< @brief Number of MCP2515 controllers
#define CAN0_CS PIN_PB2 ///< @brief Chip Select pin for CAN Controller 0 (PB2/SS)
#define CAN1_CS PIN_PB1 ///< @brief Chip Select pin for CAN Controller 1 (PB1)

// Rotary Encoder Pins
#define PIN_ENC_A PIN_PD2 ///< @brief Pin for Rotary Encoder A output (PC0 - Analog 0)
#define PIN_ENC_B PIN_PC1 ///< @brief Pin for Rotary Encoder B output (PC1 - Analog 1)

#define PIN_SHIFT_ENC_B PC1 ///< @brief Amount to shift for read

constexpr int8_t NUM_PAGES = 4;

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
// update ticks
uint32_t lastLcdTick = 0;
uint32_t lastCanReadTick = 0;
int8_t write_counter = 0;
MCP2515 cans[NUM_MCP] = {can_vcu, can_ssru};



byte num1_inverted[8] = {
	0b11011, 
	0b10011,
	0b11011, 
	0b11011, 
	0b11011, 
	0b11011, 
	0b10001, 
	0b11111 };
byte num2_inverted[8] = { 
	0b10001, 
	0b11101, 
	0b11101, 
	0b11011, 
	0b10111, 
	0b10111, 
	0b10000, 
	0b11111 };
byte num3_inverted[8] = { 
	0b10001, 
	0b11101, 
	0b11101, 
	0b11001, 
	0b11101, 
	0b11101, 
	0b10001, 
	0b11111 };
byte num4_inverted[8] = { 
	0b11101, 
	0b11001, 
	0b10101, 
	0b10000, 
	0b11101, 
	0b11101, 
	0b11101, 
	0b11111 };
	constexpr byte CHAR_LOCKED = 0;
    constexpr byte CHAR_DEG = 1;
	constexpr byte CHAR_PWR = 2;
    constexpr byte PAGE_INDICATOR_1 = 3;
    constexpr byte PAGE_INDICATOR_2 = 4;
    constexpr byte PAGE_INDICATOR_3 = 5;
    constexpr byte PAGE_INDICATOR_4 = 6;


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
	byte byte_pwr[8] = {
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111
    };
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


void drawPageIndicators(int currentPage) {
    lcd.setCursor(19, 0);
    if (currentPage == 0) lcd.write(PAGE_INDICATOR_1); // Inverted 1
    else lcd.print("1");                      // Normal 1
    lcd.setCursor(19, 1);
    if (currentPage == 1) lcd.write(PAGE_INDICATOR_2); // Inverted 2
    else lcd.print("2");                      // Normal 2
    lcd.setCursor(19, 2);
    if (currentPage == 2) lcd.write(PAGE_INDICATOR_3); // Inverted 3
    else lcd.print("3");                      // Normal 3
    lcd.setCursor(19, 3);
    if (currentPage == 3) lcd.write(PAGE_INDICATOR_4); // Inverted 4
    else lcd.print("4");                      // Normal 4
}



volatile uint8_t encoder_count = 0;
volatile bool encoder_changed = false;

ISR(INT0_vect)
{
	if (PINC & (1 << PIN_SHIFT_ENC_B))
	{ // clockwise
		++encoder_count;
	}
	else
	{ // anticlockwise
		--encoder_count;
	}
	encoder_changed = true;
}

Page* pages[] = {
	&driverPage,
    &vcuPage,
    &bmsPage,
	&defaultPage,
};
constexpr uint8_t PAGE_COUNT = sizeof(pages) / sizeof(pages[0]);
uint8_t currentPageIndex = 0;
Page* currentPage = pages[currentPageIndex];
//Page* DEFPage = pages[3];


void setup()
{
	pinMode(PIN_ENC_A, INPUT_PULLUP);
	pinMode(PIN_ENC_B, INPUT);

	cli();
	EICRA |= (1 << ISC01);
	EICRA &= ~(1 << ISC00);
	EIMSK |= (1 << INT0);
	sei();

	randomSeed(analogRead(GPIO_1)); // Seed random number generator with noise from an unconnected analog pin for better randomness
	Serial.begin(115200);
	lcd.begin(20, 4);
	lcd.clear();
	lcd.init();
	lcd.backlight();
	lcd.setCursor(0, 0);
	lcd.print("Dash Init ");
	lcd.createChar(PAGE_INDICATOR_1, num1_inverted);
    lcd.createChar(PAGE_INDICATOR_2, num2_inverted);
    lcd.createChar(PAGE_INDICATOR_3, num3_inverted);
    lcd.createChar(PAGE_INDICATOR_4, num4_inverted);
	lcd.createChar(CHAR_LOCKED, byte_char_locked);
	lcd.createChar(CHAR_PWR,byte_pwr);
	lcd.createChar(CHAR_DEG, degCelsius);
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
	lcd.clear();
	currentPage->setup();
}


const unsigned long PAGE_SWITCH_INTERVAL = 5000;
long lastupdate = 0;
void loop()
{
	if (millis() - lastupdate >= PAGE_SWITCH_INTERVAL)
	{
		lastupdate = millis();
		currentPageIndex = (currentPageIndex + 1 + PAGE_COUNT) % PAGE_COUNT;
		currentPage = pages[currentPageIndex];
		lcd.clear();
		currentPage->setup();
		currentPage->update();
		drawPageIndicators(currentPageIndex);
	}

	if (encoder_changed) {
        currentPageIndex = (currentPageIndex + encoder_count + PAGE_COUNT) % PAGE_COUNT;
        currentPage = pages[currentPageIndex];
		lcd.clear();
        currentPage->setup();
        encoder_changed = false;
        encoder_count = 0;
		drawPageIndicators(currentPageIndex);
    }
	hasStarted = (car.pedal.status.bits.car_status == CarStatus::Drive);
	MCP2515::ERROR read_state = can_vcu.readMessage(&rx_frame);
	if (read_state == MCP2515::ERROR_OK)
	{
		uint8_t output = 0b10100000;
		switch (rx_frame.can_id) // 20 20 200 ms counttime
		{
		case 0x700: // vcu pedals
			car.pedal.status.byte = rx_frame.data[5];
			car.pedal.faults.byte = rx_frame.data[6];
			carstate = (rx_frame.data[5]);
			break;
		case 0x701: // vcu motors
			torque_val = (rx_frame.data[1] << 8) | rx_frame.data[0];
			motor_rpm = (rx_frame.data[3] << 8) | rx_frame.data[2];
			if (hasStarted)
			{
				motor_warn |= ((rx_frame.data[5] << 8) | rx_frame.data[4]);
				motor_error |= ((rx_frame.data[7] << 8) | rx_frame.data[6]);
			}
			output += 1;
			break;
		case 0x710: // vcu bms
			memcpy(bms.raw_data, rx_frame.data, 8);
			bms.status = BmsStatus::Started;
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
	}
	if (millis() - lastLcdTick >= lcd_update::update_interval_ms)
	{
		odometer_integral += abs(motor_rpm);
		lastLcdTick += lcd_update::update_interval_ms;
		currentPage->update();
	}
}
