/**
 * @file main.cpp
 * @brief This file contains the main functions for DASH of gen6 car
 * @note This code is intended for testing purposes only.
 */

#include <Arduino.h>
#include <mcp2515.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <SPI.h>

// Custom Char
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


can_frame rx_frame;

// Define pin assignment using actual Arduino pin numbers
// SPI Pins for CAN Controllers (MCP2515)
#define CAN0_CS_PIN PIN_PB2 ///< @brief Chip Select pin for CAN Controller 0 (PB2/SS)
#define CAN1_CS_PIN PIN_PB1 ///< @brief Chip Select pin for CAN Controller 1 (PB1)

// HC-12 Wireless Serial Communication Module Pins
#define HC12_RX_PIN PIN_PD0  ///< @brief Receive pin for HC-12 module (PD0 - Serial RX)
#define HC12_TX_PIN PIN_PD1  ///< @brief Transmit pin for HC-12 module (PD1 - Serial TX)
#define HC12_SET_PIN PIN_PB0 ///< @brief SET pin for HC-12 module (PB0)

// Rotary Encoder Pins
#define ENC_A PIN_PC0 ///< @brief Pin for Rotary Encoder A output (PC0 - Analog 0)
#define ENC_B PIN_PC1 ///< @brief Pin for Rotary Encoder B output (PC1 - Analog 1)
int lastStateA , lastStateB;

// I2C Communication Pins
#define SDA_PIN PIN_PC4 ///< @brief SDA pin for I2C communication (PC4 - Analog 4)
#define SCL_PIN PIN_PC5 ///< @brief SCL pin for I2C communication (PC5 - Analog 5)

// General Purpose Input/Output (GPIO) Pins
#define GPIO_1_PIN PIN_PC2 ///< @brief General Purpose I/O Pin 1 (PC2 - Analog 2)
#define GPIO_2_PIN PIN_PC3 ///< @brief General Purpose I/O Pin 2 (PC3 - Analog 3)
#define GPIO_3_PIN PIN_PD7  ///< @brief General Purpose I/O Pin 3 (PD7 - Digital 7)
#define GPIO_4_PIN PIN_PD2  ///< @brief General Purpose I/O Pin 4 (PD2 - Digital 2)
#define GPIO_5_PIN PIN_PD3  ///< @brief General Purpose I/O Pin 5 (PD3 - Digital 3)
#define GPIO_6_PIN PIN_PD4  ///< @brief General Purpose I/O Pin 6 (PD4 - Digital 4)
#define GPIO_7_PIN PIN_PD5  ///< @brief General Purpose I/O Pin 7 (PD5 - Digital 5)
#define GPIO_8_PIN PIN_PD6  ///< @brief General Purpose I/O Pin 8 (PD6 - Digital 6)

/**
 * @brief An array containing all the GPIO pins to be tested.
 */
const int gpioPins[] = {
	GPIO_1_PIN, GPIO_2_PIN, GPIO_3_PIN, GPIO_4_PIN,
	GPIO_5_PIN, GPIO_6_PIN, GPIO_7_PIN, GPIO_8_PIN};
/**
 * @brief MCP2515 functions and PINS
 */
MCP2515 can_vcu(CAN0_CS_PIN);
MCP2515 can_ssru(CAN1_CS_PIN);
static constexpr canid_t VCU_READ = 0x181; /**< Motor read CAN ID */


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
const int numGpioPins = sizeof(gpioPins) / sizeof(gpioPins[0]);

LiquidCrystal_I2C lcd(0x27, 20, 4);

// update ticks
uint32_t lastLcdTick = 0;
uint32_t lastCanReadTick = 0;

uint8_t write_counter = 0;

void setup()
{
	// Initialize Serial communication at 9600 baud rate. 
    // READ full documentations for how this number is calculated to achieve 500m at maximum rates.
	Serial.begin(9600);
	lcd.begin(20, 4);

	can_vcu.reset();
	can_vcu.setBitrate(CAN_500KBPS, MCP_20MHZ);
	can_vcu.setNormalMode();
	Serial.write("working");

	lcd.print("CAN Initialized");
	delay(1000);
	lcd.clear();

	while (!Serial)
	{
		;
	}
	pinMode(HC12_SET_PIN,OUTPUT);
    digitalWrite(HC12_SET_PIN,LOW);
    Serial.println("AT+C069");

    digitalWrite(HC12_SET_PIN,HIGH);
	

	lcd.init();
	Serial.println("LCD init done");
	delay(100);
	lcd.backlight();
	lcd.createChar(0, degCelsius);
	Serial.println("Backlight on");
	delay(100);
	lcd.setCursor(0, 0);
	lcd.print("Ready");
	delay(500);

	pinMode(ENC_A, INPUT);
    pinMode(ENC_B, INPUT);
    lastStateA = digitalRead(ENC_A);
	lastStateB = digitalRead(ENC_B);
}

/**
 * @brief The Arduino loop function.
 * @details 
 */
void loop()
{
	/*
	// Cycle through each GPIO pin, setting it HIGH, waiting, then LOW.
	for (int i = 0; i < numGpioPins; i++)
	{
		Serial.print(gpioPins[i]);
		digitalWrite(gpioPins[i], HIGH);
	}
	delay(1000); // Wait for 1 second to observe the HIGH state.
	for (int i = 0; i < numGpioPins; i++)
	{

		Serial.print(gpioPins[i]);
		digitalWrite(gpioPins[i], LOW);
	}
	delay(500); // Wait for 0.5 seconds to observe the LOW state.

	*/

	lcd.setCursor(0, 3);
	lcd.print(millis());

	lcd.setCursor(8,1);
	lcd.write(0);
	


	MCP2515::ERROR read_state = can_vcu.readMessage(&rx_frame); //later detect encoder input to swap page from speed+temp into debug page
//page selector is rotating 

	Serial.write("test2");
	Serial.write('\n');



	int currentStateA = digitalRead(ENC_A);  ///< @brief read high/low of enc A & B
	int currentStateB = digitalRead(ENC_B);

	if (read_state == MCP2515::ERROR_OK)
	{
		//Serial.write(rx_frame.can_id+'0');
		//Serial.write("\t DLC: ");
	//	Serial.write(rx_frame.can_dlc+'0');
	//	Serial.write("\t Data: ");
	//	for (uint8_t i = 0; i < 8; ++i)
	//	{
	//		Serial.write(rx_frame.data[i]+'0');
	//		Serial.write("\t");
	//	}



		lcd.setCursor(0, 0);
		lcd.print("Temp:        ");
		lcd.setCursor(3, 0);
		lcd.print(rx_frame.can_id,HEX);
		lcd.setCursor(0, 1);
	    lcd.print("SPD:");
		lcd.print(rx_frame.can_dlc);
		lcd.setCursor(0, 2);
		for (uint8_t i = 0; i < rx_frame.can_dlc && i < 8; i++)
		{
			if (rx_frame.data[i] < 0x10)
				lcd.print("0");
			lcd.print(rx_frame.data[i], HEX);
		}
		Serial.print("CAN_ID:");
        Serial.println(rx_frame.can_id, HEX);
	}
}