/**
 * @file main.cpp
 * @brief This file contains the LCD testing code for the Gen6 Dash project.
 *        It allows output of CAN signals to the lcd
 * @note This code is intended for testing purposes only.
 */

#include <Arduino.h>
#include <mcp2515.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
static constexpr uint8_t RPM_PERIOD = 20;

can_frame rx_frame;

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

// Define pin assignment using actual Arduino pin numbers
// SPI Pins for CAN Controllers (MCP2515)
#define CAN0_CS_PIN PIN_PB2 ///< @brief Chip Select pin for CAN Controller 0 (PB2/SS)
#define CAN1_CS_PIN PIN_PB1 ///< @brief Chip Select pin for CAN Controller 1 (PB1)

// HC-12 Wireless Serial Communication Module Pins
#define HC12_RX PIN_PD0  ///< @brief Receive pin for HC-12 module (PD0 - Serial RX)
#define HC12_TX PIN_PD1  ///< @brief Transmit pin for HC-12 module (PD1 - Serial TX)
#define HC12_SET PIN_PB0 ///< @brief SET pin for HC-12 module (PB0)
SoftwareSerial HC12(HC12_RX,HC12_TX);

// Rotary Encoder Pins
#define ENC_A PIN_PC0 ///< @brief Pin for Rotary Encoder A output (PC0 - Analog 0)
#define ENC_B PIN_PC1 ///< @brief Pin for Rotary Encoder B output (PC1 - Analog 1)
int lastStateA , lastStateB;

// I2C Communication Pins
#define SDA_PIN PIN_PC4 ///< @brief SDA pin for I2C communication (PC4 - Analog 4)
#define SCL_PIN PIN_PC5 ///< @brief SCL pin for I2C communication (PC5 - Analog 5)

// General Purpose Input/Output (GPIO) Pins
#define GPIO_1 PIN_PC2 ///< @brief General Purpose I/O Pin 1 (PC2 - Analog 2)
#define GPIO_2 PIN_PC3 ///< @brief General Purpose I/O Pin 2 (PC3 - Analog 3)
#define GPIO_3 PIN_PD7  ///< @brief General Purpose I/O Pin 3 (PD7 - Digital 7)
#define GPIO_4 PIN_PD2  ///< @brief General Purpose I/O Pin 4 (PD2 - Digital 2)
#define GPIO_5 PIN_PD3  ///< @brief General Purpose I/O Pin 5 (PD3 - Digital 3)
#define GPIO_6 PIN_PD4  ///< @brief General Purpose I/O Pin 6 (PD4 - Digital 4)
#define GPIO_7 PIN_PD5  ///< @brief General Purpose I/O Pin 7 (PD5 - Digital 5)
#define GPIO_8 PIN_PD6  ///< @brief General Purpose I/O Pin 8 (PD6 - Digital 6)


/**
 * @brief An array containing all the GPIO pins to be tested.
 */
const int gpioPins[] = {
	GPIO_1, GPIO_2, GPIO_3, GPIO_4,
	GPIO_5, GPIO_6, GPIO_7, GPIO_8};
MCP2515 can_vcu(CAN0_CS_PIN);
MCP2515 can_ssru(CAN1_CS_PIN);
static constexpr canid_t VCU_READ = 0x181; /**< Motor read CAN ID */

// === even if unused, initialize ALL mcp2515 to make sure the CS pin is set up and they don't interfere with the SPI bus ===
/**MCP2515 mcp2515_motor(CS_CAN_MOTOR); // motor CAN
 *MCP2515 mcp2515_BMS(CS_CAN_BMS);     // BMS CAN
 *MCP2515 mcp2515_DL(CS_CAN_DL);       // datalogger CAN
 *
 *#define mcp2515_motor mcp2515_DL
 *#define mcp2515_BMS mcp2515_DL

// #define mcp2515_DL mcp2515_motor

 *constexpr uint8_t NUM_MCP = 3;
 *MCP2515 MCPS[NUM_MCP] = {mcp2515_motor, mcp2515_BMS, mcp2515_DL};
*/
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
 *
 *
 * ENCODER A - PC0 (Analog 0)
 * ENCODER B - PC1 (Analog 1)
 *
 * SDA - PC4 (Analog 4)
 * SCL - PC5 (Analog 5)
 * CAN0 CS - PB2 (Digital 10)
 * CAN1 CS - PB1 (Digital 9)
 */

/**
 * @brief The Arduino setup function. This function runs once when the sketch starts.
 * @details It initializes serial communication for debugging and configures all defined
 *          GPIO pins as OUTPUTs, setting their initial state to LOW.
 */
const int numGpioPins = sizeof(gpioPins) / sizeof(gpioPins[0]);

LiquidCrystal_I2C lcd(0x27, 20, 4);

// update ticks
uint32_t lastLcdTick = 0;
uint32_t lastCanReadTick = 0;

uint8_t write_counter = 0;

void setup()
{
	// Initialize Serial communication at 9600 baud rate for debugging output.	
    Serial.begin(9600);
    HC12.begin(9600);
	lcd.begin(20, 4);

	can_vcu.reset();
	can_vcu.setBitrate(CAN_500KBPS, MCP_20MHZ);
	can_vcu.setNormalMode();

	lcd.print("CAN Initialized");
	delay(1000);
	lcd.clear();

	while (!Serial)
	{
		;
	}

	// Iterate through the array of GPIO pins, setting each as an OUTPUT and ensuring it's LOW.
	for (int i = 0; i < numGpioPins; i++)
	{
		//pinMode(gpioPins[i], OUTPUT);
	}
	lcd.init();
	Serial.println("LCD init done");
	delay(100);
	lcd.backlight();
	Serial.println("Backlight on");
	delay(100);
	lcd.setCursor(0, 0);
	lcd.print("Ready");
	delay(500);
}

/**
 * @brief The Arduino loop function. This function runs repeatedly after setup().
 * @details It continuously cycles through each defined GPIO pin, setting it HIGH for 1 second,
 *          then LOW for 0.5 seconds. Serial messages indicate the current pin being tested
 *          and its state. A 2-second delay is introduced after each complete cycle.
 */
void loop()
{

	lcd.setCursor(0, 3);
	lcd.print(millis());

	MCP2515::ERROR read_state = can_vcu.readMessage(&rx_frame);
	Serial.print("State: ");
	Serial.write(static_cast<char>(read_state)+'a');
	Serial.write("\n");

	if (read_state == MCP2515::ERROR_OK)
	{
		Serial.print("ID: 0x");
		Serial.print(rx_frame.can_id, HEX);
		Serial.print("\tDLC: ");
		Serial.print(rx_frame.can_dlc);
		Serial.print("\tData: ");
		for (int i = 0; i < rx_frame.can_dlc && i < 8; ++i)
		{
			if (rx_frame.data[i] < 0x10)
				Serial.print("0");
			Serial.print(rx_frame.data[i], HEX);
			Serial.print(" ");
		}
		Serial.println();

		lcd.setCursor(0, 0);
		lcd.print("ID:        ");
		lcd.setCursor(3, 0);
		lcd.print(rx_frame.can_id, HEX);
		lcd.setCursor(0, 1);
		lcd.print("DLC:");
		lcd.print(rx_frame.can_dlc);
		lcd.setCursor(0, 2);
		for (uint8_t i = 0; i < rx_frame.can_dlc && i < 8; i++)
		{
			if (rx_frame.data[i] < 0x10)
				lcd.print("0");
			lcd.print(rx_frame.data[i], HEX);
		}
	}
}
