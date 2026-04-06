/**
 * @file main.cpp
 * @brief This file contains the GPIO testing code for the Gen6 Dash project.
 *        It is designed to cycle through General Purpose Input/Output
 *        pins on the ATmega328P microcontroller, setting them HIGH and LOW.
 *        This can be observed with a multimeter or an LED connected to the respective pins
 * @note This code is intended for testing purposes only.
 */

#include <Arduino.h>
#include "BoardConfig.h"
#include <mcp2515.h>
#include <LiquidCrystal_I2C.h>

// Define pin assignment using actual Arduino pin numbers
// SPI Pins for CAN Controllers (MCP2515)
#define CAN0_CS_PIN    10    ///< @brief Chip Select pin for CAN Controller 0 (PB2/SS)
#define CAN1_CS_PIN    9     ///< @brief Chip Select pin for CAN Controller 1 (PB1)

// HC-12 Wireless Serial Communication Module Pins
#define HC12_RX_PIN    0     ///< @brief Receive pin for HC-12 module (PD0 - Serial RX)
#define HC12_TX_PIN    1     ///< @brief Transmit pin for HC-12 module (PD1 - Serial TX)
#define HC12_SET_PIN   8     ///< @brief SET pin for HC-12 module (PB0)

// Rotary Encoder Pins
#define ENC_A_PIN      A0    ///< @brief Pin for Rotary Encoder A output (PC0 - Analog 0)
#define ENC_B_PIN      A1    ///< @brief Pin for Rotary Encoder B output (PC1 - Analog 1)

// I2C Communication Pins
#define SDA_PIN        A4    ///< @brief SDA pin for I2C communication (PC4 - Analog 4)
#define SCL_PIN        A5    ///< @brief SCL pin for I2C communication (PC5 - Analog 5)

// General Purpose Input/Output (GPIO) Pins
#define GPIO_1_PIN     A2    ///< @brief General Purpose I/O Pin 1 (PC2 - Analog 2)
#define GPIO_2_PIN     A3    ///< @brief General Purpose I/O Pin 2 (PC3 - Analog 3)
#define GPIO_3_PIN     7     ///< @brief General Purpose I/O Pin 3 (PD7 - Digital 7)
#define GPIO_4_PIN     2     ///< @brief General Purpose I/O Pin 4 (PD2 - Digital 2)
#define GPIO_5_PIN     3     ///< @brief General Purpose I/O Pin 5 (PD3 - Digital 3)
#define GPIO_6_PIN     4     ///< @brief General Purpose I/O Pin 6 (PD4 - Digital 4)
#define GPIO_7_PIN     5     ///< @brief General Purpose I/O Pin 7 (PD5 - Digital 5)
#define GPIO_8_PIN     6     ///< @brief General Purpose I/O Pin 8 (PD6 - Digital 6)

/**
 * @brief An array containing all the GPIO pins to be tested.
 */
const int gpioPins[] = {
  GPIO_1_PIN, GPIO_2_PIN, GPIO_3_PIN, GPIO_4_PIN,
  GPIO_5_PIN, GPIO_6_PIN, GPIO_7_PIN, GPIO_8_PIN
};
const int numGpioPins = sizeof(gpioPins) / sizeof(gpioPins[0]); ///< @brief The number of GPIO pins in the `gpioPins` array.

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


MCP2515 can0(10);
LiquidCrystal_I2C lcd(0x27,20,4);

//update ticks
uint32_t lastLcdTick = 0;


void setup() {
  // Initialize Serial communication at 9600 baud rate for debugging output.
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for the serial port to connect. This is needed for native USB port only.
  }
  Serial.println("Starting Dash Testing Code...");
  Serial.println("--- GPIO Test ---");

  Serial.begin(9600);
  // Iterate through the array of GPIO pins, setting each as an OUTPUT and ensuring it's LOW.
  for (int i = 0; i < numGpioPins; i++) {
    pinMode(gpioPins[i], OUTPUT);
    digitalWrite(gpioPins[i], LOW);
  }
}

/**
 * @brief The Arduino loop function. This function runs repeatedly after setup().
 * @details It continuously cycles through each defined GPIO pin, setting it HIGH for 1 second,
 *          then LOW for 0.5 seconds. Serial messages indicate the current pin being tested
 *          and its state. A 2-second delay is introduced after each complete cycle.
 */
void loop() {
  // Cycle through each GPIO pin, setting it HIGH, waiting, then LOW.
  for (int i = 0; i < numGpioPins; i++) {
    Serial.print("Testing GPIO Pin: ");
    Serial.print(gpioPins[i]);
    Serial.println(" - Setting HIGH");
    digitalWrite(gpioPins[i], HIGH);
    delay(1000); // Wait for 1 second to observe the HIGH state.

    Serial.print("Testing GPIO Pin: ");
    Serial.print(gpioPins[i]);
    Serial.println(" - Setting LOW");
    digitalWrite(gpioPins[i], LOW);
    delay(500); // Wait for 0.5 seconds to observe the LOW state.
  }

  Serial.println("--- GPIO Test Cycle Complete ---");
  delay(2000); // Wait for 2 seconds before repeating the entire GPIO test cycle.
}