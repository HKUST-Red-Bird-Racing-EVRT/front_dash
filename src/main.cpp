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

// Define pin assignment using actual Arduino pin numbers
// SPI Pins for CAN Controllers (MCP2515)
#define NUM_MCP 2       ///< @brief Number of MCP2515 controllers
#define CAN0_CS PIN_PB2 ///< @brief Chip Select pin for CAN Controller 0 (PB2/SS)
#define CAN1_CS PIN_PB1 ///< @brief Chip Select pin for CAN Controller 1 (PB1)

// HC-12 Wireless Serial Communication Module Pins
#define HC12_RX PIN_PD0  ///< @brief Receive pin for HC-12 module (PD0 - Serial RX)
#define HC12_TX PIN_PD1  ///< @brief Transmit pin for HC-12 module (PD1 - Serial TX)
#define HC12_SET PIN_PB0 ///< @brief SET pin for HC-12 module (PB0)
SoftwareSerial HC12(PIN_PD1, PIN_PD0);

// Rotary Encoder Pins
#define ENC_A PIN_PC0 ///< @brief Pin for Rotary Encoder A output (PC0 - Analog 0)
#define ENC_B PIN_PC1 ///< @brief Pin for Rotary Encoder B output (PC1 - Analog 1)
int lastStateA, lastStateB;

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
    for (int i = 0; i < NUM_MCP; ++i)
    {
        cans[i].reset();
        cans[i].setBitrate(CAN_500KBPS, MCP_20MHZ);
        cans[i].setNormalMode();
    }
    Serial.begin(9600);
    while (!Serial)
    {
        ;
    }
}

void loop()
{
    MCP2515::ERROR read_state = can_vcu.readMessage(&rx_frame);
    if (read_state == MCP2515::ERROR_OK)
    {
        uint8_t output = 0b10100000;
        switch (rx_frame.can_id) //20 20 200 ms counttime
        {
            case 0x700:
                break;
            case 0x701:
                output += 1;
                break;
            case 0x710:
                output += 2;
                break;
            // case 0x720:TODO
            default:
                output += 3;
        }
        Serial.write(output);
        // Serial.write(rx_frame.can_id >> 8 & 0xFF);
        // Serial.write(rx_frame.can_id >> 16 & 0xFF);
        // Serial.write(rx_frame.can_id >> 24 & 0xFF);
        // Serial.write(rx_frame.can_dlc);
        // for (int i = 0; i < rx_frame.can_dlc && i < 8; ++i)
        // for (int i = 0; i < 8; ++i)
        //{
        Serial.write(rx_frame.data, 8);
        //}
    }
}