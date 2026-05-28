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
#include "Page.hpp"


/**
 * @brief Get pedal fault
 */
const char *getPedalFaultString(uint8_t fault_byte)
{
	if (fault_byte & 0x01)
		return "Pedal FAULT ACTIVE!";
	if (fault_byte & 0x02)
		return "Pedal fault >100ms";
	if (fault_byte & 0x04)
		return "APPS 5V low";
	if (fault_byte & 0x08)
		return "APPS 5V high";
	if (fault_byte & 0x10)
		return "APPS 3V3 low";
	if (fault_byte & 0x20)
		return "APPS 3V3 high";
	if (fault_byte & 0x40)
		return "Brake low";
	if (fault_byte & 0x80)
		return "Brake high";
	return "No fault";
}

/**
 * @brief Get BMS status
 */
const char *getBmsStatusString(BmsStatus status)
{
	switch (status)
	{
	case BmsStatus::NoMsg:
		return "No BMS message";
	case BmsStatus::Waiting:
		return "BMS Waiting";
	case BmsStatus::Starting:
		return "BMS Starting HV";
	case BmsStatus::Started:
		return "BMS HV Started";
	default:
		return "BMS Unknown";
	}
}

can_frame rx_frame;
int message_count = 0;
uint8_t pot_buffer[8] = {0}; // Buffer for merged POT data (0x730 + 0x750)
bool pot_730_ready = false;
bool pot_750_ready = false;
bool hasStarted = false;

int16_t torque_val, motor_rpm = 0;
uint16_t motor_warn, motor_error = 0; // Variable to store motor stuff
uint16_t carstate = 0;
uint32_t odometer_integral = 0; // Variable to store integral of RPM for odometer calculation

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
#define PIN_ENC_A PIN_PD2 ///< @brief Pin for Rotary Encoder A output (PC0 - Analog 0)
#define PIN_ENC_B PIN_PC1 ///< @brief Pin for Rotary Encoder B output (PC1 - Analog 1)

#define PIN_SHIFT_ENC_B PC1 ///< @brief Amount to shift for read

constexpr int8_t NUM_PAGES = 5;

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
int8_t write_counter = 0;
MCP2515 cans[NUM_MCP] = {can_vcu, can_ssru};

// Page instance
DashboardPage dashboardPage(lcd);
Page* currentPage = &dashboardPage;

CarState car;

// BMS State
struct BmsData
{
	uint8_t raw_data[8];
	BmsStatus status;
} bms;

// Snake Game State
struct SnakeGame
{
	static constexpr uint8_t LCD_WIDTH = 20;
	static constexpr uint8_t LCD_HEIGHT = 4;
	static constexpr uint8_t MAX_SNAKE_LENGTH = 20;
	struct Point
	{
		uint8_t x;
		uint8_t y;
	};

	Point snake[MAX_SNAKE_LENGTH];
	uint8_t snake_length;
	Point apple;
	uint8_t direction; // 0=up, 1=right, 2=down, 3=left
	uint8_t next_direction;
	bool game_over;
	uint32_t last_move_time;
	static constexpr uint32_t MOVE_DELAY_MS = 300;

	void init()
	{
		snake_length = 3;
		snake[0] = {10, 2};
		snake[1] = {9, 2};
		snake[2] = {8, 2};
		direction = 1; // moving right
		next_direction = 1;
		game_over = false;
		last_move_time = millis();
		spawn_apple();
	}

	void spawn_apple()
	{
		bool valid = false;
		while (!valid)
		{
			apple.x = random(0, LCD_WIDTH);
			apple.y = random(0, LCD_HEIGHT);
			valid = true;
			for (uint8_t i = 0; i < snake_length; ++i)
			{
				if (snake[i].x == apple.x && snake[i].y == apple.y)
				{
					valid = false;
					break;
				}
			}
		}
	}

	void update()
	{
		if (game_over)
			return;

		uint32_t now = millis();
		if (now - last_move_time < MOVE_DELAY_MS)
			return;

		direction = next_direction;
		Point new_head = snake[0];
		switch (direction)
		{
		case 0: // up
			new_head.y = (new_head.y == 0) ? LCD_HEIGHT - 1 : new_head.y - 1;
			break;
		case 1: // right
			new_head.x = (new_head.x == LCD_WIDTH - 1) ? 0 : new_head.x + 1;
			break;
		case 2: // down
			new_head.y = (new_head.y == LCD_HEIGHT - 1) ? 0 : new_head.y + 1;
			break;
		case 3: // left
			new_head.x = (new_head.x == 0) ? LCD_WIDTH - 1 : new_head.x - 1;
			break;
		}
		// Check collision
		for (uint8_t i = 0; i < snake_length; ++i)
		{
			if (new_head.x == snake[i].x && new_head.y == snake[i].y)
			{
				game_over = true;
				return;
			}
		}

		// Move snake
		for (uint8_t i = snake_length; i > 0; --i)
		{
			snake[i] = snake[i - 1];
		}
		snake[0] = new_head;
		if (new_head.x == apple.x && new_head.y == apple.y)
		{
			if (snake_length < MAX_SNAKE_LENGTH)
			{
				snake_length++;
			}
			spawn_apple();
		}

		last_move_time = now;
		delay(50);
	}
} snake_game;

volatile uint8_t encoder_count = 0;
volatile bool encoder_changed = false;
volatile uint8_t last_key_pressed = 0; // 0=none, 'w'=up, 'a'=left, 's'=down, 'd'=right, 'r'=restart

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

void lcd_updater(int pagenum, int lcd_update_state)
{
	if (pagenum == 0)
	{ // driver
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
	}
	else if (pagenum == 1)
	{ // VCU
		static bool page1_init = false;
		if (!page1_init || lcd_update_state == 0)
		{
			lcd.clear();
			lcd.setCursor(3, 0);
			lcd.print("  VCU/CAR PROBLEMS");
			lcd.setCursor(0, 1);
			lcd.print("Pedal Faults:");
			page1_init = true;
		}

		if (lcd_update_state >= 1 && lcd_update_state <= 4)
		{
			lcd.setCursor(0, 2 + (lcd_update_state - 1));
			lcd.print("                    ");
			lcd.setCursor(0, 2 + (lcd_update_state - 1));

			if (car.pedal.faults.byte == 0)
			{
				if (lcd_update_state == 1)
					lcd.print("OK");
			}
			else
			{
				uint8_t fault_index = (lcd_update_state - 1) % 8;
				if (car.pedal.faults.byte & (1 << fault_index))
				{
					lcd.print(getPedalFaultString(car.pedal.faults.byte));
				}
			}
		}
	}
	else if (pagenum == 2)
	{ // BMS
		static bool page2_init = false;
		if (!page2_init || lcd_update_state == 0)
		{
			lcd.clear();
			lcd.setCursor(3, 0);
			lcd.print("   BMS PROBLEMS");
			lcd.setCursor(0, 1);
			lcd.print("BMS Status:");
			page2_init = true;
		}

		if (lcd_update_state >= 1 && lcd_update_state <= 3)
		{
			lcd.setCursor(0, 2);
			lcd.print(" ");
			lcd.setCursor(0, 2);
			lcd.print(getBmsStatusString(bms.status));

			lcd.setCursor(0, 3);
			lcd.print(" ");
			if (bms.status == BmsStatus::NoMsg)
			{
				lcd.setCursor(0, 3);
				lcd.print("ERROR: No BMS msg!");
			}
		}
	}
	else if (pagenum == 3)
	{ // Reserved
		switch (lcd_update_state)
		{
		case 0:
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("hi");
			break;
		}
		}
	}
	else if (pagenum == 4)
	{ // Snake (yes)
		if (lcd_update_state == 0)
		{
			if (!snake_game.game_over)
			{
				lcd.clear();
				snake_game.init();
			}

			if (last_key_pressed != 0)
			{
				switch (last_key_pressed)
				{
				case 'w': // up
					if (snake_game.direction != 2)
						snake_game.next_direction = 0;
					break;
				case 'd': // right
					if (snake_game.direction != 3)
						snake_game.next_direction = 1;
					break;
				case 's': // down
					if (snake_game.direction != 0)
						snake_game.next_direction = 2;
					break;
				case 'a': // left
					if (snake_game.direction != 1)
						snake_game.next_direction = 3;
					break;
				case 'r': // restart
					snake_game.init();
					break;
				}
				last_key_pressed = 0;
			}
			snake_game.update();
			lcd.clear();
			for (uint8_t i = 0; i < snake_game.snake_length; ++i)
			{
				lcd.setCursor(snake_game.snake[i].x, snake_game.snake[i].y);
				if (i == 0)
					lcd.print("@"); // head
				else
					lcd.print("o"); // body
			}
			lcd.setCursor(snake_game.apple.x, snake_game.apple.y);
			lcd.print("*");
			if (snake_game.game_over)
			{
				lcd.setCursor(0, 0);
				lcd.print("GAME OVER! Len:");
				lcd.setCursor(15, 0);
				lcd.print(snake_game.snake_length, DEC);
				lcd.setCursor(2, 2);
				lcd.print("Press R to restart");
			}
		}
	}
}

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

	// Initialize the page
	if (currentPage != nullptr)
	{
		currentPage->setup();
	}
}
void loop()
{
	if (Serial.available())
	{
		uint8_t key = Serial.read();
		if (encoder_count == 4)
		{
			if (key == 'w' || key == 'd' || key == 's' || key == 'a' || key == 'r')
			{
				last_key_pressed = key;
			}
		}
	}

	// Update the current page
	if (currentPage != nullptr)
	{
		currentPage->update();
	}

	if (encoder_changed)
	{
		encoder_count = (encoder_count % NUM_PAGES + NUM_PAGES) % NUM_PAGES;
		
		switch (encoder_count)
		{
		case 0: // driver
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
			break;
		}
		case 1: // VCU
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("  VCU/CAR PROBLEMS");
			break;
		}
		case 2: // BMS
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("   BMS PROBLEMS");
			break;
		}
		case 3: // empty
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("Hi");
			break;
		}
		case 4: // Snake game
		{
			lcd.clear();
			lcd.setCursor(2, 1);
			lcd.print("Snake Game");
			lcd.setCursor(0, 2);
			lcd.print("Loading...");
			delay(1000);
			break;
		}
		}
		encoder_changed = false;
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
		static uint8_t lcd_update_state = 0;
		odometer_integral += abs(motor_rpm);
		lcd_updater(encoder_count, lcd_update_state);
		lcd_update_state = (lcd_update_state + 1) % (lcd_update::update_items);
		lastLcdTick += lcd_update::update_interval_ms;
	}
}
