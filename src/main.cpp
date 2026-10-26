/**
 * @file main.cpp
 * @author ChiHo Ngan, Red Bird Racing
 * @brief Front dash firmware for the gen6 car.
 * @details Reads up to 3 CAN buses and drives the LCD pages, the SD log and the
 * Serial stream. The rotary encoder switches pages; its button clears latched
 * motor faults.
 * @version 2.0
 * @date 2026-10-27
 */

#include <Arduino.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/crc16.h>
#include <mcp2515.h>
#include <string.h>
#include "pinMap.h"
#include "Dash_I2C.hpp"
#include "DashState.hpp"
#include "Structs.h"
#include "Page.hpp"
#include "SD.hpp"
#include "namespace.h"

// ===== Shared telemetry =====
DashState car;
BmsData bms;
int16_t torque_val = 0, motor_rpm = 0;
uint16_t motor_warn = 0, motor_error = 0;
uint32_t odometer_integral = 0;
uint8_t pot_buffer[8] = {0}; ///< 0x730 bytes 0-3 + 0x750 bytes 0-3
/// millis() of the last frame per Src. Starts expired so pages show "--" until data arrives.
uint32_t lastRxMs[3] = {~0UL - STALE_MS_BMS, ~0UL - STALE_MS_BMS, ~0UL - STALE_MS_BMS};

can_frame rx_frame;
bool pot_730_ready = false;
bool pot_750_ready = false;
bool hasStarted = false;

DashI2CDriver i2c;
SdLogger sdLog;
LiquidCrystal_I2C lcd(0x27, 20, 4);

DriverPage driverPage;
VCUPage vcuPage;
BMSPage bmsPage;
DefaultPage defaultPage;

// ===== CAN =====
// Bus 0 = VCU bus, decoded in handleVcuFrame(). Buses 1 and 2 are only logged.
#ifdef CAN2_CS
constexpr uint8_t NUM_CAN = 3;
MCP2515 cans[NUM_CAN] = {CAN0_CS, CAN1_CS, CAN2_CS};
#else
constexpr uint8_t NUM_CAN = 2;
MCP2515 cans[NUM_CAN] = {CAN0_CS, CAN1_CS};
#endif
#ifdef CAN_INT_BITS
constexpr uint8_t CAN_INT_BIT[NUM_CAN] = CAN_INT_BITS;
#endif
bool canOk[NUM_CAN] = {};

// ===== LCD timing / redraw cache =====
uint32_t lastLcdTick = 0;
uint32_t idleStuckSince = 0;
uint8_t lcdItem = 0;                                ///< Field redrawn on the next tick
uint16_t fieldCrc[lcd_update::update_items];        ///< CRC of each field's last draw
uint8_t forceTicks = lcd_update::update_items;      ///< Ticks left that redraw regardless of CRC
uint8_t refreshTick = 0;
bool btnWasDown = false;
#ifdef PAGE_AUTOCYCLE_MS
uint32_t lastPageSwitch = 0;
#endif
bool powerWasOk = true;

// ===== Custom LCD glyphs (PROGMEM) =====
const byte num1_inverted[8] PROGMEM = {
	B11111,
	B11011,
	B10011,
	B11011,
	B11011,
	B11011,
	B10001,
	B11111};
const byte num2_inverted[8] PROGMEM = {
	B11111,
	B10011,
	B11101,
	B11101,
	B11011,
	B10111,
	B10001,
	B11111};
const byte num3_inverted[8] PROGMEM = {
	B11111,
	B10011,
	B11101,
	B11011,
	B11101,
	B10011,
	B11111,
	B00000};
const byte num4_inverted[8] PROGMEM = {
	B11111,
	B10101,
	B10101,
	B10101,
	B10001,
	B11101,
	B11101,
	B11111};
extern constexpr byte CHAR_LOCKED = 0;
extern constexpr byte CHAR_DEG = 1;
extern constexpr byte CHAR_PWR = 2;
constexpr byte PAGE_INDICATOR_1 = 3;
constexpr byte PAGE_INDICATOR_2 = 4;
constexpr byte PAGE_INDICATOR_3 = 5;
constexpr byte PAGE_INDICATOR_4 = 6;

const byte byte_char_locked[8] PROGMEM = {
	0b01110,
	0b10001,
	0b10001,
	0b11111,
	0b11011,
	0b11011,
	0b11011,
	0b11111};
const byte byte_pwr[8] PROGMEM = {
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111};
const byte degCelsius[8] PROGMEM = {
	0b01000,
	0b10100,
	0b01000,
	0b00011,
	0b00100,
	0b00100,
	0b00100,
	0b00011};

/**
 * @brief Draw the page numbers in column 19, current page inverted.
 * @param currentPage Index of the active page.
 */
void drawPageIndicators(int currentPage)
{
	lcd.setCursor(19,0);
	if (currentPage == 0)
	{
		lcd.write(PAGE_INDICATOR_1);
	}
	else
		lcd.print(F("1"));
	lcd.setCursor(19, 1);
	if (currentPage == 1)
		lcd.write(PAGE_INDICATOR_2);
	else
		lcd.print(F("2"));
	lcd.setCursor(19, 2);
	if (currentPage == 2)
		lcd.write(PAGE_INDICATOR_3);
	else
		lcd.print(F("3"));
	lcd.setCursor(19, 3);
	if (currentPage == 3)
		lcd.write(PAGE_INDICATOR_4);
	else
		lcd.print(F("4"));
}

// ===== Rotary encoder =====
// ENC_A = PD2 (INT0), ENC_B = PD3 (INT1) or PC1 (PCINT9 on the old PCB).
// Quadrature decoded on every edge of both pins.
constexpr int8_t ENC_STEPS_PER_DETENT = 4; ///< Quadrature steps per click
constexpr int8_t ENC_DIRECTION = 1;        ///< -1 to reverse the page direction
/// Step per transition, index = (previous BA << 2) | current BA.
constexpr int8_t ENC_STEP[16] PROGMEM = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
static_assert(ENC_STEP[0b0001] + ENC_STEP[0b0111] + ENC_STEP[0b1110] + ENC_STEP[0b1000] == -4, "full cycle 00>01>11>10>00 must be 4 steps one way");
static_assert(ENC_STEP[0b0100] + ENC_STEP[0b1101] + ENC_STEP[0b1011] + ENC_STEP[0b0010] == 4, "reverse cycle must be 4 steps the other way");
static_assert(ENC_STEP[0b0000] == 0 && ENC_STEP[0b0101] == 0 && ENC_STEP[0b1111] == 0 && ENC_STEP[0b1010] == 0, "no change = no step");
static_assert(ENC_STEP[0b0011] == 0 && ENC_STEP[0b1100] == 0 && ENC_STEP[0b0110] == 0 && ENC_STEP[0b1001] == 0, "skipped state (both pins flipped) = ignored");
uint8_t encState = 0;         ///< Last two BA states, ISR-only after setup()
volatile int8_t encSteps = 0; ///< Steps not yet turned into page changes

/** @return Current encoder pins as BA (A = bit 0, B = bit 1). */
inline uint8_t readEncoder()
{
	return static_cast<uint8_t>(((PIND >> PD2) & 1) | (((ENC_B_PINREG >> ENC_B_BIT) & 1) << 1));
}

ISR(INT0_vect)
{
	encState = static_cast<uint8_t>(((encState << 2) | readEncoder()) & 0x0F);
	encSteps += static_cast<int8_t>(pgm_read_byte(&ENC_STEP[encState]));
}
#ifdef OLD_PCB
ISR(PCINT1_vect, ISR_ALIASOF(INT0_vect));
#else
ISR(INT1_vect, ISR_ALIASOF(INT0_vect));
#endif

ISR(TWI_vect)
{
	i2c.handleIsr();
}

// ===== Odometer persistence =====
/** @brief EEPROM odometer slot. Two slots are alternated so an interrupted write keeps the other. */
struct OdoSlot
{
	uint32_t value; ///< odometer_integral
	uint8_t seq;    ///< Incremented per save, newest wins
	uint8_t crc;    ///< CRC-8 of value and seq
};
OdoSlot *const ODO_EE = reinterpret_cast<OdoSlot *>(0); ///< EEPROM bytes 0..11
uint8_t odoSlot = 1; ///< Slot with the newest value
uint8_t odoSeq = 0;

/** @return CRC-8 of a slot's value and seq. Blank EEPROM (0xFF) never matches. */
uint8_t odoCrc(const OdoSlot &s)
{
	const uint8_t *p = reinterpret_cast<const uint8_t *>(&s);
	uint8_t crc = 0;
	for (uint8_t i = 0; i < offsetof(OdoSlot, crc); ++i)
		crc = _crc8_ccitt_update(crc, p[i]);
	return crc;
}

/** @return Newest valid odometer value from EEPROM, 0 if none. */
uint32_t loadOdometer()
{
	OdoSlot s[2];
	eeprom_read_block(s, ODO_EE, sizeof(s));
	bool ok0 = odoCrc(s[0]) == s[0].crc;
	bool ok1 = odoCrc(s[1]) == s[1].crc;
	if (!ok0 && !ok1)
		return 0;
	odoSlot = (ok0 && (!ok1 || static_cast<int8_t>(s[0].seq - s[1].seq) > 0)) ? 0 : 1;
	odoSeq = s[odoSlot].seq;
	return s[odoSlot].value;
}

/**
 * @brief Save the odometer to the older slot. Takes about 20 ms.
 * @param value odometer_integral to store.
 */
void saveOdometer(uint32_t value)
{
	odoSlot ^= 1;
	OdoSlot s = {value, ++odoSeq, 0};
	s.crc = odoCrc(s);
	eeprom_update_block(&s, &ODO_EE[odoSlot], sizeof(s));
}

Page *pages[] = {
	&driverPage,
	&vcuPage,
	&bmsPage,
	&defaultPage,
};
constexpr uint8_t PAGE_COUNT = sizeof(pages) / sizeof(pages[0]);
uint8_t currentPageIndex = 0;
Page *currentPage = pages[currentPageIndex];

/**
 * @brief Switch to a page: clear, draw labels, repaint every field.
 * @param index Index into pages[].
 */
void showPage(uint8_t index)
{
	currentPageIndex = index;
	currentPage = pages[index];
	lcd.clear();
	currentPage->setup();
	forceTicks = lcd_update::update_items;
	drawPageIndicators(index);
}

/** @brief Initialise the LCD and load the custom glyphs. */
void initLcd()
{
	lcd.begin(20, 4);
	lcd.backlight();
	lcd.createChar(CHAR_LOCKED, byte_char_locked);
	lcd.createChar(CHAR_DEG, degCelsius);
	lcd.createChar(CHAR_PWR, byte_pwr);
	lcd.createChar(PAGE_INDICATOR_1, num1_inverted);
	lcd.createChar(PAGE_INDICATOR_2, num2_inverted);
	lcd.createChar(PAGE_INDICATOR_3, num3_inverted);
	lcd.createChar(PAGE_INDICATOR_4, num4_inverted);
}

/** @brief Decode rx_frame from the VCU bus into telemetry and forward it on Serial. */
void handleVcuFrame()
{
	uint8_t output = 0b10100000;
	switch (rx_frame.can_id)
	{
	case 0x700: // VCU pedals, 10-bit fields packed by TelemetryFramePedal::fromCanFrame()
		car.pedal.apps_5v = rx_frame.data[0] | (static_cast<uint16_t>(rx_frame.data[1] & 0x03) << 8);
		car.pedal.apps_3v3 = ((rx_frame.data[1] >> 2) & 0x3F) | (static_cast<uint16_t>(rx_frame.data[2] & 0x0F) << 6);
		car.pedal.brake = ((rx_frame.data[2] >> 4) & 0x0F) | (static_cast<uint16_t>(rx_frame.data[3] & 0x3F) << 4);
		car.pedal.hall_sensor = ((rx_frame.data[3] >> 6) & 0x03) | (static_cast<uint16_t>(rx_frame.data[4]) << 2);
		car.pedal.status.byte = rx_frame.data[5];
		car.pedal.faults.byte = rx_frame.data[6];
		lastRxMs[static_cast<uint8_t>(Src::Pedal)] = millis();
		break;
	case 0x701: // VCU motor
		torque_val = (rx_frame.data[1] << 8) | rx_frame.data[0];
		motor_rpm = (rx_frame.data[3] << 8) | rx_frame.data[2];
		if (hasStarted)
		{
			motor_warn |= ((rx_frame.data[5] << 8) | rx_frame.data[4]);
			motor_error |= ((rx_frame.data[7] << 8) | rx_frame.data[6]);
		}
		lastRxMs[static_cast<uint8_t>(Src::Motor)] = millis();
		output += 1;
		break;
	case 0x710: // VCU BMS
		memcpy(bms.raw_data, rx_frame.data, 8);
		bms.status = BmsStatus::Started;
		lastRxMs[static_cast<uint8_t>(Src::Bms)] = millis();
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
		output += 15;
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

void setup()
{
	uint8_t resetCause = MCUSR;
	MCUSR = 0;
	wdt_disable();

#ifdef SD_CS
	// Deselect the SD card before any SPI traffic so it can't drive MISO.
	digitalWrite(SD_CS, HIGH);
	pinMode(SD_CS, OUTPUT);
#endif

	pinMode(PIN_ENC_A, INPUT_PULLUP);
	pinMode(PIN_ENC_B, INPUT_PULLUP);
#ifdef PIN_ENC_BTN
	pinMode(PIN_ENC_BTN, INPUT_PULLUP); // pressed = LOW
#endif
#ifdef PIN_POWER_FAIL
	pinMode(PIN_POWER_FAIL, INPUT);
#endif

	Serial.begin(115200);
	initLcd();

	// Boot screen
	lcd.setCursor(0, 0);
	lcd.print(F("RBR Dash"));
	lcd.setCursor(0, 1);
	lcd.print(F("CAN"));
	for (uint8_t i = 0; i < NUM_CAN; ++i)
	{
		cans[i].reset();
		canOk[i] = cans[i].setBitrate(CAN_500KBPS, MCP_20MHZ) == MCP2515::ERROR_OK &&
				   cans[i].setNormalMode() == MCP2515::ERROR_OK;
		lcd.write(' ');
		lcd.write('0' + i);
		lcd.print(canOk[i] ? F(":OK") : F(":NO"));
	}
	lcd.setCursor(0, 2);
#ifdef SD_CS
	lcd.print(sdLog.begin(SD_CS) ? F("SD OK") : F("SD NO CARD"));
#else
	lcd.print(F("old PCB: no SD"));
#endif
	lcd.setCursor(0, 3);
	if (resetCause & (1 << WDRF))
		lcd.print(F("WATCHDOG RESET"));
	odometer_integral = loadOdometer();
	delay(400);

	showPage(0);

	encState = readEncoder();
#ifdef OLD_PCB
	EICRA = (1 << ISC00);   // INT0 any edge
	PCMSK1 = (1 << PCINT9);
	PCIFR = (1 << PCIF1);
	PCICR = (1 << PCIE1);
	EIFR = (1 << INTF0);
	EIMSK = (1 << INT0);
#else
	EICRA = (1 << ISC00) | (1 << ISC10); // INT0, INT1 any edge
	EIFR = (1 << INTF0) | (1 << INTF1);
	EIMSK = (1 << INT0) | (1 << INT1);
#endif

	lastLcdTick = millis();
	wdt_enable(WDTO_1S);
}

void loop()
{
	wdt_reset();

	// Power fail: flush the SD log and save the odometer.
#ifdef PIN_POWER_FAIL
	bool powerOk = digitalRead(PIN_POWER_FAIL);
#else
	const bool powerOk = true;
#endif
	sdLog.setPowerOk(powerOk);
	if (!powerOk && powerWasOk)
		saveOdometer(odometer_integral);
	powerWasOk = powerOk;

	// Encoder: whole detents change page, leftover steps carry over.
	cli();
	int8_t detents = encSteps / ENC_STEPS_PER_DETENT;
	encSteps -= detents * ENC_STEPS_PER_DETENT;
	sei();
	if (detents)
	{
		showPage((currentPageIndex + ENC_DIRECTION * detents % PAGE_COUNT + PAGE_COUNT) % PAGE_COUNT);
	}
#ifdef PAGE_AUTOCYCLE_MS
	if (detents)
	{
		lastPageSwitch = millis();
	}
	else if (millis() - lastPageSwitch >= PAGE_AUTOCYCLE_MS)
	{
		lastPageSwitch = millis();
		showPage((currentPageIndex + 1) % PAGE_COUNT);
	}
#endif

	hasStarted = (car.pedal.status.bits.car_status == CarStatus::Drive);
	for (uint8_t bus = 0; bus < NUM_CAN; ++bus)
	{
		if (!canOk[bus])
			continue;
#ifdef CAN_INT_BITS
		if (PINC & (1 << CAN_INT_BIT[bus])) // INT high: nothing received
			continue;
#endif
		bool got = false;
		while (cans[bus].readMessage(&rx_frame) == MCP2515::ERROR_OK)
		{
			got = true;
			sdLog.log(rx_frame, bus);
			if (bus == 0)
				handleVcuFrame();
		}
#ifdef CAN_INT_BITS
		if (!got)
		{
			// INT low without a frame: error flag, clear it.
			cans[bus].clearERRIF();
			cans[bus].clearMERR();
		}
#else
		(void)got;
#endif
	}
	sdLog.service();

	if (millis() - lastLcdTick >= lcd_update::update_interval_ms)
	{
		if (!stale(Src::Motor))
			odometer_integral += absU16(motor_rpm);
		lastLcdTick += lcd_update::update_interval_ms;

#ifdef PIN_ENC_BTN
		// Encoder button press clears latched motor faults.
		bool btnDown = !digitalRead(PIN_ENC_BTN);
		if (btnDown && !btnWasDown)
		{
			motor_warn = 0;
			motor_error = 0;
		}
		btnWasDown = btnDown;
#endif

		if (++refreshTick == lcd_update::update_count) // once per second
		{
			refreshTick = 0;
			forceTicks = lcd_update::update_items;
		}
		// Redraw the field only if its content changed or a refresh is forced.
		lcd.beginDryRun();
		currentPage->update(lcdItem);
		uint16_t crc = lcd.endDryRun();
		if (forceTicks || crc != fieldCrc[lcdItem])
		{
			fieldCrc[lcdItem] = crc;
			currentPage->update(lcdItem);
		}
		if (forceTicks)
			--forceTicks;
		if (++lcdItem == lcd_update::update_items)
			lcdItem = 0;
		i2c.pump();
	}

	// I2C queue stuck for 500 ms: reset the driver, then re-init and redraw the LCD.
	if (i2c.queueEmpty())
	{
		idleStuckSince = 0;
	}
	else if (idleStuckSince == 0)
	{
		idleStuckSince = millis();
	}
	else if (millis() - idleStuckSince > 500)
	{
		cli();
		DashI2CDriver fresh;
		memcpy(&i2c, &fresh, sizeof(DashI2CDriver));
		sei();
		idleStuckSince = 0;
		initLcd();
		showPage(currentPageIndex);
	}
}
