/**
 * @file Page.tpp
 * @author ChiHo
 * @brief Implementation of Page abstract base class and derived classes
 * @version 1.0
 * @date 2026-07-30
 * @see Page.hpp
 */
#include "namespace.h"
#include <Arduino.h>
#include "Page.hpp"
#include "Dash_I2C.hpp"
#include <mcp2515.h>
#include "DashState.hpp"
#include "Structs.h"





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
extern uint8_t pot_buffer[8]; // merged POT data (0x730 front + 0x750 rear), see main.cpp

namespace
{
	/**
	 * @brief Formats a byte as 2 fixed-width uppercase hex digits (zero padded).
	 * @details A fixed width matters here: DashLcd::printQueued(uint16_t, base)
	 * is variable-width (no leading zeros), so if a value shrinks between two
	 * update() calls (e.g. 0x1F2A -> 0x0A), the old wider text partially
	 * "survives" on the LCD as stale leftover characters. Always writing the
	 * same number of characters avoids that.
	 * @param dest Buffer of at least 3 chars.
	 */
	inline char hexNibble(uint8_t n)
	{
		return static_cast<char>(n < 10 ? '0' + n : 'A' + n - 10);
	}

	void formatHexByte(uint8_t value, char *dest)
	{
		// arithmetic, not a lookup table, to keep the 17-byte "0123..F"
		// string out of RAM
		dest[0] = hexNibble((value >> 4) & 0x0F);
		dest[1] = hexNibble(value & 0x0F);
		dest[2] = '\0';
	}

	/**
	 * @brief Formats a uint16_t as 4 fixed-width uppercase hex digits. @see formatHexByte
	 * @param dest Buffer of at least 5 chars.
	 */
	void formatHexWord(uint16_t value, char *dest)
	{
		formatHexByte(static_cast<uint8_t>((value >> 8) & 0xFF), dest);
		formatHexByte(static_cast<uint8_t>(value & 0xFF), dest + 2);
	}

	/**
	 * @brief Formats value_centi (hundredths) as a fixed-width "D.DD" string.
	 * @note Assumes a single whole digit is enough (value_centi < 1000), which
	 * fits a Li-ion cell voltage (e.g. 0..4.20V); revisit if used for a pack
	 * voltage instead.
	 * @param dest Buffer of at least 5 chars.
	 */
	void formatFixedPoint2(uint16_t value_centi, char *dest)
	{
		uint16_t whole = value_centi / 100;
		uint16_t frac = value_centi % 100;
		dest[0] = static_cast<char>('0' + (whole % 10));
		dest[1] = '.';
		dest[2] = static_cast<char>('0' + (frac / 10));
		dest[3] = static_cast<char>('0' + (frac % 10));
		dest[4] = '\0';
	}

	/**
	 * @brief Formats value_deci (tenths) as a fixed-width "+DD.D"/"-DD.D" string.
	 * @note Assumes |value_deci| < 1000 (two whole digits, e.g. up to +/-99.9A).
	 * @param dest Buffer of at least 6 chars.
	 */
	void formatSignedFixedPoint1(int16_t value_deci, char *dest)
	{
		bool negative = value_deci < 0;
		uint16_t mag = negative ? static_cast<uint16_t>(-value_deci) : static_cast<uint16_t>(value_deci);
		uint16_t whole = mag / 10;
		uint16_t frac = mag % 10;
		dest[0] = negative ? '-' : '+';
		dest[1] = static_cast<char>('0' + (whole / 10) % 10);
		dest[2] = static_cast<char>('0' + whole % 10);
		dest[3] = '.';
		dest[4] = static_cast<char>('0' + frac);
		dest[5] = '\0';
	}

	/**
	 * @brief Formats value as a fixed-width "+DD"/"-DD" string.
	 * @note Assumes |value| < 100.
	 * @param dest Buffer of at least 4 chars.
	 */
	void formatSignedInt2(int8_t value, char *dest)
	{
		bool negative = value < 0;
		uint8_t mag = negative ? static_cast<uint8_t>(-value) : static_cast<uint8_t>(value);
		dest[0] = static_cast<char>('0' + (mag / 10) % 10);
		dest[1] = static_cast<char>('0' + mag % 10);
		dest[2] = '\0';
	}
	/**
	 * @brief Formats value_deci (tenths) as a fixed-width "D.D" string.
	 * @note Unsigned, assumes value_deci < 100 (single whole digit) - matches
	 * the 3-column gap BMSPage::setup() leaves before its static "V".
	 * @param dest Buffer of at least 4 chars.
	 */
	void formatFixedPoint1(uint16_t value_deci, char *dest)
	{
		uint16_t whole = value_deci / 10;
		uint16_t frac = value_deci % 10;
		dest[0] = static_cast<char>('0' + (whole % 10));
		dest[1] = '.';
		dest[2] = static_cast<char>('0' + frac);
		dest[3] = '\0';
	}

	/**
	 * @brief Formats a CarStatus as a fixed-width 5-character word (space padded).
	 * @param dest Buffer of at least 6 chars.
	 */
	void formatCarStatusWord(CarStatus status, char *dest)
	{
		const char *word;
		switch (status)
		{
		case CarStatus::Init:
			word = "INIT ";
			break;
		case CarStatus::Startin:
			word = "STRT ";
			break;
		case CarStatus::Bussin:
			word = "BUS  ";
			break;
		case CarStatus::Drive:
			word = "DRIVE";
			break;
		default:
			word = "N/A  "; // unreachable - car_status is a 2-bit field, all 4 values handled above
			break;
		}
		for (uint8_t i = 0; i < 5; ++i)
		{
			dest[i] = word[i];
		}
		dest[5] = '\0';
	}

	/**
	 * @brief Left-justifies src into dest, space-padding (or truncating) to
	 * exactly `width` characters plus a null terminator.
	 * @details Exists so fixed-width messages (e.g. a fault line that
	 * alternates between two different-length strings) don't need their
	 * padding hand-counted - a mismatch there is exactly the kind of stale-
	 * leftover-character bug described in formatHexByte's note above.
	 * @param dest Buffer of at least width+1 chars.
	 */
	void padToWidth(const char *src, char *dest, uint8_t width)
	{
		uint8_t i = 0;
		while (i < width && src[i] != '\0')
		{
			dest[i] = src[i];
			++i;
		}
		while (i < width)
		{
			dest[i] = ' ';
			++i;
		}
		dest[width] = '\0';
	}
}

// TODO: placeholder ADC values confirm after pedal is calibrated (10-bit ADC, full range).
constexpr uint16_t THROTTLE_ADC_MIN = 0;
constexpr uint16_t THROTTLE_ADC_MAX = 1023;

// TODO: BMS CAN placeholders
// TELEMETRY_BMS_MSG 

constexpr uint8_t BMS_BYTE_VOLTAGE_MAX = 0; // uint16_t, bytes 0-1, max cell voltage, placeholder units: 0.1V/count (fits the 3-column "D.D" gap in BMSPage's layout)
constexpr uint8_t BMS_BYTE_VOLTAGE_MIN = 2; // uint16_t, bytes 2-3, min cell voltage, same units
constexpr uint8_t BMS_BYTE_CURRENT = 4;     // int16_t, bytes 4-5, pack current, placeholder units: 0.1A/count, +discharge/-charge - reserved but not shown: current setup() layout has no labeled space for it
constexpr uint8_t BMS_BYTE_TEMP = 6;        // int8_t, byte 6, max cell temperature in deg C
constexpr uint8_t BMS_BYTE_STATUS = 7;      // uint8_t, byte 7, BMS status/fault bitfield

void DriverPage::updatepwr(){
	// telembms is never populated anywhere (see DefaultPage::update note); read the
	// live global `bms` instead. Byte 2 is a placeholder for pack charge / SOC %.
	uint8_t pct = bms.raw_data[2];
	if (pct > 100)
	{
		pct = 100;
	}
	uint8_t bars = pct / 10; // 0..10 filled cells
	// One setCursor, then 10 back-to-back cells relying on the LCD's auto-increment.
	// Always write the full width so the bar shrinks when the value drops.
	lcd.setCursorQueued(8, 1);
	for (uint8_t i = 0; i < 10; ++i)
	{
		lcd.writeQueued(i < bars ? CHAR_PWR : ' ');
	}
}

void DriverPage::updaterpm(){
	lcd.setCursorQueued(4,1);
	uint16_t rpm = (uint32_t)absU16(motor_rpm) * rpm_calc::MAX_MOTOR_RPM / rpm_calc::MAX_MOTOR_RPM_READING;    // motor rpm
	char rpm_str[5];
	rpm_str[5] = '\0';
	for (uint8_t i = 4; i >= 1; --i)
	{
		rpm_str[i] = (rpm % 10) + '0';
		rpm /= 10;
	}
	lcd.printQueued(rpm_str);
}

void DriverPage::update_car_spd(){
	lcd.setCursorQueued(4,0);
	uint8_t speed = absU16(motor_rpm) / rpm_calc::RPM_TO_KMH_DIVISOR;
	char speed_str[4];
	speed_str[4] = '\0';
	for (uint8_t i = 3; i >= 1; --i)
	{
		speed_str[i] = (speed % 10) + '0';
		speed /= 10;
	}
	lcd.printQueued(speed_str);
}

void DriverPage::updateodo(){
	lcd.setCursorQueued(0, 3);
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
	lcd.printQueued(odometer_str);
}

/**
 * @brief Update the throttle position readout at (col,row).
 * @note File-local (static) helper, not a member: it is shared by more than one
 * page and must not have external linkage. The caller passes the position so the
 * cursor is not hard-coded to one page's layout.
 */
static void updatethrottle(uint8_t col, uint8_t row){
	lcd.setCursorQueued(col, row);
	uint16_t raw = car.pedal.apps_5v; // TODO: confirm APPS channel (5V vs 3.3V)
	uint8_t pct;
	if (raw <= THROTTLE_ADC_MIN)
	{
		pct = 0;
	}
	else if (raw >= THROTTLE_ADC_MAX)
	{
		pct = 100;
	}
	else
	{
		pct = static_cast<uint8_t>(static_cast<uint32_t>(raw - THROTTLE_ADC_MIN) * 100 / (THROTTLE_ADC_MAX - THROTTLE_ADC_MIN));
	}
	char pct_str[4];
	pct_str[3] = '\0';
	pct_str[2] = static_cast<char>('0' + pct % 10);
	pct_str[1] = static_cast<char>('0' + (pct / 10) % 10);
	pct_str[0] = static_cast<char>('0' + (pct / 100) % 10);
	lcd.printQueued(pct_str);
}




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
	lcd.setCursor(7, 3);
	lcd.print("km");
	lcd.setCursor(17,3);
	lcd.print("%");
}

/**
 * @brief Update the driver menu page.
 * Called repeatedly to refresh vehicle data on display.
 * Reads from the shared 'car' telemetry state (see DashState.hpp) and the
 * motor_rpm/torque_val globals.
 * - Update speed
 * - Update RPM
 * - Update odometer
 * - Update status indicators
 */
void DriverPage::update()
{
	lcd.setCursorQueued(9, 0);
    switch (car.pedal.status.bits.car_status)
	{
	    case CarStatus::Init:
		{
		    lcd.writeQueued(CHAR_LOCKED);
			break;
		}
	    case CarStatus::Startin:
		{
			lcd.printQueued("S");
			break;
		}
		case CarStatus::Bussin:
		{
			lcd.printQueued("B");
			break;
		}
		case CarStatus::Drive:
		{
			lcd.printQueued("D");
			break;}
	}
	updateodo();
	update_car_spd();
	updaterpm();
	updatepwr();
}

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
	lcd.setCursor(7,0);
	lcd.print("VCU");
    lcd.setCursor(0, 1);
    lcd.print("ERROR:");
    lcd.setCursor(0, 2);
    lcd.print("STATE:");
	lcd.setCursor(0,3);
	lcd.print("POT:");
	lcd.setCursor(9,3);
	lcd.print("PEDAL:");
}

/**
 * @brief Update the VCU debug page.
 * Called repeatedly to refresh VCU debug data on display.
 * @note The exact byte offsets those CAN messages use are still placeholders -
 */
void VCUPage::update()
{
	char buf[7];

	// Error (row 1 label is "ERROR:")
	lcd.setCursorQueued(6, 1);
	formatHexWord(motor_error, buf);
	lcd.printQueued(buf);

	// VCU State
	lcd.setCursorQueued(6, 2);
	formatCarStatusWord(car.pedal.status.bits.car_status, buf);
	lcd.printQueued(buf);
	lcd.setCursorQueued(12, 2);
	formatHexByte(car.pedal.status.byte, buf);
	lcd.printQueued(buf);

	// POT
	lcd.setCursorQueued(4, 3);
	uint16_t pot_front = static_cast<uint16_t>(pot_buffer[0]) | (static_cast<uint16_t>(pot_buffer[1]) << 8);
	formatHexWord(pot_front, buf);
	lcd.printQueued(buf);

	// Pedal (label "PEDAL:" is at cols 9-14, value goes at col 15)
	updatethrottle(15, 3);
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
	lcd.setCursor(3,2);
	lcd.write(CHAR_DEG);
    lcd.setCursor(0, 3);
    lcd.print("MAX:");
	lcd.setCursor(7,3);
	lcd.print("V");
	lcd.setCursor(10, 3);
	lcd.print("MIN:");
	lcd.setCursor(17, 3);
	lcd.print("V");
}

/**
 * @brief Update the BMS debug page.
 * Called repeatedly to refresh BMS debug data on display.
 */
void BMSPage::update()
{
	// bms.raw_data is filled from the last received CAN frame; decode multi-byte fields little-endian.
	uint16_t voltage_max = static_cast<uint16_t>(bms.raw_data[BMS_BYTE_VOLTAGE_MAX]) |
	                        (static_cast<uint16_t>(bms.raw_data[BMS_BYTE_VOLTAGE_MAX + 1]) << 8);
	uint16_t voltage_min = static_cast<uint16_t>(bms.raw_data[BMS_BYTE_VOLTAGE_MIN]) |
	                        (static_cast<uint16_t>(bms.raw_data[BMS_BYTE_VOLTAGE_MIN + 1]) << 8);
	int8_t temp = static_cast<int8_t>(bms.raw_data[BMS_BYTE_TEMP]);
	uint8_t status_fault = bms.raw_data[BMS_BYTE_STATUS];

	char buf[6];

	// Warning
	lcd.setCursorQueued(6, 1);
	if (status_fault == 0)
	{
		lcd.printQueued("OK");
	}
	else
	{
		formatHexByte(status_fault, buf);
		lcd.printQueued(buf);
	}

	// --- Temperature
	lcd.setCursorQueued(0, 2);
	formatSignedInt2(temp, buf);
	lcd.printQueued(buf);

	// BMS status - fixed 3-wide, queued (was a blocking lcd.print("N/A") that also
	// left "/A" behind when switching to a 1-char state).
	lcd.setCursorQueued(5, 2);
	switch (bms.status)
	{
	case BmsStatus::NoMsg:
		lcd.printQueued("N/A");
		break;
	case BmsStatus::Waiting:
		lcd.printQueued("W  ");
		break;
	case BmsStatus::Starting:
		lcd.printQueued("S  ");
		break;
	case BmsStatus::Started:
		lcd.printQueued("R  ");
		break;
	default:
		lcd.printQueued("?  ");
		break;
	}

	// Voltage
	lcd.setCursorQueued(4, 3);
	formatFixedPoint1(voltage_max, buf);
	lcd.printQueued(buf);
	lcd.setCursorQueued(14, 3);
	formatFixedPoint1(voltage_min, buf);
	lcd.printQueued(buf);
}

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
 * @brief Update the default/status page.
 * Called repeatedly to refresh status data on display.
 * @note Matches the setup() layout: BRAKE (brake fault state), BUTTON (car
 * status word - there's no raw start-button pin state exposed on car.pedal,
 * so this shows the state machine instead; Startin literally means "driver
 * holds Start button", see Enums.hpp), HV (ready flag), and a fault summary
 * on row 3 (unused by setup(), so it has the full 20 columns to itself).
 */
void DefaultPage::update()
{
	char buf[19];

	// --- Brake fault state ---
	lcd.setCursorQueued(6, 0);
	if (car.pedal.faults.bits.brake_high)
	{
		lcd.printQueued("HIGH");
	}
	else if (car.pedal.faults.bits.brake_low)
	{
		lcd.printQueued("LOW ");
	}
	else
	{
		lcd.printQueued("OK  ");
	}

	// --- Start button / car status word ---
	lcd.setCursorQueued(7, 1);
	formatCarStatusWord(car.pedal.status.bits.car_status, buf);
	lcd.printQueued(buf);

	// --- HV ready ---
	// NOTE: this used to compare against telembms.bms_data[6], but telembms
	// is never written anywhere in main.cpp (always 0), which made this
	// backwards and effectively random. Reading hv_ready directly instead.
	lcd.setCursorQueued(3, 2);
	if (car.pedal.status.bits.hv_ready)
	{
		lcd.printQueued("Y");
	}
	else
	{
		lcd.printQueued("N");
	}

	// --- Fault summary ---
	lcd.setCursorQueued(0, 3);
	if (car.pedal.faults.bits.fault_active)
	{
		padToWidth("Pedal/Brake error", buf, 18);
	}
	else
	{
		padToWidth("no Error", buf, 18);
	}
	lcd.printQueued(buf);
}
