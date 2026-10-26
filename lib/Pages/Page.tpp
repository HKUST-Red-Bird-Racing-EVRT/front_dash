/**
 * @file Page.tpp
 * @author ChiHo Ngan, Red Bird Racing
 * @brief Telemetry declarations, BMS byte layout and formatting helpers shared by the pages.
 * @version 1.1
 * @date 2026-09-30
 * @see Page.hpp
 */
#include "namespace.h"
#include <Arduino.h>
#include "Dash_I2C.hpp"
#include <mcp2515.h>
#include "DashState.hpp"
#include "Structs.h"

extern int16_t torque_val, motor_rpm;
extern uint16_t motor_warn, motor_error; ///< Latched while driving, cleared by the encoder button
extern uint32_t odometer_integral;       ///< Sum of RPM per LCD tick

/** @brief Telemetry sources with a timeout, index into lastRxMs. */
enum class Src : uint8_t
{
	Pedal, /**< 0x700 */
	Motor, /**< 0x701 */
	Bms    /**< 0x710 */
};
extern uint32_t lastRxMs[3];
constexpr uint16_t STALE_MS_VCU = 500;  ///< Timeout for 0x700/0x701 (sent every 100 ms)
constexpr uint16_t STALE_MS_BMS = 2500; ///< Timeout for 0x710 (sent every 1000 ms)

/** @return true if `s` has not sent a frame within its timeout. */
inline bool stale(Src s)
{
	return millis() - lastRxMs[static_cast<uint8_t>(s)] > (s == Src::Bms ? STALE_MS_BMS : STALE_MS_VCU);
}
constexpr uint8_t ODO_NUM_DIGITS = 6;
constexpr uint8_t ODO_DECIMAL_PLACES = rpm_calc::NUM_DECIMAL_PLACE;
constexpr uint8_t ODO_STR_LENGTH = ODO_NUM_DIGITS + 1 + 1; ///< Digits + '.' + NUL
constexpr uint8_t ODO_POS_OFFSET = ODO_STR_LENGTH + 1 + 2;
constexpr uint8_t STR_START_POS = 21 - ODO_POS_OFFSET;
extern const byte CHAR_LOCKED;
extern const byte CHAR_DEG;
extern const byte CHAR_PWR;
extern uint8_t pot_buffer[8]; ///< 0x730 bytes 0-3 + 0x750 bytes 0-3

namespace
{
	/** @return Uppercase hex digit for n (0-15). */
	inline char hexNibble(uint8_t n)
	{
		return static_cast<char>(n < 10 ? '0' + n : 'A' + n - 10);
	}

	/**
	 * @brief Format a byte as 2 zero-padded hex digits.
	 * @param dest Buffer of at least 3 chars.
	 */
	void formatHexByte(uint8_t value, char *dest)
	{
		dest[0] = hexNibble((value >> 4) & 0x0F);
		dest[1] = hexNibble(value & 0x0F);
		dest[2] = '\0';
	}

	/**
	 * @brief Format a word as 4 zero-padded hex digits.
	 * @param dest Buffer of at least 5 chars.
	 */
	void formatHexWord(uint16_t value, char *dest)
	{
		formatHexByte(static_cast<uint8_t>((value >> 8) & 0xFF), dest);
		formatHexByte(static_cast<uint8_t>(value & 0xFF), dest + 2);
	}



	/**
	 * @brief Format value as " DD" or "-DD".
	 * @note Assumes |value| < 100.
	 * @param dest Buffer of at least 4 chars.
	 */
	void formatSignedInt2(int8_t value, char *dest)
	{
		bool negative = value < 0;
		uint8_t mag = negative ? static_cast<uint8_t>(-value) : static_cast<uint8_t>(value);
		dest[0] = negative ? '-' : ' ';
		dest[1] = static_cast<char>('0' + (mag / 10) % 10);
		dest[2] = static_cast<char>('0' + mag % 10);
		dest[3] = '\0';
	}
	/**
	 * @brief Format tenths as "D.D".
	 * @note Assumes value_deci < 100.
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

	/** @brief Queue a CarStatus as a 5-character word. */
	void printCarStatusQueued(CarStatus status)
	{
		switch (status)
		{
		case CarStatus::Init:
			lcd.printQueued(F("INIT "));
			break;
		case CarStatus::Startin:
			lcd.printQueued(F("STRT "));
			break;
		case CarStatus::Bussin:
			lcd.printQueued(F("BUS  "));
			break;
		case CarStatus::Drive:
			lcd.printQueued(F("DRIVE"));
			break;
		}
	}
}

// TODO: confirm after pedal calibration
constexpr uint16_t THROTTLE_ADC_MIN = 0;
constexpr uint16_t THROTTLE_ADC_MAX = 1023;

// TODO: BMS byte layout of 0x710 is a placeholder
constexpr uint8_t BMS_BYTE_VOLTAGE_MAX = 0; ///< uint16_t, max cell voltage, 0.1 V/count
constexpr uint8_t BMS_BYTE_VOLTAGE_MIN = 2; ///< uint16_t, min cell voltage, 0.1 V/count
constexpr uint8_t BMS_BYTE_CURRENT = 4;     ///< int16_t, pack current, 0.1 A/count, negative = regen
constexpr uint8_t BMS_BYTE_TEMP = 6;        ///< int8_t, max cell temperature, deg C
constexpr uint8_t BMS_BYTE_STATUS = 7;      ///< uint8_t, BMS fault bits

// TODO: confirm against BMS spec
constexpr uint16_t MAX_PACK_CURRENT_DECIAMP = 4000; ///< Full-scale current for the power bar, 0.1 A

/**
 * @brief Queue the throttle position as a 3-digit percentage.
 * @param col Column.
 * @param row Row.
 */
void updatethrottle(uint8_t col, uint8_t row){
	lcd.setCursorQueued(col, row);
	uint16_t raw = car.pedal.apps_5v; // TODO: confirm APPS channel
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
