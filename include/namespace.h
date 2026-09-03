#ifndef FRONT_DASH_NAMESPACE_H
#define FRONT_DASH_NAMESPACE_H

#include <Arduino.h>
#include <mcp2515.h>
#include "Dash_I2C.hpp"
#include <stdint.h>
#include <SoftwareSerial.h>

/**
 * @brief |v| for an int16_t, safe against INT16_MIN.
 * @details AVR `int` is 16-bit, so the stdlib/Arduino `abs()` computes `-(-32768)`
 * which overflows (UB, stays negative) and then poisons any following unsigned
 * math. Promote to 32-bit before negating.
 */
inline uint16_t absU16(int16_t v)
{
	return v < 0 ? static_cast<uint16_t>(-static_cast<int32_t>(v))
				 : static_cast<uint16_t>(v);
}

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
#endif // FRONT_DASH_NAMESPACE_H