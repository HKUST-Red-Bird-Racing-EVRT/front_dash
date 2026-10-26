/**
 * @file Structs.h
 * @author ChiHo Ngan, Red Bird Racing
 * @brief Radio frame, telemetry placeholders and BMS state.
 * @version 1.0
 * @date 2026-10-27
 */
#ifndef STRUCTS_H
#define STRUCTS_H

#include <stdint.h>
#include "Dash_I2C.hpp"
#include "Enums.hpp"


/** @brief Packet sent over the HC-12 radio. */
struct RadioFrame{
    uint32_t time;    ///< millis() when sent
    uint8_t type;     ///< RadioMsgType
    uint8_t data[24]; ///< Payload
};

/** @brief TODO: VCU radio payload. */
struct VcuData{

};

/** @brief TODO: front SSRU radio payload. */
struct SsruFrontData{

};

/** @brief TODO: rear SSRU radio payload. */
struct SsruRearData{

};

/** @brief All radio payloads. */
struct DashData{
    VcuData vcu_data;
    SsruFrontData ssru_front_data;
    SsruRearData ssru_rear_data;
};


extern LiquidCrystal_I2C lcd;

/** @brief Last BMS frame and status. */
struct BmsData
{
	uint8_t raw_data[8]; ///< Payload of the last 0x710 frame
	BmsStatus status;    ///< BMS state
};
extern BmsData bms;

class DashState;







#endif // STRUCTS_H