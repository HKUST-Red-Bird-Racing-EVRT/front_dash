
#ifndef STRUCTS_H
#define STRUCTS_H

#include <stdint.h>
#include <LiquidCrystal_I2C.h>
#include "Enums.hpp"


struct RadioFrame{
    uint32_t time;
    uint8_t type;
    uint8_t data[24];
};

struct VcuData{

};

struct SsruFrontData{

};

struct SsruRearData{

};

struct DashData{
    VcuData vcu_data;
    SsruFrontData ssru_front_data;
    SsruRearData ssru_rear_data;
};


extern LiquidCrystal_I2C lcd;

// BMS State
struct BmsData
{
	uint8_t raw_data[8];
	BmsStatus status;
} bms;

class DashState;

// Inline variable declarations

//inline bool motor_warn = false;
//inline bool motor_error = false;







#endif // STRUCTS_H