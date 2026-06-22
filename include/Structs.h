

#include <stdint.h>

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


#include "Enums.hpp"
#include "Page.hpp"
#include "DashState.hpp"

extern LiquidCrystal_I2C lcd(0x27, 20, 4);

// BMS State
struct BmsData
{
	uint8_t raw_data[8];
	BmsStatus status;
} bms;

// Car State
DashState dashState;

// Inline variable declarations
inline int motor_rpm = 0;
inline int torque_val = 0;
inline bool motor_warn = false;
inline bool motor_error = false;
inline long odometer_integral = 0;

// Page instances
DriverPage driverPage(lcd, dashState);
VCUPage vcuPage(lcd, dashState);
BMSPage bmsPage(lcd, dashState);
ReservedPage reservedPage(lcd, dashState);

// Current page pointer
Page* currentPage = &driverPage;





