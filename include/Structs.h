

#include <stdint.h>

#include "ssru_f_msg.h"
#include "ssru_r_msg.h"

struct RadioFrame{
    uint32_t time;
    uint8_t type;
    uint8_t data[24];
};

struct VcuData {

    // 0x700 - VCU_PEDALS_100MS
    struct PedalTelemetry {
        uint16_t apps_5v;
        uint16_t apps_3v3;
        uint16_t brake;
        uint16_t hall_sensor; 

        union {
            uint8_t byte;
            struct {
                uint8_t car_status     : 2;  
                uint8_t state_unknown  : 1;  
                uint8_t hv_ready       : 1;  
                uint8_t bms_no_msg     : 1;  
                uint8_t motor_no_read  : 1; 
                uint8_t screenshot     : 1;
                uint8_t force_stop     : 1; 
            } bits;
        } status;

        union {
            uint8_t byte;
            struct {
                uint8_t fault_active    : 1;  
                uint8_t fault_exceeded  : 1;  
                uint8_t apps_5v_low     : 1;
                uint8_t apps_5v_high    : 1;
                uint8_t apps_3v3_low    : 1;
                uint8_t apps_3v3_high   : 1;
                uint8_t brake_low       : 1;
                uint8_t brake_high      : 1;
            } bits;
        } faults;
    } pedal;

    // 0x701 - VCU_MOTOR_100MS
    struct MotorTelemetry {
        int16_t  torque_val;  
        uint16_t motor_rpm; 
        uint16_t motor_error; 
        uint16_t motor_warn;
    } motor;

    // 0x710 - VCU_BMS_1000MS
    struct BmsTelemetry {
        uint8_t data[8];
    } bms;
};

struct SsruFrontData{
    ssru_f_730_t frame730;
    ssru_f_731_t frame731;
    ssru_f_740_t frame740;
};

struct SsruRearData{
    ssru_r_750_t frame750;
    ssru_r_751_t frame751; 
    ssru_r_760_t frame760;
};

struct DashData{
    VcuData vcu_data;
    SsruFrontData ssru_front_data;
    SsruRearData ssru_rear_data;
};