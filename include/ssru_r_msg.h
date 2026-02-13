#ifndef SSRU_F_MSG_H
#define SSRU_F_MSG_H

#include <stdint.h>

typedef struct{
    uint16_t pot_fl_in;
    uint16_t pot_fr_in;
    uint8_t imu1_a_x;
    uint8_t imu1_a_y;
    uint8_t imu1_a_z;
    uint8_t imu1_g_x;

} ssru_r_750_t;

typedef struct{
    uint8_t imu1_g_y;
    uint8_t imu1_g_z;
    uint8_t pump_status;
    uint8_t pump_mode;

} ssru_r_751_t;

typedef struct{
    uint16_t braketemp1_in;
    uint16_t braketemp2_in;
    uint16_t padding;
    uint16_t pump_ntc_50k_in;
} ssru_r_752_t;

typedef struct{
    uint16_t flow_in;
} ssru_r_753_t;

#endif