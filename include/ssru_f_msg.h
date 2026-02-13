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

} ssru_f_700_t;

typedef struct{
    uint8_t imu1_g_y;
    uint8_t imu1_g_z;
    uint8_t imu2_a_x;
    uint8_t imu2_a_y;
    uint8_t imu2_a_z;
    uint8_t imu2_g_x;
    uint8_t imu2_g_y;
    uint8_t imu2_g_z;

} ssru_f_701_t;

typedef struct{
    uint16_t encoder;
} ssru_f_702_t;

typedef struct{
    uint16_t braketemp1_in;
    uint16_t braketemp2_in;
    uint16_t whatever_temp;
} ssru_f_703_t;

#endif