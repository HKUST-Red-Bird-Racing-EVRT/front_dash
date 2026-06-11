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

} ssru_f_730_t;

typedef struct{
    uint8_t imu1_g_y;
    uint8_t imu1_g_z;
    uint8_t imu2_a_x;
    uint8_t imu2_a_y;
    uint8_t imu2_a_z;
    uint8_t imu2_g_x;
    uint8_t imu2_g_y;
    uint8_t imu2_g_z;

} ssru_f_731_t;

typedef struct {
    uint16_t encoder;  
    uint16_t ds18b20;    

} ssru_f_740_t;

#endif