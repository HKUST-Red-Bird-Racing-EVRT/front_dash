#ifndef SSRU_R_MSG_H
#define SSRU_R_MSG_H

#include <stdint.h>

typedef struct{
    uint16_t pot1_in;
    uint16_t pot2_in;

} ssru_r_750_t;

typedef struct{
    int8_t  accel_x;
    int8_t  accel_y;
    int8_t  accel_z;
    int8_t  gyro_x;
    int8_t  gyro_y;
    int8_t  gyro_z;

} ssru_r_751_t;

typedef struct{
    uint8_t  pump_status;
    uint8_t  pump_mode;
    uint16_t flow_in;   
    uint16_t ds18b20_temp;  
    uint16_t pump_ntc_50k_in;

} ssru_r_760_t;

#endif