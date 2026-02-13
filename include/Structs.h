

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