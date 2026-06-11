
// ignore -Wpedantic warnings for mcp2515.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <mcp2515.h>
#pragma GCC diagnostic pop

#include "Structs.h"

class CanManager {
public:
    CanManager(MCP2515& can_vcu, MCP2515& can_ssru);
    ~CanManager() = default;

    void readVCU(DashData& dash_storage);
    void readSSRU(DashData& dash_storage);

    void packPedalFrame(const VcuData::PedalTelemetry& pedal, uint8_t frame[8]);
    void sendPedalFrame(const VcuData::PedalTelemetry& pedal);
    void sendMotorFrame(const VcuData::MotorTelemetry& motor);
    void sendBmsFrame(const VcuData::BmsTelemetry& bms);

private:
    MCP2515& can_vcu;
    MCP2515& can_ssru;
};