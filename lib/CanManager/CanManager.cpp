#include "CanManager.hpp"
#include <Arduino.h>

// CAN IDs
constexpr uint32_t CAN_VCU_PEDALS   = 0x700;
constexpr uint32_t CAN_VCU_MOTOR    = 0x701;
constexpr uint32_t CAN_VCU_BMS      = 0x710;

constexpr uint32_t CAN_SSRU_F_730   = 0x730;
constexpr uint32_t CAN_SSRU_F_731   = 0x731;
constexpr uint32_t CAN_SSRU_F_740   = 0x740;

constexpr uint32_t CAN_SSRU_R_750   = 0x750;
constexpr uint32_t CAN_SSRU_R_751   = 0x751;
constexpr uint32_t CAN_SSRU_R_760   = 0x760;

CanManager::CanManager(MCP2515& can_vcu, MCP2515& can_ssru)
  : can_vcu(can_vcu), can_ssru(can_ssru)
{}


void CanManager::readVCU(DashData& dash_storage)
{
    struct can_frame frame;
    
    // Drain the hardware buffer until empty for this cycle
    while (can_vcu.readMessage(&frame) == MCP2515::ERROR_OK)
    {
        switch (frame.can_id)
        {
            case CAN_VCU_PEDALS:
            {
                uint16_t v1 = ((uint16_t)frame.data[0] << 2) | ((frame.data[1] & 0xC0) >> 6);
                uint16_t v2 = ((uint16_t)(frame.data[1] & 0x3F) << 4) | ((frame.data[2] & 0xF0) >> 4);
                uint16_t v3 = ((uint16_t)(frame.data[2] & 0x0F) << 6) | ((frame.data[3] & 0xFC) >> 2);
                uint16_t v4 = ((uint16_t)(frame.data[3] & 0x03) << 8) | frame.data[4];

                dash_storage.vcu_data.pedal.apps_5v      = v1;
                dash_storage.vcu_data.pedal.apps_3v3     = v2;
                dash_storage.vcu_data.pedal.brake        = v3;
                dash_storage.vcu_data.pedal.hall_sensor  = v4;

                dash_storage.vcu_data.pedal.status.byte  = frame.data[5];
                dash_storage.vcu_data.pedal.faults.byte  = frame.data[6];
                break;
            }
            case CAN_VCU_MOTOR:
            {
                if (frame.can_dlc == sizeof(VcuData::MotorTelemetry)) 
                {
                    memcpy(&dash_storage.vcu_data.motor, frame.data, sizeof(VcuData::MotorTelemetry));
                }
                break;
            }
            case CAN_VCU_BMS:
            {
                if (frame.can_dlc == sizeof(VcuData::BmsTelemetry)) 
                {
                    memcpy(dash_storage.vcu_data.bms.data, frame.data, sizeof(VcuData::BmsTelemetry));
                }
                break;
            }
            default:
                break;
        }
    }
}

void CanManager::readSSRU(DashData& dash_storage)
{
    struct can_frame frame;

    while (can_ssru.readMessage(&frame) == MCP2515::ERROR_OK)
    {
        switch (frame.can_id)
        {
            // Front SSRU
            case CAN_SSRU_F_730:
                if (frame.can_dlc == sizeof(ssru_f_730_t))
                {
                    memcpy(&dash_storage.ssru_front_data.frame730, frame.data, sizeof(ssru_f_730_t));
                }
                break;
            case CAN_SSRU_F_731:
                if (frame.can_dlc == sizeof(ssru_f_731_t)) 
                {
                    memcpy(&dash_storage.ssru_front_data.frame731, frame.data, sizeof(ssru_f_731_t));
                }
                break;
            case CAN_SSRU_F_740:
                if (frame.can_dlc == sizeof(ssru_f_740_t)) 
                {
                    memcpy(&dash_storage.ssru_front_data.frame740, frame.data, sizeof(ssru_f_740_t));
                }
                break;

            // Rear SSRU
            case CAN_SSRU_R_750:
                if (frame.can_dlc == sizeof(ssru_r_750_t)) 
                {
                    memcpy(&dash_storage.ssru_rear_data.frame750, frame.data, sizeof(ssru_r_750_t));
                }
                break;
            case CAN_SSRU_R_751:
                if (frame.can_dlc == sizeof(ssru_r_751_t)) 
                {
                    memcpy(&dash_storage.ssru_rear_data.frame751, frame.data, sizeof(ssru_r_751_t));
                }
                break;
            case CAN_SSRU_R_760:
                if (frame.can_dlc == sizeof(ssru_r_760_t)) 
                {
                    memcpy(&dash_storage.ssru_rear_data.frame760, frame.data, sizeof(ssru_r_760_t));
                }
                break;

            default:
                break;
        }
    }
}


void CanManager::packPedalFrame(const VcuData::PedalTelemetry &pedal, uint8_t *frame)
{
    memset(frame, 0, 8);

    uint16_t v1 = pedal.apps_5v  & 0x03FF;  
    uint16_t v2 = pedal.apps_3v3 & 0x03FF;
    uint16_t v3 = pedal.brake    & 0x03FF;
    uint16_t v4 = pedal.hall_sensor & 0x03FF;

    frame[0] = (uint8_t)(v1 >> 2);
    frame[1] = (uint8_t)((v1 & 0x03) << 6) | (uint8_t)(v2 >> 4);
    frame[2] = (uint8_t)((v2 & 0x0F) << 4) | (uint8_t)(v3 >> 6);
    frame[3] = (uint8_t)((v3 & 0x3F) << 2) | (uint8_t)(v4 >> 8);
    frame[4] = (uint8_t)(v4 & 0xFF);

    frame[5] = pedal.status.byte;
    frame[6] = pedal.faults.byte;
    frame[7] = 0;
}

void CanManager::sendPedalFrame(const VcuData::PedalTelemetry &pedal)
{
    uint8_t frame[8];
    packPedalFrame(pedal, frame);

    struct can_frame msg;
    msg.can_id  = 0x700;
    msg.can_dlc = 8;
    memcpy(msg.data, frame, 8);

    can_vcu.sendMessage(&msg);
}

void CanManager::sendMotorFrame(const VcuData::MotorTelemetry& motor)
{
    struct can_frame msg;
    msg.can_id  = 0x701;
    msg.can_dlc = 8;    
    memcpy(msg.data, &motor, 8); 

    can_vcu.sendMessage(&msg);
}

void CanManager::sendBmsFrame(const VcuData::BmsTelemetry& bms)
{
    struct can_frame msg;
    msg.can_id  = 0x710;
    msg.can_dlc = 8;
    memcpy(msg.data, bms.data, 8);        

    can_vcu.sendMessage(&msg);
}