/**
 * @file Radio.cpp
 * @author ChiHo Ngan, Red Bird Racing 
 * @brief HC-12 telemetry transmitter implementation.
 * @version 1.0
 * @date 2026-07-24
 * 
 * @copyright Copyright (c) 2026
 * @see Radio.hpp for the overall design notes / placeholder disclaimers.
 */

#include "Radio.hpp"
#include <Arduino.h>
#include "pinMap.h"

Radio::Radio(SoftwareSerial &serial) : hc12(serial)
{
}

void Radio::begin()
{
    hc12.begin(RADIO_BAUD);
}

/**
 * @brief Simple additive checksum over the frame contents.
 */
uint8_t Radio::checksum(const RadioFrame &frame)
{
    uint8_t sum = frame.type;
    sum += static_cast<uint8_t>(frame.time & 0xFF);
    sum += static_cast<uint8_t>((frame.time >> 8) & 0xFF);
    sum += static_cast<uint8_t>((frame.time >> 16) & 0xFF);
    sum += static_cast<uint8_t>((frame.time >> 24) & 0xFF);
    for (uint8_t i = 0; i < sizeof(frame.data); ++i)
    {
        sum += frame.data[i];
    }
    return sum;
}

/**
 * @brief Wire format (all placeholder)
 * [START_BYTE][time: 4 bytes, little-endian][type: 1 byte][data: 24 bytes][checksum: 1 byte]

 */
void Radio::send(const RadioFrame &frame)
{
    hc12.write(RADIO_FRAME_START_BYTE);
    hc12.write(reinterpret_cast<const uint8_t *>(&frame.time), sizeof(frame.time));
    hc12.write(frame.type);
    hc12.write(frame.data, sizeof(frame.data));
    hc12.write(checksum(frame));
}

void Radio::updateVcu()
{
    RadioFrame frame = {};
    frame.time = millis();
    frame.type = static_cast<uint8_t>(RadioMsgType::Vcu);

    // TODO: VcuData is an empty placeholder in Structs.h until the VCU CAN
    // signal mapping is confirmed.
    //
    //   VcuData vcu_data = { ... };
    //   static_assert(sizeof(vcu_data) <= sizeof(frame.data)
    //   memcpy(frame.data, &vcu_data, sizeof(vcu_data));

    send(frame);
}

void Radio::updateSsruFront()
{
    RadioFrame frame = {};
    frame.time = millis();
    frame.type = static_cast<uint8_t>(RadioMsgType::SsruFront);

    // TODO:SsruFrontData is currently template

    send(frame);
}

void Radio::updateSsruRear()
{
    RadioFrame frame = {};
    frame.time = millis();
    frame.type = static_cast<uint8_t>(RadioMsgType::SsruRear);

    // TODO:SsruRearData is currently empty. See ssru_r_msg.h.

    send(frame);
}

void Radio::enterConfigMode()
{
    pinMode(HC12_SET, OUTPUT);
    digitalWrite(HC12_SET, LOW);
    delay(80); // HC-12 needs >40ms to switch into AT mode?
}

void Radio::exitConfigMode()
{
    digitalWrite(HC12_SET, HIGH);
    delay(80);
}
