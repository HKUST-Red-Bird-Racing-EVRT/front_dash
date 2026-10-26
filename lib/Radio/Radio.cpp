/**
 * @file Radio.cpp
 * @author ChiHo Ngan, Red Bird Racing
 * @brief Radio implementation.
 * @version 1.0
 * @date 2026-07-24
 *
 * @copyright Copyright (c) 2026
 * @see Radio.hpp
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

    // TODO: fill frame.data from VcuData

    send(frame);
}

void Radio::updateSsruFront()
{
    RadioFrame frame = {};
    frame.time = millis();
    frame.type = static_cast<uint8_t>(RadioMsgType::SsruFront);

    // TODO: fill frame.data from SsruFrontData

    send(frame);
}

void Radio::updateSsruRear()
{
    RadioFrame frame = {};
    frame.time = millis();
    frame.type = static_cast<uint8_t>(RadioMsgType::SsruRear);

    // TODO: fill frame.data from SsruRearData

    send(frame);
}

void Radio::enterConfigMode()
{
    pinMode(HC12_SET, OUTPUT);
    digitalWrite(HC12_SET, LOW);
    delay(80);
}

void Radio::exitConfigMode()
{
    digitalWrite(HC12_SET, HIGH);
    delay(80);
}
