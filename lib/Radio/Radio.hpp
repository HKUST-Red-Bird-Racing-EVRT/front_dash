/**
 * @file Radio.hpp
 * @author ChiHo Ngan, Red Bird Racing
 * @brief HC-12 telemetry transmitter.
 * @version 1.0
 * @date 2026-08-3
 *
 * @copyright Copyright (c) 2026
 */

#ifndef RADIO_HPP
#define RADIO_HPP

#include <stdint.h>
#include <SoftwareSerial.h>
#include "Structs.h"

/**
 * @brief Payload type in RadioFrame::type.
 * @note Placeholder values, confirm with the decoder.
 */
enum class RadioMsgType : uint8_t
{
    Vcu = 0,
    SsruFront = 1,
    SsruRear = 2,
};

constexpr uint8_t RADIO_FRAME_START_BYTE = 0x7E; ///< TODO: confirm with decoder
constexpr uint32_t RADIO_BAUD = 115200;          ///< HC-12 UART baud

/**
 * @brief Sends RadioFrame packets to the ground station over an HC-12 in transparent mode.
 * @note Placeholder, not used yet.
 */
class Radio
{
public:
    /** @param serial SoftwareSerial wired to the HC-12. */
    explicit Radio(SoftwareSerial &serial);

    /** @brief Open the UART at RADIO_BAUD. */
    void begin();

    /**
     * @brief Send one frame: [start][time 4][type 1][data 24][checksum 1].
     * @param frame Frame to send.
     */
    void send(const RadioFrame &frame);

    /** @brief Send the VCU telemetry. TODO: payload. */
    void updateVcu();

    /** @brief Send the front SSRU telemetry. TODO: payload. */
    void updateSsruFront();

    /** @brief Send the rear SSRU telemetry. TODO: payload. */
    void updateSsruRear();

    /** @brief Pull SET low to enter AT command mode. Bench use only. */
    void enterConfigMode();

    /** @brief Release SET to return to transparent mode. */
    void exitConfigMode();

private:
    /** @return 8-bit sum of time, type and data. */
    static uint8_t checksum(const RadioFrame &frame);

    SoftwareSerial &hc12;
};

#endif // RADIO_HPP