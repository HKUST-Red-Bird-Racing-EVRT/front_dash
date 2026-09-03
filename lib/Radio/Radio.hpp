/**
 * @file Radio.hpp
 * @author ChiHo Ngan, Red Bird Racing 
 * @brief HC-12 telemetry transmitter implementation.
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
 * @brief Message type identifier carried in RadioFrame::type, so the ground
 *        station knows which DashData member the payload bytes belong to.
 * @note Placeholder values - confirm with decoder for values.
 */
enum class RadioMsgType : uint8_t
{
    Vcu = 0,
    SsruFront = 1,
    SsruRear = 2,
};

constexpr uint8_t RADIO_FRAME_START_BYTE = 0x7E; ///< TODO: confirm with decoder
constexpr uint32_t RADIO_BAUD = 115200;            ///< HC-12 default transparent-mode baud

/**
 * @brief Packages telemetry into RadioFrame packets and transmits them to the
 *        ground station over an HC-12 433MHz transceiver in transparent
 *        (UART passthrough) mode.
 *
 * @details The dashboard only serializes and sends raw telemetry - any
 * decoding/decryption happens on the receiving computer.
 *
 * @note Everything below is currently a placeholder
 */
class Radio
{
public:
    /**
     * @param serial Reference to the SoftwareSerial instance wired to the HC-12's RXD/TXD pins.
     */
    explicit Radio(SoftwareSerial &serial);

    /**
     * @brief Opens the UART link to the HC-12 at RADIO_BAUD. Call once from setup().
     */
    void begin();

    /**
     * @brief Serializes and transmits a single RadioFrame over the HC-12 link.
     * @param frame The frame to send. frame.time should be filled in by the caller (e.g. millis()).
     */
    void send(const RadioFrame &frame);

    /**
     * @brief Packages the latest VCU telemetry into a RadioFrame and sends it.
     * @note VcuData is currently an empty placeholder struct (see Structs.h) -
     * fill in its fields (from the same 'car'/torque_val/motor_rpm state the
     * LCD pages read) and this function's body once VCU CAN signals are confirmed.
     */
    void updateVcu();

    /// @brief Packages and sends the latest front SSRU telemetry. @see updateVcu
    void updateSsruFront();

    /// @brief Packages and sends the latest rear SSRU telemetry. @see updateVcu
    void updateSsruRear();

    /**
     * @brief Pulls the HC-12's SET pin low, putting it into AT command mode so
     * configuration commands (channel/power/baud) can be sent over `serial`.
     * @note Only for a one-off bench configuration session - do not call this
     * from loop(), the module drops out of transparent mode while in AT mode.
     */
    void enterConfigMode();

    /// @brief Returns the HC-12 to normal transmit mode after enterConfigMode().
    void exitConfigMode();

private:
    static uint8_t checksum(const RadioFrame &frame);

    SoftwareSerial &hc12;
};

#endif // RADIO_HPP