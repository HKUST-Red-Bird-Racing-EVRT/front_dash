/**
 * @file SD.hpp
 * @author ChiHo Ngan, Red Bird Racing
 * @brief Raw CAN frame logger to the SD card.
 * @details Each boot opens a new LOGnnn.BIN. A record is 14 bytes, little-endian:
 * [millis u32][id u16][data 8]. id = 11-bit CAN id | (bus << 12), bus 0 = VCU,
 * 1 = SSRU, 2 = CAN2. Decode with tools/decode_log.py.
 * @version 1.0
 * @date 2026-10-27
 */
#ifndef SD_HPP
#define SD_HPP

#include <Arduino.h>
#include <SD.h>
#include <can.h>

/**
 * @brief Double-buffered SD logger.
 * @details log() only copies into the active buffer. When it fills, the buffers swap
 * and service() writes the full one from the main loop. Everything is flushed once
 * per FLUSH_INTERVAL_MS.
 */
class SdLogger
{
public:
    static constexpr uint16_t FLUSH_INTERVAL_MS = 1000; ///< Max data lost on a power cut
    static constexpr uint8_t RECORDS_PER_BUF = 16;      ///< Limited by the 328P's 2 KB RAM

    /** @brief One logged CAN frame. */
    struct __attribute__((packed)) Record
    {
        uint32_t ms;     ///< millis() at reception
        uint16_t id;     ///< CAN id | (bus << 12)
        uint8_t data[8]; ///< Frame payload
    };

    /**
     * @brief Mount the card and open the next free LOGnnn.BIN.
     * @param cs SD chip-select pin.
     * @return true if a log file is open.
     */
    bool begin(uint8_t cs)
    {
        if (!SD.begin(cs))
            return false;
        char name[] = "LOG000.BIN";
        for (uint16_t i = 0; i < 1000; ++i)
        {
            name[3] = '0' + i / 100;
            name[4] = '0' + i / 10 % 10;
            name[5] = '0' + i % 10;
            if (!SD.exists(name))
            {
                file = SD.open(name, FILE_WRITE);
                ok = file;
                return ok;
            }
        }
        return false;
    }

    /**
     * @brief Update the power-fail state. Call every loop.
     * @details On power loss both buffers are written and flushed once, then logging
     * pauses until power returns.
     * @param powerOk Power-fail input level.
     */
    void setPowerOk(bool powerOk)
    {
        if (!powerOk && !paused)
            flushAll();
        paused = !powerOk;
    }

    /**
     * @brief Copy a frame into the active buffer. Does not touch SPI.
     * @param f Received frame.
     * @param bus CAN bus index.
     */
    void log(const can_frame &f, uint8_t bus)
    {
        if (!ok || paused)
            return;
        if (count[active] == RECORDS_PER_BUF)
        {
            if (pending)
            {
                ++dropped;
                return;
            }
            pending = true;
            active ^= 1;
        }
        Record &r = buf[active][count[active]++];
        r.ms = millis();
        r.id = static_cast<uint16_t>((f.can_id & CAN_SFF_MASK) | (bus << 12));
        memcpy(r.data, f.data, 8);
    }

    /** @brief Write a full buffer and flush on the interval. Call once per loop. */
    void service()
    {
        if (!ok || paused)
            return;
        if (pending)
            writeBuf(active ^ 1);
        if (millis() - lastFlush >= FLUSH_INTERVAL_MS)
            flushAll();
    }

    bool ok = false;      ///< File open and no write error
    bool paused = false;  ///< Power fail active
    uint16_t dropped = 0; ///< Frames lost because both buffers were full

private:
    /** @brief Write buffer i to the file. A write error disables logging. */
    void writeBuf(uint8_t i)
    {
        size_t n = count[i] * sizeof(Record);
        if (n && file.write(reinterpret_cast<const uint8_t *>(buf[i]), n) != n)
            ok = false;
        count[i] = 0;
        if (i != active)
            pending = false;
    }

    /** @brief Write both buffers, oldest first, and flush the file. */
    void flushAll()
    {
        if (!ok)
            return;
        if (pending)
            writeBuf(active ^ 1);
        writeBuf(active);
        lastFlush = millis();
        if (ok)
            file.flush();
    }

    Record buf[2][RECORDS_PER_BUF]; ///< A/B buffers
    uint8_t count[2] = {0, 0};      ///< Records in each buffer
    uint8_t active = 0;             ///< Buffer log() writes into
    bool pending = false;           ///< buf[active ^ 1] is full, waiting for service()
    File file;                      ///< Open log file
    uint32_t lastFlush = 0;         ///< millis() of the last flush
};

#endif // SD_HPP
