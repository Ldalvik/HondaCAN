#pragma once
#include <stdint.h>

// Conversion helpers
static constexpr float KM_TO_MI = 0.621371f;
static constexpr float M_TO_MI = 0.000621371f;
static constexpr float L_TO_GAL = 0.264172f;
static constexpr float RAW_TO_GAL = 0.0000264172f; // Not sure if correct, liters?

// Single bit flag
static inline uint8_t bit_U8(uint8_t byte, uint8_t index)
{
    return (byte >> index) & 0x01; //  #define bit(data, n) (((data) >> (n)) & 0x01)
}

// Unsigned 16-bit big-endian
static inline uint16_t U16(const uint8_t *d)
{
    return ((uint16_t)d[0] << 8) | d[1];
}

// Signed 16-bit big-endian
static inline int16_t I16(const uint8_t *d)
{
    return (int16_t)U16(d);
}

// Unsigned 24-bit big-endian
static inline uint32_t U24(const uint8_t *d)
{
    return ((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2];
}

static inline bool validateFrame(uint16_t id, const uint8_t *d, uint8_t dlc, uint8_t expectedDlc,
                   uint8_t *lastCounter, bool *firstFrame)
{
    if (dlc != expectedDlc)
    {
        Serial.println("DLC fail");
        return false;
    }

    int sum = 0;

    while (id)
    {
        sum += id & 0x0F;
        id >>= 4;
    }

    for (uint8_t i = 0; i < dlc - 1; i++)
        sum += (d[i] & 0x0F) + (d[i] >> 4);

    uint8_t counter = d[dlc - 1] >> 4;
    uint8_t checksum = d[dlc - 1] & 0x0F;
    uint8_t expectedChecksum = (uint8_t)((8 - (sum + counter)) & 0x0F);

    if (checksum != expectedChecksum)
    {
        Serial.println("Checksum fail");
        return false;
    }

    // not sure if all frames use mod 4
    bool counterOk = *firstFrame || (counter == (uint8_t)((*lastCounter + 1) & 0x03));
    *lastCounter = counter;
    *firstFrame = false;

    if (!counterOk) Serial.println("Counter fail");
    return counterOk;
}

static inline bool validateFrameVSA(const uint8_t *d, uint8_t dlc, uint8_t *lastCounter, bool *firstFrame)
{
    if (dlc != 8)
    {
        Serial.println("DLC fail");
        return false;
    }

    uint8_t sum = 0;
    for (uint8_t i = 0; i < 8; i++)
        sum += d[i];

    if (sum != 0)
    {
        Serial.println("Checksum fail");
        return false;
    }

    bool counterOk = *firstFrame || (d[6] == (uint8_t)(*lastCounter + 1));
    *lastCounter = d[6];
    *firstFrame = false;

    if (!counterOk) Serial.println("Counter fail");
    return counterOk;
}