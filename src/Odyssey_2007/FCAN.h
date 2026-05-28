#pragma once
#include "VehicleProfile.h"
#include "BitHelpers.h"
#include "Enums.h"

static inline void parse_0x0C8(const uint8_t *d, VehicleData &out)
{
    /* EngineRPM - bytes [2..3] */
    out.PCM.EngineRPM = U16(&d[2]);

    /* TripDistance - byte 6 */
    uint8_t tripDistance = d[6];
    if (tripDistance < out.MISC.TripDistanceLast) out.MISC.TripDistanceOverflow++;
    out.MISC.TripDistanceLast = tripDistance;
    
    // ECU only stores 2 bytes of trip data, which is 2.55km
    uint32_t totalDistance = (out.MISC.TripDistanceOverflow * 256U) + tripDistance;
    float meters = totalDistance * 10U;

    out.PCM.TripDistance = meters * M_TO_MI;

    // uint8_t counter = d[7] >> 4;
    // uint8_t checksum = d[7] & 0x0F;
}

static inline void parse_0x12C(const uint8_t *d, VehicleData &out)
{
    /* EngineTemp - byte 0 */
    out.PCM.EngineTemp = d[0] - 40;

    /* IntakeTemp - byte 1 */
    out.PCM.IntakeTemp = d[1] - 40;

    /* EngineRPM - bytes [4..5] */
    uint16_t EngineRPM = U16(&d[4]);
    //out.PCM.EngineRPM = U16(&d[4]);
}

static inline void parse_0x224(const uint8_t *d, VehicleData &out)
{
    // TripFuelConsumed - bytes [2..3]
    out.PCM.TripFuelConsumed = U16(&d[2]) * 0.0001f * L_TO_GAL;
}

