#pragma once
#include "FCAN.h"

// All available IDs
// 0x039 - Heartbeat @ ~60hz
// 0x06A -           @ ~140hz
// 0x0A6 -           @ 100hz
// 0x0AA -           @ 100hz
// 0x0AF -           @ 100hz
// 0x0C8 - PcmData,  @ 100hz
// 0x0D4 -           @ 100hz
// 0x12C - PcmData,  @ 100hz
// 0x188 -           @ 100hz
// 0x1C0 -           @ 50hz
// 0x1F4 -           @ 50hz
// 0x20C -           @ 50hz
// 0x224 - PcmData,  @ 10hz
// 0x233 -           @ 10hz
// 0x2E4 -           @ 10hz
// 0x305 -           @ 10hz
// 0x405 -           @ 3hz
// 0x429 -           @ 3hz
// 0x6C1 -           @ 3hz
// 0x6C4 -           @ 3hz
// 0x6D9 -           @ 3hz

static inline void parse_Odyssey2007(uint16_t id, const uint8_t *d, uint8_t dlc, VehicleData &out)
{
    if(dlc != 8) return; // CAN classic, all frames are 8 bytes

    switch (id)
    {
    case 0x0C8: parse_0x0C8(d, out); break;
    case 0x12C: parse_0x12C(d, out); break;
    case 0x224: parse_0x224(d, out); break;
        
    default: break;
    }
}

inline const VehicleProfile Odyssey2007 = {"Honda Odyssey 2007", &parse_Odyssey2007};
