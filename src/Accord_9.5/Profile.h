#pragma once
#include "FCAN.h"
#include "BCAN.h"

static inline void parse_Accord_9_5(uint16_t id, const uint8_t *d, uint8_t dlc, VehicleData &out)
{
    switch (id)
    {
    /* FCAN frames*/
    // case 0x039: parse_0x039(d, out); break;
    // case 0x091: parse_0x091(d, out); break;
    case 0x13C:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 8, &last, &first)) parse_0x13C(d, out);
        break;
    }
    case 0x156:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 6, &last, &first)) parse_0x156(d, out);
        break;
    }
    case 0x158:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 8, &last, &first)) parse_0x158(d, out);
        break;
    }
    case 0x17C:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 8, &last, &first)) parse_0x17C(d, out);
        break;
    }
    case 0x18E:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 3, &last, &first)) parse_0x18E(d, out);
        break;
    }
    case 0x191:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 8, &last, &first)) parse_0x191(d, out);
        break;
    }
    case 0x1A4:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 8, &last, &first)) parse_0x1A4(d, out);
        break;
    }
    case 0x1A6:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 8, &last, &first)) parse_0x1A6(d, out);
        break;
    }
    case 0x1B0:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 7, &last, &first)) parse_0x1B0(d, out);
        break;
    }
    case 0x1DC:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 4, &last, &first)) parse_0x1DC(d, out);
        break;
    }
    case 0x1EA:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 8, &last, &first)) parse_0x1EA(d, out);
        break;
    }
    case 0x221:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 8, &last, &first)) parse_0x221(d, out);
        break;
    }
    case 0x255:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrameVSA(d, dlc, &last, &first)) parse_0x255(d, out);
        break;
    }
    case 0x294:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 8, &last, &first)) parse_0x294(d, out);
        break;
    }
    case 0x295:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 4, &last, &first)) parse_0x295(d, out);
        break;
    }
    case 0x305:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 7, &last, &first)) parse_0x305(d, out);
        break;
    }

    case 0x309:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 8, &last, &first)) parse_0x309(d, out);
        break;
    }
    case 0x320:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 8, &last, &first)) parse_0x320(d, out);
        break;
    }
    case 0x324:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 8, &last, &first)) parse_0x324(d, out);
        break;
    }
    case 0x372:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 2, &last, &first)) parse_0x372(d, out);
        break;
    }
    case 0x3A1:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 8, &last, &first)) parse_0x3A1(d, out);
        break;
    }
    case 0x405:
    {
        static uint8_t last = 0;
        static bool first = true;
        if (validateFrame(id, d, dlc, 8, &last, &first)) parse_0x405(d, out);
        break;
    }

    /* BCAN frames below here*/

    default:
        break;
    }
}

inline const VehicleProfile Accord_9_5 = {"Honda Accord 9.5th Gen (2016-17)", &parse_Accord_9_5};
