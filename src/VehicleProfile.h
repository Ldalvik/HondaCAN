#pragma once
#include <stdint.h>
#include "VehicleData.h"

struct VehicleProfile
{
    const char* name;
    void (*parser)(uint16_t id, const uint8_t* data, uint8_t dlc, VehicleData& out);
};