// Documentation byte layout
// Can be used to store data if done correctly- haven't tested frames that arent 8 bytes yet
// Must use be16toh from <endian.h> on most values
#pragma once
#include <cstdint>

typedef struct __attribute__((packed))
{
    uint8_t byte0; // changes?
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} PCM_0x039;

typedef struct __attribute__((packed))
{
    uint8_t byte0; // sits at 255
    uint8_t byte1; // sits at 128
    uint8_t byte2; // occasionally jumps to 1 or 2
    uint8_t byte3; // occasionally jumps to numbers between 0 and 10
    uint8_t byte4; // wide jumps to 10 and drops to 5
    uint8_t byte5; // sits at 225
    uint8_t byte6; // frequent movement mid-range
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} XXX_0x06A;

typedef struct __attribute__((packed))
{
    uint8_t byte0; // sits at 0, jumps up to 2 with byte1
    uint8_t byte1; // jumps up to 128 with byte0
    uint8_t byte2; // frequent movement, matches byte3
    uint8_t byte3; // frequent movement, matches byte2
    uint8_t byte4; // matches byte2/3, but lower, sits at 0 during lower peaks
    uint8_t byte5; // frequent movement
    uint8_t byte6; // frequent movement, just spiking
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} XXX_0x0A6;

typedef struct __attribute__((packed))
{
    uint8_t GasPedal;
    uint8_t byte1; // sits at 0
    uint8_t byte2; // sits at 0
    uint8_t byte3; // sits at 0
    uint8_t byte4; // sits at 0
    uint8_t byte5; // sits at 0
    uint8_t byte6; // sits at 0
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} PCM_0x0AA;

typedef struct __attribute__((packed))
{
    uint16_t bytes0_1; // everywhere
    uint16_t byte2_3;  // everywhere
    uint8_t byte4;     // jumps from 0 to 255, correlates with byte5
    uint8_t byte5;     // sporadic but seems to have a flow sometimes
    uint8_t byte6;     // sits at 0
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} XXX_0x0AF;

typedef struct __attribute__((packed))
{
    uint16_t bytes0_1; // speed?
    int16_t EngineRPM;
    uint16_t bytes4_5; // tachometer?
    uint8_t TripDistance;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} PCM_0x0C8;

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
} XXX_0x0D4;

typedef struct __attribute__((packed))
{
    uint8_t EngineTemp;
    uint8_t IntakeTemp;
    uint8_t byte2; // similar to 0x0AA byte0, and 0x0A6 byte 3 and 4, possibly filtered versions
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} PCM_0x12C;

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
} XXX_0x188;

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
} XXX_0x1C0;

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
} XXX_0x1F4;

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
} XXX_0x20C;

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t byte1;
    uint16_t TripFuelConsumed;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} PCM_0x224;

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
} XXX_0x233;

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
} XXX_0x2E4;

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
} XXX_0x305;

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
} XXX_0x405;

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
} XXX_0x429;

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
} XXX_0x6C1;

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
} XXX_0x6C4;

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
} XXX_0x6D9;
