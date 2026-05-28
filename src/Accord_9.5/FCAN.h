#pragma once
#include "VehicleProfile.h"
#include "BitHelpers.h"
#include "Enums.h"

// static inline void parse_0x039(const uint8_t *d, VehicleData &out){}
// static inline void parse_0x091(const uint8_t *d, VehicleData &out){}

static inline void parse_0x13C(const uint8_t *d, VehicleData &out)
{
    out.PCM.TorqueEstimate = I16(&d[0]); // Normalized
    out.PCM.TorqueRequest = I16(&d[2]);  // Normalized
    out.PCM.GasPedalPosition = d[4];
    bool IdleRequest = bit_U8(d[5], 1); // true when off gas pedal. Engine braking or idle/coast request?
}

static inline void parse_0x156(const uint8_t *d, VehicleData &out)
{
    out.PCM.SteerAngle = I16(&d[0]) * -0.1f;
    out.PCM.SteerRate = I16(&d[2]) * -1.0f;
}

static inline void parse_0x158(const uint8_t *d, VehicleData &out)
{
    uint16_t TransmissionSpeed = U16(&d[0]) * 0.01f; // speed from transmission
    uint16_t SpeedometerSpeed = U16(&d[2]) * 0.01f;  // speed on speedometer
    out.PCM.TripDistance = d[6] * 10;                // meters, overflows at 255
}

static inline void parse_0x17C(const uint8_t *d, VehicleData &out)
{
    uint8_t GasPedalPosition = d[0];
    uint8_t GasPedalPosition2 = d[1]; // matches PEDAL_GAS but way smaller number
    out.PCM.EngineRPM = U16(&d[2]);
    bool BrakeSwitchOff = bit_U8(d[4], 0); // (or d[4] & 0x01) true when shift lock is disabled
    bool GasPedalPressed = bit_U8(d[4], 7);
    bool BrakePedalPressed = bit_U8(d[6], 5);
}

static inline void parse_0x18E(const uint8_t *d, VehicleData &out)
{
    out.BCM.test_Yaw = d[1]; // One of the kinematic sensors, need to test
}

static inline void parse_0x191(const uint8_t *d, VehicleData &out)
{
    out.PCM.GearShifter = d[0] & 0x3F; // Gears go to a max value of 32, which is a PRNDSL setup
    out.PCM.ManualGear = d[1] >> 4;    // Manual gear mode, 1-7
    uint8_t TransmissionState = d[2];  // 128 in D, 0 in S, 8 when manual mode activates. Undetermined
    
    constexpr float max = 2.645f, min = 0.405f; // CVT gear ratio
    float CvtRatioEstimate = max - (d[3] / 255.0f) * (max - min);
    float CvtRatioRequest  = max - (d[4] / 255.0f) * (max - min);
    uint8_t InternalShiftStatus = d[5]; // Some kind of internal shift status, similar to GearShifter
    uint8_t DriveState = d[6];          // Tracks transmission mode collectively, GearShifter + ManualGear
}

// static inline void parse_0x19B(const uint8_t *d, VehicleData &out){}

static inline void parse_0x1A4(const uint8_t *d, VehicleData &out)
{
    out.PCM.BrakePedalPosition = U16(&d[0]);
    uint8_t ComputerBraking = bit_U8(d[2], 7); // ABS?
    uint8_t EspDisabled = bit_U8(d[3], 4);     // VSA disabled
}

static inline void parse_0x1A6(const uint8_t *d, VehicleData &out)
{
    out.SCM.ScmButtons = (d[0] & 0xF0) >> 4;
    out.SCM.HeadlightState = d[0] & 0x0F;
    out.BCM.ParkingBrake = bit_U8(d[0], 5);
    bool AcCompressorOn = bit_U8(d[2], 4); // ? test this
    out.BCM.FuelLevel = d[3];
    out.SCM.CruiseMainOn = bit_U8(d[5], 7);
}

// static inline void parse_0x1AA(const uint8_t *d, VehicleData &out){}

static inline void parse_0x1B0(const uint8_t *d, VehicleData &out)
{
    bool WheelsMoving = bit_U8(d[1], 4);
}

// static inline void parse_0x1D0(const uint8_t *d, VehicleData &out){}

static inline void parse_0x1DC(const uint8_t *d, VehicleData &out)
{
    uint16_t EngineRPM = U16(&d[1]);
}

static inline void parse_0x1EA(const uint8_t *d, VehicleData &out)
{
    out.BCM.test_LatAccel = I16(&d[0]) * 0.0015f;
    out.BCM.test_LongAccel = I16(&d[2]) * 0.0015f;
}

// static inline void parse_0x21A(const uint8_t *d, VehicleData &out){}
// static inline void parse_0x21E(const uint8_t *d, VehicleData &out){}

static inline void parse_0x221(const uint8_t *d, VehicleData &out)
{
    uint8_t EconMode = d[2] & 0x03;
    bool EconOn = EconMode == 0x03; // 0 for off, 3 for on
}

static inline void parse_0x255(const uint8_t *d, VehicleData &out)
{
    out.VSA.WheelSpeedFL = d[0];
    out.VSA.WheelSpeedFR = d[1];
    out.VSA.WheelSpeedRL = d[2];
    out.VSA.WheelSpeedRR = d[3];
}

static inline void parse_0x294(const uint8_t *d, VehicleData &out)
{
    out.SCM.WiperStatus = (d[0] >> 3) & 0x03;
    out.SCM.BlinkerLeft = bit_U8(d[0], 5);
    out.SCM.BlinkerRight = bit_U8(d[0], 6);
    out.SCM.Odometer = U24(&d[3]) * KM_TO_MI;
}

static inline void parse_0x295(const uint8_t *d, VehicleData &out)
{
    bool TpmsButtonPressed = bit_U8(d[1], 4);
}

static inline void parse_0x305(const uint8_t *d, VehicleData &out)
{
    bool DriverLamp = bit_U8(d[0], 7);
    bool PassengerUnlatched = bit_U8(d[1], 2);
    bool DriverUnlatched = bit_U8(d[1], 4);
    bool PassengerAirbagOff = bit_U8(d[1], 6);
}

static inline void parse_0x309(const uint8_t *d, VehicleData &out)
{
    out.PCM.Speed = (U16(&d[4]) * 0.01f) * KM_TO_MI;
}

static inline void parse_0x320(const uint8_t *d, VehicleData &out)
{
    out.PCM.VTEC = bit_U8(d[1], 1); // VTEC engagement
}

static inline void parse_0x324(const uint8_t *d, VehicleData &out)
{
    out.PCM.EngineTemp = d[0] - 40;
    out.PCM.IntakeTemp = d[1] - 40;
    out.PCM.TripFuelConsumed = U16(&d[2]) * RAW_TO_GAL;
}

// static inline void parse_0x328(const uint8_t *d, VehicleData &out){}
// static inline void parse_0x367(const uint8_t *d, VehicleData &out){}

static inline void parse_0x372(const uint8_t *d, VehicleData &out)
{
    out.BCM.HoodOpen = bit_U8(d[0], 5);
}

// static inline void parse_0x374(const uint8_t *d, VehicleData &out){}
// static inline void parse_0x37B(const uint8_t *d, VehicleData &out){}
// static inline void parse_0x396(const uint8_t *d, VehicleData &out){}

static inline void parse_0x3A1(const uint8_t *d, VehicleData &out)
{
    out.SCM.WiperSpeed = d[0]; // 0 to 60
}

// static inline void parse_0x3D7(const uint8_t *d, VehicleData &out){}
// static inline void parse_0x3D9(const uint8_t *d, VehicleData &out){}
// static inline void parse_0x400(const uint8_t *d, VehicleData &out){}
// static inline void parse_0x403(const uint8_t *d, VehicleData &out){}

static inline void parse_0x405(const uint8_t *d, VehicleData &out)
{
    out.BCM.DoorOpenFL = bit_U8(d[4], 5);
    out.BCM.DoorOpenFR = bit_U8(d[4], 6);
    out.BCM.DoorOpenRL = bit_U8(d[4], 7);
    out.BCM.DoorOpenRR = bit_U8(d[5], 0); // (or d[5] & 0x01)
    out.BCM.TrunkOpen = bit_U8(d[5], 1);
}
// static inline void parse_0x406(const uint8_t *d, VehicleData &out){}
// static inline void parse_0x40C(const uint8_t *d, VehicleData &out){}
// static inline void parse_0x40F(const uint8_t *d, VehicleData &out){}
// static inline void parse_0x421(const uint8_t *d, VehicleData &out){}
// static inline void parse_0x428(const uint8_t *d, VehicleData &out){}
