#pragma once
#include <stdint.h>

struct VehicleData
{
    struct
    {
        uint8_t Speed = 0.0f; // MPH
        uint16_t EngineRPM = 0;
        float TripDistance = 0.0f;     // Miles
        float TripFuelConsumed = 0.0f; // Gallons
        uint8_t EngineTemp = 0;
        uint8_t IntakeTemp = 0;
        bool VTEC = false;
        int16_t TorqueEstimate = 0;
        int16_t TorqueRequest = 0;
        uint8_t GasPedalPosition = 0;
        uint8_t BrakePedalPosition = 0;
        int16_t SteerAngle = 0;
        int16_t SteerRate = 0;
        uint8_t GearShifter = 0;
        uint8_t ManualGear = 0;
        //uint8_t TransmissionTemp = 0;

    } PCM;

    struct
    {
        float WheelSpeedFL = 0.0f; // MPH
        float WheelSpeedFR = 0.0f; // MPH
        float WheelSpeedRL = 0.0f; // MPH
        float WheelSpeedRR = 0.0f; // MPH
    } VSA;

    struct
    {
        uint8_t ScmButtons = 0;
        uint8_t HeadlightState = 0;
        uint32_t Odometer = 0;
        bool CruiseMainOn = 0;
        bool BlinkerLeft = false;
        bool BlinkerRight = false;
        uint8_t WiperStatus = 0;
        uint8_t WiperSpeed = 0;
        bool EconOn = false;
    } SCM;

    struct
    {
        uint8_t FuelLevel = 0;
        uint8_t ParkingBrake = 0;
        bool HoodOpen = false;
        bool DoorOpenFL = false;
        bool DoorOpenFR = false;
        bool DoorOpenRL = false;
        bool DoorOpenRR = false;
        bool TrunkOpen = false;
        int16_t test_LatAccel = 0;
        int16_t test_LongAccel = 0;
        uint8_t test_Yaw = 0;
    } BCM;
    
     struct
    {
        uint8_t TripDistanceLast = 0;
        uint8_t TripDistanceOverflow = 0;
    } MISC;
};
