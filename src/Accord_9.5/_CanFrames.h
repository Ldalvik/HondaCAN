#pragma once
#include <cstdint>

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t UnknownBits_1 : 7;
    uint8_t AccMode : 1; // 1 when in ACC, off in all other modes? (need to test)
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} PCM_0x039; // Heartbeat (DLC: 3, 25Hz)

typedef struct __attribute__((packed))
{
    uint16_t bytes0_1; 
    uint16_t byte2_3;
    int8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} PCM_0x091; // (DLC: 8, 100Hz)

/*  //----- INFO -----//
    TorqueRequest and TorqueEstimate are in "engineering units", a normalized/scaled
    measurement that has no known direct conversion to real engine torque. At the peak torque range
    for my vehicle, this value reached 1100. It does seem to be accurate the power curve of the engine.
    Using a reference table of the cars real torque output will give the most accurate results.
    The ECU uses these values among others to reach the requested engine load.
*/
typedef struct __attribute__((packed))
{
    int16_t TorqueEstimate; // byte [0..1]
    int16_t TorqueRequest; // byte [2..3]
    uint8_t GasPedalPosition; // byte 4
    uint8_t UnknownBit_1 : 1; // byte 5, bit 0
    uint8_t IdleRequest : 1; // byte 5, bit 1
    uint8_t UnknownBits_2 : 6; // byte 5, bits [2..7]
    uint8_t byte6; // always 6
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} PCM_0x13C; // Data (DLC: 8, 100Hz)

typedef struct __attribute__((packed))
{
    int16_t SteerAngle; // Fine steering angle sensor position. -512 is far left, 512 is far right
    int16_t SteerRate;  // How fast the wheel is being turned (degrees/s)
    uint8_t byte4;      // always 7
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} EPS_0x156; // Steering Sensors (DLC: 6, 100Hz)

typedef struct __attribute__((packed))
{
    uint16_t TransmissionSpeed; // bytes [0..1]
    uint8_t byte2; // always 0
    uint8_t byte3; // always 0
    uint16_t SpeedometerSpeed; // byte [4..5]
    uint8_t TripDistance; // byte 6
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} PCM_0x158; // Data (DLC: 8, 100Hz)

typedef struct __attribute__((packed))
{
    uint8_t GasPedalPosition;       // byte 0
    uint8_t GasPedalPosition2;      // byte 1
    uint16_t EngineRPM;             // bytes [2..3]
    uint8_t BrakeSwitchOff : 1;     // byte 4, bit 0
    uint8_t UnknownBits_1 : 6;     // byte 4, bits [1..6]
    uint8_t GasPedalPressed : 1;   // byte 4, bit 7
    uint8_t byte5;                  // byte 5
    uint8_t UnknownBits_2 : 5;     // byte 6, bits [0..4]
    uint8_t BrakePedalPressed : 1; // byte 6, bit 5
    uint8_t UnknownBits_3 : 2;     // byte 6, bits [6..7]
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} PCM_0x17C; // Data (DLC: 8, 100Hz)

typedef struct __attribute__((packed))
{
    uint8_t byte0;
    uint8_t byte1; // lateral yaw rate sensor?
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} VSA_0x18E; // Kinematics (DLC: 3, 100Hz)

typedef struct __attribute__((packed)) {
    uint8_t GearShifter; //  byte 0
    uint8_t ManualGear; // byte 1
    uint8_t TransmissionState; // byte 2
    uint8_t CvtRatioEstimate; // byte 3
    uint8_t CvtRatioRequest; // byte 4
    uint8_t InternalShiftStatus; // byte 5;
    uint8_t DriveState; // byte 6
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} PCM_0x191; // Gearbox (DLC: 8, 100Hz)

typedef struct __attribute__((packed)) {
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} XXX_0x19B; // (DLC: 5, 100Hz)

typedef struct __attribute__((packed)) {
    uint16_t BrakePedalPosition; // bytes [0..1]
    uint8_t UnknownBits_1 : 7;   // byte 2, bits [0..6]
    uint8_t ComputerBraking : 1; // byte 2, bit 7
    uint8_t UnknownBits_2 : 4; // byte 3, bits [0..3]
    uint8_t EspDisabled : 1; // byte 3, bit 4
    uint8_t UnknownBits_3 : 3; // byte 3, bits [5..7]
    uint8_t byte4; // byte 4
    uint8_t byte5; // byte 5
    uint8_t byte6; // byte 6
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} VSA_0x1A4; // VSA status (DLC: 8, 50Hz)

typedef struct __attribute__((packed)) {
    uint8_t HeadlightState : 4; // byte 0, bits [0..3]
    uint8_t ScmButton      : 4; // byte 0, bits [4..7]
    uint8_t byte1;
    uint8_t UnknownBits_1  : 4; // byte 2, bits [0..3]
    uint8_t AcCompressorOn : 4; // byte 2, bits [4..7]
    uint8_t FuelLevel; // byte 3
    uint8_t byte4;
    uint8_t UnknownBits_2   : 7; // byte 5, bits [0..6]
    uint8_t CRUISE_MAIN_ON  : 1; // byte 5, bit 7
    uint8_t byte6;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} SCM_0x1A6; // Feedback (DLC: 8, 50Hz)

typedef struct __attribute__((packed)) {} XXX_0x1AA; // (DLC: -, 50Hz)
typedef struct __attribute__((packed)) {
    uint8_t byte0;
    uint8_t UnknownBits_1 : 4; // byte 2, bits [0..3]
    uint8_t WheelsMoving : 1; // byte 2, bit 4
    uint8_t UnknownBits_2 : 2; // byte 2, bits [5..7]
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} VSA_0x1B0; // Standstill (DLC: 7, 50Hz)

typedef struct __attribute__((packed)) {
    uint16_t bytes0_1;
    uint16_t bytes2_3;
    uint16_t bytes4_5;
    uint16_t bytes6;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} VSA_0x1D0; // Wheel speed? (DLC: 8, 50Hz)

typedef struct __attribute__((packed)) {
    uint8_t byte0; // always 2
    uint16_t EngineRPM; // bytes [1..2]
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} PCM_0x1DC; // RPM (DLC: 4, 50Hz)

typedef struct __attribute__((packed)) {
    int16_t LatAccel;  // bytes [0..1]
    int16_t LongAccel; // bytes [1..2]
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} VSA_0x1EA; // Vehicle dynamics (DLC: 8, 50Hz)

typedef struct __attribute__((packed)) {} XXX_0x21A; // (DLC: -, 25Hz)
typedef struct __attribute__((packed)) {} XXX_0x21E; // (DLC: -, 25Hz)
typedef struct __attribute__((packed)) {
    uint8_t byte0;
    uint8_t byte1;
    uint8_t EconOn : 3; // byte 2, bits [0..2]
    uint8_t UnknownBits_1 : 5; // byte 2, bits [3..7]
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} BCM_0x221; // Drive modes (DLC: 8, 25Hz)

typedef struct __attribute__((packed)) {
    uint8_t WheelSpeedFL; // byte 0
    uint8_t WheelSpeedFR; // byte 1
    uint8_t WheelSpeedRL; // byte 2
    uint8_t WheelSpeedRR; // byte 3
    uint8_t byte4; // always 85
    uint8_t byte5; // always 85
    uint8_t COUNTER; // long counter
    uint8_t CHECKSUM; 
} PCM_0x255; // Rough wheel speed (DLC: 8, 25Hz)

typedef struct __attribute__((packed)) {
    uint8_t UnknownBits_1 : 3; // byte 0, bits [0..2]
    uint8_t WiperStatus  : 2; // byte 0, bits [3..4]
    uint8_t BlinkerLeft  : 1; // byte 0, bit 5
    uint8_t BlinkerRight : 1; // byte 0, bit 6
    uint8_t UnknownBit_2 : 1; // byte 0, bit 7
    uint8_t byte1;
    uint8_t byte2;
    uint32_t Odometer : 24; // bytes [3..5]
    uint8_t byte6;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} SCM_0x294; // Feedback (DLC: 8, 25Hz)

typedef struct __attribute__((packed)) {
    uint8_t byte0;
    uint8_t UnknownBits_1 : 4; // byte 1, bits [0..3]
    uint8_t TpmsButtonPressed : 1; // byte 1, bit 4
    uint8_t UnknownBits_2 : 3; // byte 1, bits [5..7]
} BCM_0x295; // TPMS Feedback (DLC: 4, 25Hz)

typedef struct __attribute__((packed)) {
    uint8_t UnknownBits_1 : 7; // byte 0, bits [0..6]
    uint8_t DriverLamp    : 1; // byte 0, bit 7
    uint8_t UnknownBits_2 : 2; // byte 1, bits [0..1]
    uint8_t PassengerUnlatched : 1; // byte 1, bit 2
    uint8_t UnknownBit_4  : 1; // byte 1, bit 3
    uint8_t DriverUnlatched : 1; // byte 1, bit 4
    uint8_t UnknownBit_3  : 1;  // byte 1, bit 5
    uint8_t PassengerAirbagOff : 1; // byte 1, bit 6
    uint8_t UnknownBit_4  : 1; // byte 1, bit 7
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} BCM_0x305; // Seatbelt status (DLC: 7, 10Hz)

typedef struct __attribute__((packed)) {
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint16_t Speed; // bytes [4..5] 
    uint8_t byte6;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} PCM_0x309; // Car speed (DLC: 8, 10Hz)

typedef struct __attribute__((packed)) {
    uint8_t byte0;
    uint8_t UnknownBit_1 : 1; // byte 1, bit 0
    uint8_t VTEC : 1;        // byte 1, bit 1
    uint8_t UnknownBits_2 : 6; // byte 1, bits [2..7]
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} BCM_0x320; // (DLC: 8, 10Hz)

typedef struct __attribute__((packed)) {
    uint8_t EngineTemp; // byte 0
    uint8_t IntakeTemp; // byte 1
    uint16_t TripFuelConsumed; // bytes [2..3]
    uint8_t byte4;
    int8_t byte5; // changes a lot
    uint8_t byte6;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} PCM_0x324; // Data (DLC: 8, 10Hz)

typedef struct __attribute__((packed)) {} XXX_0x328; // (DLC: -, 10Hz)
typedef struct __attribute__((packed)) {} XXX_0x367; // (DLC: -, 10Hz)
typedef struct __attribute__((packed)) {
    uint8_t UnknownBits_1 : 5; // byte 0, bits [0..4]
    uint8_t HoodOpen : 1; // byte 0, bit 5
    uint8_t UnknownBits_2 : 2; // byte 0, bits [6..7]
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} BCM_0x372; // Hood latch (DLC: 2, 10Hz)

typedef struct __attribute__((packed)) {} XXX_0x374; // (DLC: -, 10Hz)
typedef struct __attribute__((packed)) {} XXX_0x37B; // (DLC: -, 10Hz)
typedef struct __attribute__((packed)) {} XXX_0x396; // (DLC: -, 10Hz)

typedef struct __attribute__((packed)) {
    uint8_t WiperSpeed; // byte 0
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} SCM_0x3A1; // Feedback (DLC: 8, 5Hz)

typedef struct __attribute__((packed)) {} XXX_0x3D7; // (DLC: -, 5Hz)
typedef struct __attribute__((packed)) {} XXX_0x3D9; // (DLC: -, 5Hz)
typedef struct __attribute__((packed)) {} XXX_0x400; // (DLC: -, 3Hz)
typedef struct __attribute__((packed)) {} XXX_0x403; // (DLC: -, 3Hz)

typedef struct __attribute__((packed)) {
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t UnknownBits_1 : 5; // byte 4, bits [0..4]
    uint8_t DoorOpenFL : 1; // byte 4, bit 5
    uint8_t DoorOpenFR : 1; // byte 4, bit 6
    uint8_t DoorOpenRL : 1; // byte 4, bit 7
    uint8_t DoorOpenRR : 1; // byte 5, bit 0
    uint8_t TrunkOpen : 1;  // byte 5, bit 1
    uint8_t UnknownBits_2 : 6; // byte 5, bits [2..7]
    uint8_t byte6;
    uint8_t COUNTER : 4;
    uint8_t CHECKSUM : 4;
} SCM_0x405; // Feedback (DLC: 8, 3Hz)

typedef struct __attribute__((packed)) {} XXX_0x406; // (DLC: -, 3Hz)
typedef struct __attribute__((packed)) {} XXX_0x40C; // (DLC: -, 3Hz)
typedef struct __attribute__((packed)) {} XXX_0x40F; // (DLC: -, 3Hz)
typedef struct __attribute__((packed)) {} XXX_0x421; // (DLC: -, 3Hz)
typedef struct __attribute__((packed)) {} XXX_0x428; // (DLC: -, 3Hz)

typedef struct __attribute__((packed)) {
    uint8_t Mux;       // byte 0
    uint8_t Data[7];   // bytes 1-7
} XXX_0x440; // (DLC: 8, 3Hz)

typedef struct __attribute__((packed)) {} XXX_0x441; // (DLC: -, 3Hz)
typedef struct __attribute__((packed)) {} XXX_0x454; // (DLC: -, 3Hz)
typedef struct __attribute__((packed)) {} XXX_0x465; // (DLC: -, 3Hz)
typedef struct __attribute__((packed)) {} XXX_0x510; // (DLC: -, 2Hz)
typedef struct __attribute__((packed)) {} XXX_0x555; // (DLC: -, 2Hz)
