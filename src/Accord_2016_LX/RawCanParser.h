#pragma once
#include <stdint.h>
#include "Const.h"

#define bit(data, n) (((data) >> (n)) & 0x01)

inline float toFloat16(uint8_t h, uint8_t l)
{
  return ((uint16_t(h) << 8) | l);
}

inline uint16_t toU16(uint8_t h, uint8_t l)
{
  return (uint16_t(h) << 8) | l;
}

enum class CANFrame : uint16_t
{
    // 0x39, heartbeat? 25hz. 29 signals are undecoded
    ENGINE_DATA_1 = 0x13C,     // 100hz
    STEERING_SENSORS = 0x156,  // 100hz
    ENGINE_DATA_2 = 0x158,     // 100hz
    POWERTRAIN_DATA = 0x17C,   // 100hz
    KINEMATICS = 0x18E,        // 100hz
    GEARBOX = 0x191,           // 100hz
    VSA_STATUS = 0x1A4,        // 50hz
    SCM_FEEDBACK_1 = 0x1A6,    // 50hz
    STANDSTILL = 0x1B0,        // 50hz
    // WHEEL_SPEED = 0x1D0,       // 50hz
    VEHICLE_DYNAMICS = 0x1EA,  // 50hz
    DRIVE_MODES = 0x221,       // 25hz
    ROUGH_WHEEL_SPEED = 0x255, // 25hz
    SCM_FEEDBACK_2 = 0x294,    // 25hz
    TPMS_FEEDBACK = 0x295,     // 25hz
    SEATBELT_STATUS = 0x305,   // 10hz
    CAR_SPEED = 0x309,         // 10hz
    ENGINE_DATA_3 = 0x324,     // 10hz
    HOOD_LATCH = 0x372,        // 10hz
    SCM_FEEDBACK_3 = 0x3A1,    // 5hz
    SCM_FEEDBACK_4 = 0x405,    // 3hz
};

inline void parseEngineData1(const uint8_t *d, RawCanData_t &out)
{
    out.EngineData1.TORQUE_ESTIMATE = int16_t(toU16(d[0], d[1]));
    out.EngineData1.TORQUE_REQUEST = int16_t(toU16(d[2], d[3]));
    out.EngineData1.CAR_GAS = d[4]; // slightly smaller slope at certain points under 15% compared to what the powertrain ECU sees, raw pedal value?
}

inline void parseSteeringSensors(const uint8_t *d, RawCanData_t &out)
{
    out.SteeringSensors.STEER_ANGLE_COARSE = int8_t(d[0]);
    out.SteeringSensors.STEER_ANGLE = int16_t(toU16(d[0], d[1]));
    out.SteeringSensors.STEER_RATE = int16_t(toU16(d[2], d[3]));
}

inline void parseEngineData2(const uint8_t *d, RawCanData_t &out)
{
    out.EngineData2.XMISSION_SPEED = toFloat16(d[0], d[1]);
    out.EngineData2.XMISSION_SPEED_SPEEDOMETER = toFloat16(d[4], d[5]);
    out.EngineData2.CURRENT_TRIP_DISTANCE = uint16_t(d[6]) * 10; // meters
}

inline void parsePowertrainData(const uint8_t *d, RawCanData_t &out)
{
    out.PowertrainData.PEDAL_GAS = d[0];
    out.PowertrainData.ENGINE_RPM = toU16(d[2], d[3]);
    out.PowertrainData.GAS_PRESSED = bit(d[4], 7);
    out.PowertrainData.BRAKE_SWITCH = bit(d[4], 0);
    out.PowertrainData.BRAKE_PRESSED = bit(d[6], 5);
}

inline void parseKinematics(const uint8_t *d, RawCanData_t &out)
{
    // Not found?
}

inline void parseGearbox(const uint8_t *d, RawCanData_t &out)
{
    out.Gearbox.CURRENT_GEAR = (GearShifter)(d[0] & 0x3F);
}

inline void parseVsaStatus(const uint8_t *d, RawCanData_t &out)
{
    out.VsaStatus.USER_BRAKE = toU16(d[0], d[1]);
    // out.VsaStatus.COMPUTER_BRAKING = bit(data[2], 7);
    out.VsaStatus.ESP_DISABLED = bit(d[3], 4); // VSA
}

inline void parseScmFeedback1(const uint8_t *d, RawCanData_t &out)
{
    out.ScmFeedback1.SCM_BUTTONS = (ScmButtons)(d[0] & 0xF0);
    out.ScmFeedback1.HEADLIGHT_STATE = (HeadlightState)(d[0] & 0x0F);
    out.ScmFeedback1.E_BRAKE = bit(d[0], 5);
    out.ScmFeedback1.AC_COMPRESSOR_ON = d[2] & 0xF0; // bit(d[2], 4); ?
    out.ScmFeedback1.FUEL_LEVEL = d[3];
    out.ScmFeedback1.CRUISE_MAIN_ON = bit(d[5], 7);
}

inline void parseStandstill(const uint8_t *d, RawCanData_t &out)
{
    out.Standstill.WHEELS_MOVING = bit(d[1], 4);
}

inline void parseVehicleDynamics(const uint8_t *d, RawCanData_t &out)
{
    out.VehicleDynamics.LAT_ACCEL = int16_t(toU16(d[0], d[1]));
    out.VehicleDynamics.LONG_ACCEL = int16_t(toU16(d[2], d[3]));
}

inline void parseDriveModes(const uint8_t *d, RawCanData_t &out)
{
    out.DriveModes.ECON_ON = (Econ)(d[2] & 0x03);
}

inline void parseRoughWheelSpeed(const uint8_t *d, RawCanData_t &out)
{
    out.RoughWheelSpeed.WHEEL_SPEED_FL = d[0];
    out.RoughWheelSpeed.WHEEL_SPEED_FR = d[1];
    out.RoughWheelSpeed.WHEEL_SPEED_RL = d[2];
    out.RoughWheelSpeed.WHEEL_SPEED_RR = d[3];
}

inline void parseScmFeedback2(const uint8_t *d, RawCanData_t &out)
{
    out.ScmFeedback2.BLINKER_LEFT = bit(d[0], 5);
    out.ScmFeedback2.BLINKER_RIGHT = bit(d[0], 6);
    out.ScmFeedback2.WIPER_STATUS = (WiperStatus)(d[0] & 0x18);

    uint32_t km = ((uint32_t)d[3] << 16) |
                  ((uint32_t)d[4] << 8) |
                  ((uint32_t)d[5]);
    out.ScmFeedback2.ODOMETER = km;
}

inline void parseTpmsFeedback(const uint8_t *d, RawCanData_t &out)
{
    out.TpmsFeedback.TPMS_BUTTON = bit(d[1], 4); // or 3?
}

inline void parseSeatbeltStatus(const uint8_t *d, RawCanData_t &out)
{
    out.SeatbeltStatus.DRIVER_LAMP = bit(d[0], 7);
    out.SeatbeltStatus.PASS_UNLATCHED = bit(d[1], 2);
    out.SeatbeltStatus.DRIVER_UNLATCHED = bit(d[1], 4);
    out.SeatbeltStatus.PASS_AIRBAG_OFF = bit(d[1], 6);
}

inline void parseCarSpeed(const uint8_t *d, RawCanData_t &out)
{
    out.CarSpeed.ROUGH_CAR_SPEED = toFloat16(d[4], d[5]);
}

inline void parseEngineData3(const uint8_t *d, RawCanData_t &out)
{
    out.EngineData3.ENGINE_TEMP = d[0];
    out.EngineData3.INTAKE_TEMP = d[1];
    out.EngineData3.TRIP_FUEL_CONSUMED = toU16(d[2], d[3]);
}

inline void parseHoodLatch(const uint8_t *d, RawCanData_t &out)
{
    out.HoodLatch.HOOD_OPEN = bit(d[0], 5);
}

inline void parseScmFeedback3(const uint8_t *d, RawCanData_t &out)
{
    out.ScmFeedback3.WIPER_SPEED = d[0];
}

inline void parseScmFeedback4(const uint8_t *d, RawCanData_t &out)
{
    out.ScmFeedback4.DOOR_OPEN_FL = bit(d[4], 5);
    out.ScmFeedback4.DOOR_OPEN_FR = bit(d[4], 6);
    out.ScmFeedback4.DOOR_OPEN_RL = bit(d[4], 7);
    out.ScmFeedback4.DOOR_OPEN_RR = bit(d[5], 0);
    out.ScmFeedback4.TRUNK_OPEN = bit(d[5], 1);
}

inline void parseCanFrame(uint32_t id, const uint8_t *data, RawCanData_t &out)
{
    switch (static_cast<CANFrame>(id))
    {
    case CANFrame::ENGINE_DATA_1:
        parseEngineData1(data, out);
        break;
    case CANFrame::STEERING_SENSORS:
        parseSteeringSensors(data, out);
        break;
    case CANFrame::ENGINE_DATA_2:
        parseEngineData2(data, out);
        break;
    case CANFrame::POWERTRAIN_DATA:
        parsePowertrainData(data, out);
        break;
    case CANFrame::KINEMATICS:
        parseKinematics(data, out);
        break;
    case CANFrame::GEARBOX:
        parseGearbox(data, out);
        break;
    case CANFrame::VSA_STATUS:
        parseVsaStatus(data, out);
        break;
    case CANFrame::SCM_FEEDBACK_1:
        parseScmFeedback1(data, out);
        break;
    case CANFrame::STANDSTILL:
        parseStandstill(data, out);
        break;
    case CANFrame::VEHICLE_DYNAMICS:
        parseVehicleDynamics(data, out);
        break;
    case CANFrame::DRIVE_MODES:
        parseDriveModes(data, out);
        break;
    case CANFrame::ROUGH_WHEEL_SPEED:
        parseRoughWheelSpeed(data, out);
        break;
    case CANFrame::SCM_FEEDBACK_2:
        parseScmFeedback2(data, out);
        break;
    case CANFrame::TPMS_FEEDBACK:
        parseTpmsFeedback(data, out);
        break;
    case CANFrame::SEATBELT_STATUS:
        parseSeatbeltStatus(data, out);
        break;
    case CANFrame::CAR_SPEED:
        parseCarSpeed(data, out);
        break;
    case CANFrame::ENGINE_DATA_3:
        parseEngineData3(data, out);
        break;
    case CANFrame::HOOD_LATCH:
        parseHoodLatch(data, out);
        break;
    case CANFrame::SCM_FEEDBACK_3:
        parseScmFeedback3(data, out);
        break;
    case CANFrame::SCM_FEEDBACK_4:
        parseScmFeedback4(data, out);
        break;
    default:
        break;
    }
}
