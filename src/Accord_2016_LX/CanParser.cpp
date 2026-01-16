#include "CanParser.h"

constexpr float TORQUE_SCALE_RELATIVE = 0.194f; // rough scaling for K24W1 instead of torque map (see TORQUE_ESTIMATE in Const.h)
constexpr float NM_TO_LBFT = 0.73756f;
constexpr float KM_TO_MI = 0.621371f;
constexpr float M_TO_MI = 0.000621371f;
constexpr float ACCEL_SCALE = 0.0015f;
constexpr float UL_TO_GAL = 0.0000264172f;

CanParser::CanParser(const RawCanData_t &raw, const uint32_t &tripOverflows, const uint16_t &lastTrip)
    : RawCanData(raw), tripDistanceOverflows(tripOverflows), lastTripDistance(lastTrip)
{
}

float CanParser::getTorque_lbft() const
{
    return (RawCanData.EngineData1.TORQUE_ESTIMATE * TORQUE_SCALE_RELATIVE) * NM_TO_LBFT;
}

float CanParser::getHorsepower() const
{
    return (getTorque_lbft() * getEngineRPM()) / 5252.0f;
}

float CanParser::getRequestedTorque_lbft() const
{
    return (RawCanData.EngineData1.TORQUE_REQUEST * TORQUE_SCALE_RELATIVE) * NM_TO_LBFT;
}

float CanParser::getRequestedHorsepower() const
{
    return (getRequestedTorque_lbft() * getEngineRPM()) / 5252.0f;
}

int16_t CanParser::getTorqueEstimate_nM() const
{
    return RawCanData.EngineData1.TORQUE_ESTIMATE;
}

int16_t CanParser::getTorqueRequested_nM() const
{
    return RawCanData.EngineData1.TORQUE_REQUEST;
}

int8_t CanParser::getSteerAngleCoarse() const
{
    return RawCanData.SteeringSensors.STEER_ANGLE_COARSE;
}

int16_t CanParser::getSteerAngle() const
{
    return RawCanData.SteeringSensors.STEER_ANGLE * -0.1f;
}

int16_t CanParser::getSteerRate() const
{
    return RawCanData.SteeringSensors.STEER_RATE * -1.0f;
}

float CanParser::getTransmissionSpeed() const
{
    return RawCanData.EngineData2.XMISSION_SPEED * 0.01f;
}

float CanParser::getTransmissionSpeedSpeedometer() const
{
    return RawCanData.EngineData2.XMISSION_SPEED_SPEEDOMETER * 0.01f;
}

uint32_t CanParser::getTripDistance_m() const
{
    return (tripDistanceOverflows * 2560U) + lastTripDistance;
}

float CanParser::getTripDistance_Mi() const
{
    return getTripDistance_m() * M_TO_MI;
}

uint8_t CanParser::getGasPedal() const
{
    return RawCanData.PowertrainData.PEDAL_GAS;
    // return EngineData1.CAR_GAS;
}

uint16_t CanParser::getEngineRPM() const
{
    return RawCanData.PowertrainData.ENGINE_RPM;
}

bool CanParser::isGasPedalPressed() const
{
    return RawCanData.PowertrainData.GAS_PRESSED;
}

bool CanParser::getBrakeSwitchActive() const
{
    return RawCanData.PowertrainData.BRAKE_SWITCH;
}

bool CanParser::isBrakePedalPressed() const
{
    return RawCanData.PowertrainData.BRAKE_PRESSED;
}

GearShifter CanParser::getCurrentGear() const
{
    return RawCanData.Gearbox.CURRENT_GEAR;
}

uint8_t CanParser::getBrakePedal() const
{
    return RawCanData.VsaStatus.USER_BRAKE;
}

bool CanParser::isVsaDisabled() const
{
    return RawCanData.VsaStatus.ESP_DISABLED;
}

ScmButtons CanParser::getScmButtonPressed() const
{
    return RawCanData.ScmFeedback1.SCM_BUTTONS;
}

HeadlightState CanParser::getHeadlightState() const
{
    return RawCanData.ScmFeedback1.HEADLIGHT_STATE;
}

bool CanParser::isEbrakeOn() const
{
    return RawCanData.ScmFeedback1.E_BRAKE;
}

bool CanParser::isAcCompressorOn() const
{
    return RawCanData.ScmFeedback1.AC_COMPRESSOR_ON;
}

uint8_t CanParser::getFuelLevel() const
{
    return RawCanData.ScmFeedback1.FUEL_LEVEL;
}

bool CanParser::isCruiseMainOn() const
{
    return RawCanData.ScmFeedback1.CRUISE_MAIN_ON;
}

bool CanParser::isWheelsMoving() const
{
    return RawCanData.Standstill.WHEELS_MOVING;
}

float CanParser::getLatitudeAccel() const
{
    return RawCanData.VehicleDynamics.LAT_ACCEL * ACCEL_SCALE;
}

float CanParser::getLongitudeAccel() const
{
    return RawCanData.VehicleDynamics.LONG_ACCEL * ACCEL_SCALE;
}

bool CanParser::isEconModeOn() const
{
    return RawCanData.DriveModes.ECON_ON == Econ::ECON_ON;
}

float CanParser::getWheelspeedFL() const
{
    return RawCanData.RoughWheelSpeed.WHEEL_SPEED_FL;
}

float CanParser::getWheelspeedFR() const
{
    return RawCanData.RoughWheelSpeed.WHEEL_SPEED_FR;
}

float CanParser::getWheelspeedRL() const
{
    return RawCanData.RoughWheelSpeed.WHEEL_SPEED_RL;
}

float CanParser::getWheelspeedRR() const
{
    return RawCanData.RoughWheelSpeed.WHEEL_SPEED_RR;
}

bool CanParser::isLeftBlinkerOn() const
{
    return RawCanData.ScmFeedback2.BLINKER_LEFT;
}

bool CanParser::isRightBlinkerOn() const
{
    return RawCanData.ScmFeedback2.BLINKER_RIGHT;
}

WiperStatus CanParser::getWiperMode() const
{
    return RawCanData.ScmFeedback2.WIPER_STATUS;
}

uint32_t CanParser::getOdometer_Mi() const
{
    return RawCanData.ScmFeedback2.ODOMETER * KM_TO_MI;
}

bool CanParser::getTpmsButtonPressed() const
{
    return RawCanData.TpmsFeedback.TPMS_BUTTON;
}

bool CanParser::isPassengerSeatbeltUnlatched() const
{
    return RawCanData.SeatbeltStatus.PASS_UNLATCHED;
}

bool CanParser::isDriverSeatbeltUnlatched() const
{
    return RawCanData.SeatbeltStatus.DRIVER_UNLATCHED;
}

bool CanParser::isPassengerAirbagOff() const
{
    return RawCanData.SeatbeltStatus.PASS_AIRBAG_OFF;
}

float CanParser::getSpeed_Mph() const
{
    return (RawCanData.CarSpeed.ROUGH_CAR_SPEED * 0.01f) * KM_TO_MI;
}

float CanParser::getEngineCoolantTemp_C() const
{
    return RawCanData.EngineData3.ENGINE_TEMP - 40;
}

float CanParser::getIntakeTemp_C() const
{
    return RawCanData.EngineData3.INTAKE_TEMP - 40;
}

uint16_t CanParser::getTripFuelConsumed_uL() const
{
    return RawCanData.EngineData3.TRIP_FUEL_CONSUMED; // microliters
}

float CanParser::getTripFuelConsumed_gal() const
{
    return RawCanData.EngineData3.TRIP_FUEL_CONSUMED * UL_TO_GAL;
}

bool CanParser::isHoodLatchOpen() const
{
    return RawCanData.HoodLatch.HOOD_OPEN;
}

uint8_t CanParser::getWiperSpeed() const
{
    return RawCanData.ScmFeedback3.WIPER_SPEED;
}

bool CanParser::isDoorOpenFL() const
{
    return RawCanData.ScmFeedback4.DOOR_OPEN_FL;
}

bool CanParser::isDoorOpenFR() const
{
    return RawCanData.ScmFeedback4.DOOR_OPEN_FR;
}

bool CanParser::isDoorOpenRL() const
{
    return RawCanData.ScmFeedback4.DOOR_OPEN_RL;
}

bool CanParser::isDoorOpenRR() const
{
    return RawCanData.ScmFeedback4.DOOR_OPEN_RR;
}

bool CanParser::isTrunkOpen() const
{
    return RawCanData.ScmFeedback4.TRUNK_OPEN;
}
