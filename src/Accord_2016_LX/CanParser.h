#pragma once
#include "Const.h"

struct RawCanData_t;

class CanParser
{
public:
    explicit CanParser(const RawCanData_t& raw,
                            const uint32_t& tripOverflows,
                            const uint16_t& lastTrip);
                            
float getTorque_lbft() const;
float getHorsepower() const;
float getRequestedTorque_lbft() const;
float getRequestedHorsepower() const;
int16_t getTorqueEstimate_nM() const;
int16_t getTorqueRequested_nM() const;
int8_t getSteerAngleCoarse() const;
int16_t getSteerAngle() const;
int16_t getSteerRate() const;
float getTransmissionSpeed() const;
float getTransmissionSpeedSpeedometer() const;
uint32_t getTripDistance_m() const;
float getTripDistance_Mi() const;
uint8_t getGasPedal() const;
uint16_t getEngineRPM() const;
bool isGasPedalPressed() const;
bool getBrakeSwitchActive() const;
bool isBrakePedalPressed() const;
GearShifter getCurrentGear() const;
uint8_t getBrakePedal() const;
bool isVsaDisabled() const;
ScmButtons getScmButtonPressed() const;
HeadlightState getHeadlightState() const;
bool isEbrakeOn() const;
bool isAcCompressorOn() const;
uint8_t getFuelLevel() const;
bool isCruiseMainOn() const;
bool isWheelsMoving() const;
float getLatitudeAccel() const;
float getLongitudeAccel() const;
bool isEconModeOn() const;
float getWheelspeedFL() const;
float getWheelspeedFR() const;
float getWheelspeedRL() const;
float getWheelspeedRR() const;
bool isLeftBlinkerOn() const;
bool isRightBlinkerOn() const;
WiperStatus getWiperMode() const;
uint32_t getOdometer_Mi() const;
bool getTpmsButtonPressed() const;
bool isPassengerSeatbeltUnlatched() const;
bool isDriverSeatbeltUnlatched() const;
bool isPassengerAirbagOff() const;
float getSpeed_Mph() const;
float getEngineCoolantTemp_C() const;
float getIntakeTemp_C() const;
uint16_t getTripFuelConsumed_uL() const;
float getTripFuelConsumed_gal() const;
bool isHoodLatchOpen() const;
uint8_t getWiperSpeed() const;
bool isDoorOpenFL() const;
bool isDoorOpenFR() const;
bool isDoorOpenRL() const;
bool isDoorOpenRR() const;
bool isTrunkOpen() const;

private:
    const RawCanData_t& RawCanData;
    const uint32_t& tripDistanceOverflows;
    const uint16_t& lastTripDistance;
};
