#pragma once
#include <Arduino.h>
#include "driver/twai.h"
#include "Const.h"

class HondaCAN
{
public:
    HondaCAN();
    bool begin();
    void run();

    int LED_EN = 13;
    int RX_PIN = 4;
    int TX_PIN = 5;
    int CAN_RS = 21;
    int SENSE_V_ANA = 35;

    POWERTRAIN_DATA PowertrainData;
    GEARBOX_15T Gearbox15T;
    CAR_SPEED CarSpeed;

private:
    void parsePowertrainData(uint8_t data[8]);
    void parseGearbox15T(uint8_t data[8]);
    void parseCarSpeed(uint8_t data[8]);
    // uint64_t byteConvert(uint8_t data[8]);
};