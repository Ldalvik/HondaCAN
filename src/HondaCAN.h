#pragma once
#include <Arduino.h>
#include "driver/twai.h"
#include "Const.h"
#include "PIDS.h"

class HondaCAN
{
public:
    HondaCAN();
    bool begin();
    void run();
    float obd2Request(uint8_t pid);
    
    int LED_EN = 13;
    int RX_PIN = 4;
    int TX_PIN = 5;
    int CAN_RS = 21;
    int SENSE_V_ANA = 35;
    int CAN_BIT = 11; // 11 or 21

    POWERTRAIN_DATA Powertrain;
    GEARBOX Gearbox;
    CAR_SPEED CarSpeed;
    VSA_STATUS VsaStatus;
    DRIVE_MODES DriveModes;
    ENGINE_DATA_3 EngineDataThree;
    ENGINE_DATA_2 EngineDataTwo;

private:
    void parsePowertrainData(uint8_t data[8]);
    void parseGearbox(uint8_t data[8]);
    void parseCarSpeed(uint8_t data[8]);
    void parseVsaStatus(uint8_t data[8]);
    void parseDriveModes(uint8_t data[8]);
    void parseEngineData3(uint8_t data[8]);
    void parseEngineData2(uint8_t data[8]);

    void obd2_send(uint8_t mode, uint8_t pid);
    bool obd2_receive();
    twai_message_t obdResponse;
    // uint64_t byteConvert(uint8_t data[8]);
};