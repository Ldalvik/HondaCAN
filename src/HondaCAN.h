#pragma once
#include "driver/twai.h"
#include "Const.h"
#include "PIDS.h"
#include "CANOBD.h"
#include "Vin.h"
// Change these if you make a new parser
#include "Accord_2016_LX/RawCanParser.h"
#include "Accord_2016_LX/CanParser.h"

class HondaCAN
{
public:
    HondaCAN();
    bool macchina_a0_setup(); // Setup for Macchina A0, CAN-11@500kbaud, (2016 Honda Accord LX)
    void run();

    float batteryVoltage(); // Based on Macchina A0 power sensor wiring, reads +12v pin on OBD port

    RawCanData_t RawCanData; 
    CanParser ParsedCanData;   // Scaled and adjusted into readable units
    Telemetry telemetry;  // Struct with all data for streamlined ESP-NOW usage (see examples/espnow)
    CANOBD OBD;
    Vin VIN;

private:
    int LED_EN = 13; // Macchina A0 specific, LED pin
    int RX_PIN = 4;
    int TX_PIN = 5;
    int CAN_RS = 21;
    int POWER_SENSE = 35;

    uint32_t tripDistanceOverflows = 0;
    uint16_t lastTripDistance = 0;
    void checkForReset();       // When vehicle is powered off, we need to reset distance traveled. Not sure if reliable
    void updateTripDistance();  // ECU only stores 1 byte of trip data, so we must track the resets manually
};