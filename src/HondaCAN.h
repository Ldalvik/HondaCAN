#pragma once
#include "driver/twai.h"
#include "PIDS.h"
#include "CANOBD.h"
#include "Vin.h"
#include "VehicleData.h"
#include "Accord_9.5/Profile.h"

class HondaCAN
{
public:
    HondaCAN();
    bool macchina_a0_setup(); // Setup for Macchina A0, CAN-11@500kbaud
    void run();
    static float batteryVoltage(); // Macchina A0 power sensor wiring, reads +12v pin on OBD port
    void setVehicleProfile(const VehicleProfile &p);
    const char *vehicleName() const;

    VehicleData CanData; // Scaled and adjusted into readable units
    CANOBD OBD;
    Vin VIN;

private:
    const VehicleProfile* profile_ = nullptr;
    static constexpr int LED_EN = 13; // Macchina A0 specific, LED pin
    static constexpr int RX_PIN = 4;
    static constexpr int TX_PIN = 5;
    static constexpr int CAN_RS = 21;
    static constexpr int POWER_SENSE = 35;
};