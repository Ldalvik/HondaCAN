#pragma once
#include <Arduino.h>
#include "driver/twai.h"
#include "Const.h"

class HondaCAN {
public:
    HondaCAN(); 
    bool begin();             
    bool addFilter(uint32_t id);             
    void run();             

   int LED_EN = 13;
   int RX_PIN = 4;
   int TX_PIN = 5 ;
   int CAN_RS = 21;
   int SENSE_V_ANA = 35;

   POWERTRAIN_DATA PowertrainData;

private:
    void parsePowertrainData(uint64_t data);              
    uint64_t byteConvert(uint8_t data[8]);    
};