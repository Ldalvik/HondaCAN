#include "HondaCAN.h"

HondaCAN::HondaCAN()
{
}

bool HondaCAN::begin(/*uint32_t filter = 0xFFFFFFFF*/)
{
  pinMode(CAN_RS, OUTPUT);
  digitalWrite(CAN_RS, LOW); // LOW = high speed mode, HIGH = low power mode (listen only)

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL); // TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK or TWAI_MODE_LISTEN_ONLY
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); //{.acceptance_code = filter, .acceptance_mask = 0x7FF, .single_filter = true};

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;
  return true;
}

void HondaCAN::run()
{
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK)
  {
    switch (message.identifier)
    {
      case POWERTRAIN_DATA_ID:
        this->parsePowertrainData(message.data);
      break;
      case GEARBOX_15T_ID:
        this->parseGearbox15T(message.data);
      break;
      case CAR_SPEED_ID:
        this->parseCarSpeed(message.data);
    break;
    }
  }
}

void HondaCAN::parsePowertrainData(uint8_t data[8])
{    
    PowertrainData.PEDAL_GAS     = data[0];                    
    PowertrainData.ENGINE_RPM    = (uint16_t)(data[3] << 8 | data[2]);     
    PowertrainData.GAS_PRESSED   = (data[4] >> 7) & 0x01;                  
    PowertrainData.ACC_STATUS    = (data[4] >> 6) & 0x01;                   
    PowertrainData.BRAKE_SWITCH  = (data[4] >> 0) & 0x01;             
    PowertrainData.BRAKE_PRESSED = (data[6] >> 5) & 0x01;                                   
}

void HondaCAN::parseGearbox15T(uint8_t data[8])
{
   Gearbox_15T.GEAR_SHIFTER = data[0] & 0x3F;  
   Gearbox_15T.GEAR2        = data[3];                       
   Gearbox_15T.GEAR         = data[4];                        
}

void HondaCAN::parseCarSpeed(uint8_t data[8])
{
  CarSpeedData.CAR_SPEED         = (data[0] | (data[1] << 8)) * 0.01f;  
  CarSpeedData.ROUGH_CAR_SPEED   = data[2];                       
  CarSpeedData.ROUGH_CAR_SPEED_2 = data[3]; 
  CarSpeedData.ROUGH_CAR_SPEED_3 = (data[4] | (data[5] << 8)) * 0.01f;                      
  CarSpeedData.LOCK_STATUS       = (data[6] >> 6) & 0x03;             
  CarSpeedData.IMPERIAL_UNIT     = (data[7] >> 7) & 0x01;           
}

// void deviceVoltage(void) {
//   return analogRead(SENSE_V_ANA);
// }

// Find where B-CAN and LIN bus wires are, maybe behind radio harness?
// Then we can read climate control, lock, steering wheel, etc. maybe even write data back. 
// Is OBD port data just filtered F-CAN data, or can we tap into F-CAN bus as well?
// void sendCAN(uint8_t id, uint64_t data, uint8_t length = 8) {
//   twai_message_t message;
//   message.identifier = id;
//   message.extd = 0;
//   message.rtr = 0;
//   message.data_length_code = length;
//   message.data = data;
//   twai_transmit(&message, pdMS_TO_TICKS(1)); 
// }

