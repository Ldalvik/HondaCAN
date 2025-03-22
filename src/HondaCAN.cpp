#include "HondaCAN.h"

HondaCAN::HondaCAN() //: a(a), b(b)
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
    PowertrainData.PEDAL_GAS = (data[0] >> 7) & 0xFF;                    // (7-14)
    PowertrainData.ENGINE_RPM = (uint16_t)(data[2] << 8) | data[3];      // (23-38)
    PowertrainData.GAS_PRESSED = (data[4] >> 7) & 0x01;                  // (39)
    PowertrainData.ACC_STATUS = (data[4] >> 6) & 0x01;                   // (38)
    PowertrainData.BOH_17C = (data[4] >> 3) & 0x1F;                      // (37-41)
    PowertrainData.BRAKE_SWITCH = (data[4] >> 4) & 0x01;                 // (32)
    PowertrainData.BOH2_17C = (uint16_t)(data[5] << 2) | (data[6] >> 6); // (47-56)
    PowertrainData.BRAKE_PRESSED = (data[6] >> 5) & 0x01;                // (53)
    PowertrainData.BOH3_17C = (data[6] >> 1) & 0x1F;                     // (52-57)
    PowertrainData.COUNTER = (data[7] >> 6) & 0x03;                      // (61-62)
    PowertrainData.CHECKSUM = (data[7] >> 4) & 0x0F;                     // (59-62)
}

void HondaCAN::parseGearbox15T(uint8_t data[8])
{
   Gearbox_15T.GEAR_SHIFTER = (data[0] >> 0) & 0x3F;  // (5-0)
   Gearbox_15T.BOH = (data[0] >> 6) & 0x3F;           // (45-40)
   Gearbox_15T.GEAR2 = data[3];                       // (31-24)
   Gearbox_15T.GEAR = data[4];                        // (39-32)
   Gearbox_15T.ZEROS_BOH = (data[5] >> 6) & 0x03;     // (47-46)
   Gearbox_15T.COUNTER = (data[7] >> 6) & 0x03;       // (61-60)
   Gearbox_15T.CHECKSUM = (data[7] >> 4) & 0x0F;      // (59-56)
}

void HondaCAN::parseCarSpeed(uint8_t data[8])
{
  CarSpeedData.ROUGH_CAR_SPEED = data[2];                       // (23-16)
  CarSpeedData.CAR_SPEED = (data[0] | (data[1] << 8)) * 0.01;   // (7-0)
  CarSpeedData.ROUGH_CAR_SPEED_3 = (data[4] | (data[5] << 8));  // (39-24)
  CarSpeedData.ROUGH_CAR_SPEED_2 = data[3];                     // (31-24)
  CarSpeedData.LOCK_STATUS = (data[6] >> 6) & 0x03;             // (55-54)
  CarSpeedData.COUNTER = (data[7] >> 6) & 0x03;                 // (61-60)   
  CarSpeedData.CHECKSUM = (data[7] >> 4) & 0x0F;                // (59-56)
  CarSpeedData.IMPERIAL_UNIT = (data[7] >> 7) & 0x01;           // (63)
}

// void deviceVoltage(void) {
//   return analogRead(SENSE_V_ANA);
// }

// void sendCAN(uint8_t id, uint64_t data, uint8_t length = 8) {
//   twai_message_t message;
//   message.identifier = id;
//   message.extd = 0;
//   message.rtr = 0;
//   message.data_length_code = length;
//   message.data = data;
//   twai_transmit(&message, pdMS_TO_TICKS(1)); 
// }

uint64_t HondaCAN::byteConvert(uint8_t data[8])
{
  uint64_t result = 0;
  for (int i = 0; i < 8; ++i)
    result |= (uint64_t)data[i] << (8 * i);
  return result;
}