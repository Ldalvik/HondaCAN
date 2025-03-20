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
      this->parsePowertrainData(this->byteConvert(message.data));
      break;

    }
  }
}

void HondaCAN::parsePowertrainData(uint64_t data)
{
  PowertrainData.PEDAL_GAS = (data >> 7) & 0xFF;      // (8 bits, 7-14)
  PowertrainData.ENGINE_RPM = (data >> 23) & 0xFFFF;  // (16 bits, 23-38)
  PowertrainData.GAS_PRESSED = (data >> 39) & 0x01;   // (1 bit, bit 39)
  PowertrainData.ACC_STATUS = (data >> 38) & 0x01;    // (1 bit, bit 38)
  PowertrainData.BOH_17C = (data >> 37) & 0x1F;       // (5 bits, bits 37-41)
  PowertrainData.BRAKE_SWITCH = (data >> 32) & 0x01;  // (1 bit, bit 32)
  PowertrainData.BOH2_17C = (data >> 47) & 0x3FF;     // (10 bits, bits 47-56)
  PowertrainData.BRAKE_PRESSED = (data >> 53) & 0x01; // (1 bit, bit 53)
  PowertrainData.BOH3_17C = (data >> 52) & 0x1F;      // (5 bits, bits 52-57)
  PowertrainData.COUNTER = (data >> 61) & 0x03;       // (2 bits, bits 61-62)
  PowertrainData.CHECKSUM = (data >> 59) & 0x0F;      // (4 bits, bits 59-62)
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
    result |= (uint64_t)data[i] << (8 * (7 - i));
  return result;
}