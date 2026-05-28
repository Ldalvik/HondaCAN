#include "HondaCAN.h"

HondaCAN::HondaCAN() {}

bool HondaCAN::macchina_a0_setup()
{
  pinMode(CAN_RS, OUTPUT);
  digitalWrite(CAN_RS, LOW);             // LOW = high speed mode, HIGH = low power mode (listen only)
  analogSetPinAttenuation(35, ADC_11db); // Power sensing
  analogReadResolution(12);              // Power sensing

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    return false;
  if (twai_start() != ESP_OK)
    return false;

  return true;
}

void HondaCAN::run()
{
  twai_message_t frame;
  if (twai_receive(&frame, pdMS_TO_TICKS(10)) != ESP_OK)
    return;

  if (profile_ && profile_->parser)
    profile_->parser(frame.identifier, frame.data, frame.data_length_code, CanData);
   
  if (CanData.PCM.EngineRPM == 0 && batteryVoltage() < 12.5f)
  { // reset overflow values when vehicle is off
    CanData.MISC.TripDistanceOverflow = 0;
    CanData.MISC.TripDistanceLast = CanData.PCM.TripDistance;
  }
}

float HondaCAN::batteryVoltage()
{
  float vOut = (analogRead(POWER_SENSE) * 3.3) / 4095;
  float vIn = vOut / 0.18032786885; // resistor math magic number (R2 / (R1 + R2))
  return vIn;
}

void HondaCAN::setVehicleProfile(const VehicleProfile &p)
{
  profile_ = &p;
}

const char *HondaCAN::vehicleName() const
{
  return profile_ ? profile_->name : "None";
}