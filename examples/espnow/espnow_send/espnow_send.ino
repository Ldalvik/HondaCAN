/*
  This architecture is great for UI and low priority data transfer

*/
#include <WiFi.h>
#include <esp_now.h>
#include <driver/twai.h>
#include <stdint.h>

struct VehicleData {
  uint16_t EngineRPM;
  uint8_t EngineTemp;
  uint8_t IntakeTemp;
};

VehicleData CanData = {0};

// Channels 1, 6, and 11 have least signal overlap
// All receiving ESP32's MUST be on the same channel
#define ESPNOW_WIFI_CHANNEL 6

uint8_t addr[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };  // broadcast address

// ESP32 board version 2.0.17 -> 3.0.0 conflicts
#if ESP_IDF_VERSION_MAJOR >= 5 
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
#else
void OnDataSent(const uint8_t *addr, esp_now_send_status_t status)
#endif
{}

bool TwaiSetup() {
  const int RX_PIN = 4;
  const int TX_PIN = 5;
  const int CAN_RS = 21;

  pinMode(CAN_RS, OUTPUT);
  digitalWrite(CAN_RS, LOW);  // LOW = high speed mode, HIGH = low power mode (listen only)

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;

  return true;
}

void ReadCan() {  // frames for 2013-2017 Accord
  twai_message_t frame;
  if (twai_receive(&frame, pdMS_TO_TICKS(10)) != ESP_OK) return;

  switch (frame.id) {
    case 0x324:
      {  // byte0 = engine temp, byte1 = intake temp, byte3 = fuel consumed
        if (frame.dlc != 8) return;
        CanData.EngineTemp = frame.data[0] - 40;
        CanData.IntakeTemp = frame.data[1] - 40;
      }
      break;

    case 0x1DC:
      {  // byte1 = RPM_H, byte2 = RPM_L
        if (frame.dlc != 4) return;
        CanData.EngineRPM = ((uint16_t)frame.data[1] << 8) | frame.data[2];
      }
      break;

    default:
      break;
  }
}

void setup() {
  // Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, addr, 6);

  if (!esp_now_is_peer_exist(addr))
    esp_now_add_peer(&peerInfo);

  TwaiSetup();
}

void loop() {
  ReadCan();

  static uint32_t lastMillis = 0;
  uint32_t now = millis();

  if (now - lastMillis >= 10) {  // Limits update rate to 100 Hz
    esp_now_send(addr, (uint8_t*)&CanData, sizeof(VehicleData));
    lastMillis = now;
  }
}