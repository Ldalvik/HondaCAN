#include <esp_now.h>
#include <WiFi.h>

struct VehicleData {
  uint16_t EngineRPM;
  uint8_t EngineTemp;
  uint8_t IntakeTemp;
};

VehicleData CanData = { 0 };
volatile bool newData = false;

// Channels 1, 6, and 11 have least signal overlap
// All receiving ESP32's MUST be on the same channel
#define ESPNOW_WIFI_CHANNEL 6

// ESP32 board version 2.0.17 -> 3.0.0 conflicts
#if ESP_IDF_VERSION_MAJOR >= 5
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len)
#endif
{
  if (len != sizeof(VehicleData)) return;
  memcpy(&CanData, data, sizeof(VehicleData));
  newData = true;
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Write CAN data handling code here.
  // The CanData struct should be updated at 100hz, if you keep the 10ms millis timer in the host code.
  VehicleData data;
  bool dataReady;

  noInterrupts();
  data = CanData;
  dataReady = newData;
  newData = false;
  interrupts();

  if (dataReady) {
    Serial.println("Engine RPM: ");
    Serial.println(data.EngineRPM);
  }
}