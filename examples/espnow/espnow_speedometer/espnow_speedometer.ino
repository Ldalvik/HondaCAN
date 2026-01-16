#include <esp_now.h>
#include <WiFi.h>
#include <HondaCAN.h>
#include "lvgl.h"

HondaCAN CAN;
bool updateScreen = false;

void OnDataRecv(const uint8_t *addr, const uint8_t *incoming, int len) {
  if (len == sizeof(CAN.telemetry)) {
    memcpy(&CAN.telemetry, incoming, sizeof(CAN.telemetry));
    updateScreen = true;
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(OnDataRecv);
  
  lcd_setup();
}

void loop() {
  if (updateScreen) {
    update();
    updateScreen = false;
  }
}
