#include <HondaCAN.h>
#include <esp_now.h>
#include <WiFi.h>

uint8_t addr[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };  // broadcast address

HondaCAN CAN;
unsigned long lastMillis = 0;
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *addr, esp_now_send_status_t status) {}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_send_cb(OnDataSent);

  peerInfo.channel = 0;  // recv devices must be on the same channel
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, addr, 6);

  if (!esp_now_is_peer_exist(addr)) {
    esp_now_add_peer(&peerInfo);
  }
  if (!CAN.macchina_a0_setup()) return;
}

void loop() {
  CAN.run();

  // 100Hz max update rate
  if (millis() - lastMillis >= 10) {
    updateTelemetry();
    esp_now_send(addr, (uint8_t *)&CAN.telemetryFrame, sizeof(CAN.telemetryFrame));
    lastMillis = millis();
  }
}

void updateTelemetry() {
  CAN.telemetryFrame.speed = CAN.ParsedCanData.getSpeed_Mph();
  CAN.telemetryFrame.rpm = CAN.ParsedCanData.getEngineRPM();
  CAN.telemetryFrame.trip_distance = CAN.ParsedCanData.getTripDistance_Mi();
  CAN.telemetryFrame.fuel_consumed = CAN.ParsedCanData.getTripFuelConsumed_gal();
  CAN.telemetryFrame.fuel_level = CAN.ParsedCanData.getFuelLevel();
  CAN.telemetryFrame.current_gear = CAN.ParsedCanData.getCurrentGear();
  CAN.telemetryFrame.scm_buttons = CAN.ParsedCanData.getScmButtonPressed();
  CAN.telemetryFrame.cruise_main_on = CAN.ParsedCanData.isCruiseMainOn();
}
