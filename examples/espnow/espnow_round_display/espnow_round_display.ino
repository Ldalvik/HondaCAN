#include <esp_now.h>
#include <WiFi.h>
#include <HondaCAN.h>
#include "lvgl.h"
#include "CST816D.h"

LGFX lcd;
LGFX_Sprite frame(&lcd);

#define I2C_SDA 4
#define I2C_SCL 5
#define TP_INT 0
#define TP_RST 1

CST816D touch(I2C_SDA, I2C_SCL, TP_RST, TP_INT);

// Channels 1, 6, and 11 have least signal overlap
// All receiving ESP32's MUST be on the same channel
#define ESPNOW_WIFI_CHANNEL 6

HondaCAN CAN;
volatile bool updateScreen = false;

enum class CurrentScreen {
  MAIN,
  TEMPERATURES,
  FUEL,

  COUNT  // keep at end
};

CurrentScreen currentScreen = CurrentScreen::MAIN;

void nextScreen() {
  int s = (int)currentScreen;
  s = (s + 1) % (int)CurrentScreen::COUNT;
  currentScreen = (CurrentScreen)s;
}

// ESP32 board version 2.0.17 -> 3.0.0 conflicts
#if ESP_IDF_VERSION_MAJOR >= 5
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len)
#endif
{
  if (len != sizeof(CAN.CanData)) return;
  memcpy(&CAN.CanData, data, sizeof(CAN.CanData));

  // Ideally have a dual-CAN bus board tapped into BCAN/FCAN and send data in one struct
  // if(len == sizeof(CAN.CanData.FCAN)) {
  //   memcpy(&CAN.CanData.FCAN, data, sizeof(CAN.CanData.FCAN));
  //   if(CAN.CanData.FCAN.id != 0) return; // not FCAN data
  // }
  // if(len == sizeof(CAN.CanData.BCAN)) {
  //   memcpy(&CAN.CanData.BCAN, data, sizeof(CAN.CanData.BCAN));
  //   if(CAN.CanData.BCAN.id != 1) return; // not BCAN data
  // }
}

void setup() {
  Serial.begin(115200);
  lcd_setup();
  touch.begin();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  static unsigned long lastMillis = 0;
  if (millis() - lastMillis >= 20) {
    lastMillis = millis();

    uint8_t gesture;
    if (touch.getGesture(&gesture)) nextScreen();
  }

  static CurrentScreen lastScreen = CurrentScreen::COUNT;
  if (currentScreen != lastScreen) {
    lastScreen = currentScreen;
    frame.fillSprite(TFT_BLACK);
  }

  switch (currentScreen) {
    case CurrentScreen::MAIN:
      drawMainScreen();
      break;
    case CurrentScreen::TEMPERATURES:
      drawTemperatureScreen();
      break;
    case CurrentScreen::FUEL:
      drawFuelScreen();
      break;

    default:
      break;
  }

  // const uint32_t framePeriodMs = 33;  // 30 FPS
  // static uint32_t lastFrameMs = 0;
  // uint32_t now = millis();

  // if ((uint32_t)(now - lastFrameMs) >= framePeriodMs) {
  //   lastFrameMs += framePeriodMs;
  //   update();
  // }
}