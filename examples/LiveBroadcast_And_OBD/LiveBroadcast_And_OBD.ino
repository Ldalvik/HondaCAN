/*
  This file shows how you can read live broadcast data and send/read OBD PID commands at the same time.
  You can use other OBDII libraries that have a better queue system if you'd like, this is just an example
  of how you can do it with HondaCAN, as I use it to get information that I can't get from live broadcast data.
  Since the OBD functions are blocking, they are on the ESP32's 2nd core to not interrupt the 100hz update
  rate in the main loop.
*/

#include <HondaCAN.h>

HondaCAN CAN;
unsigned long lastMillis = 0;

volatile float obd_rpm = 0.0f;
void obdTask(void *p) 
{
  for (;;) 
  {
    float rpm = CAN.OBD.getEngineRPM();
    if (!isnan(rpm)) 
    {
      obd_rpm = rpm;
      vTaskDelay(pdMS_TO_TICKS(500));  // longer delays allow more consistent readings
    }
  }
}

void setup() 
{
  Serial.begin(115200);

  if (!CAN.begin()) return;
  delay(1000);

  xTaskCreatePinnedToCore(
    obdTask,
    "OBD Task",
    4096,
    nullptr,
    1,
    nullptr,
    0);
}

void loop() 
{
  CAN.run();
  // Keep update rate at maximum of 100hz. Some ECU modules run at a slower frequency, but the maximum is 100hz.
  if (millis() - lastMillis >= 10) 
  {
    Serial.print(CAN.ParsedCanData.getEngineRPM());
    Serial.print(" - ");
    Serial.println(obd_rpm);
    lastMillis = millis();
  }
}