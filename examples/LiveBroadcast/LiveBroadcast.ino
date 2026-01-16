#include <HondaCAN.h>

HondaCAN CAN;
unsigned long lastMillis = 0;

void setup() 
{
  Serial.begin(115200);

  if (!CAN.begin()) return;
  delay(1000);  // give time for serial monitor to load

  // Should reliably get the VIN on most OBDII gateways. Support for other vehicles being added soon
  const char* vin = CAN.VIN.get_vin(); // param allows multiple attempts with an increased wait for cars with slow ECU responses
  Serial.print("VIN: ");
  Serial.println(String(vin));
}

void loop() 
{
  CAN.run();
  // Keep update rate at maximum of 100hz. Some ECU modules run at a slower frequency, but the maximum is 100hz.
  // This mostly helps with consistent updates when sending data to LCD screens for example. (see esp-now CAN examples)
  if (millis() - lastMillis >= 10) 
  {
    Serial.print("RPM: ");
    Serial.println(CAN.ParsedCanData.getEngineRPM());

    // Raw CAN data, see files to see the difference in some functions
    // Serial.print("RPM: ");
    // Serial.print(CAN.RawCanData.PowertrainData.ENGINE_RPM);
    lastMillis = millis();
  }
}