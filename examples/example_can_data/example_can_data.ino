#include <HondaCAN.h>

HondaCAN CAN;
unsigned long lastMillis = 0;

void setup() {
  Serial.begin(115200);

  if (!CAN.macchina_a0_setup()) return;
  CAN.setVehicleProfile(Accord_9_5); // Setting a vehicle is REQUIRED

  delay(1000);  // give time for serial monitor to load

  // Should reliably get the VIN on most OBDII gateways after 2008. Support for other vehicles being added soon
  const char* vin = CAN.VIN.get_vin(3);  // param allows multiple attempts with an increased wait for cars with slow ECU responses
  Serial.print("VIN: ");
  Serial.println(String(vin));
}

void loop() {
  CAN.run();
  // Keep update rate at maximum of 100hz. Some ECU modules run at a faster/slower frequency, but this keeps it stable.
  // This mostly helps with consistent updates when sending data to LCD screens for example. (see esp-now CAN examples)
  if (millis() - lastMillis >= 10) {
    float tripDistance = CAN.CanData.PCM.TripDistance;
    float fuelConsumed = CAN.CanData.PCM.TripFuelConsumed;
    float mpg = tripDistance / fuelConsumed;

    char mpgStr[16];
    sprintf(mpgStr, "%04.1fmpg", mpg);
    Serial.print(mpg);
    Serial.print("(");
    Serial.print(fuelConsumed);
    Serial.print("gal, ");
    Serial.print(tripDistance);
    Serial.println("mi)");

    lastMillis = millis();
  }
}