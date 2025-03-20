#include <HondaCAN.h>

HondaCAN CAN;

void setup() {
  Serial.begin(115200);
  if(!CAN.begin()) Serial.println("Failed to install or start TWAI driver!");
}

void loop() {
  CAN.run();
  Serial.println(CAN.PowertrainData.ENGINE_RPM);
}
 
