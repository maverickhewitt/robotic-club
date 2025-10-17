#include <main.h>

void setup() {
  Serial.begin(115200);
  setupLeg();
  setupGyro();
  standingLeg();
  delay(1000);
}

void loop() {
  
}

