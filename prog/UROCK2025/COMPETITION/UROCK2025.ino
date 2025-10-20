#include <main.h>

void setup() {
  Serial.begin(115200);
  setupLeg();
  // setupGyro();
  generateLegPath();
  // standingLeg(); 
  delay(500);
}

void loop() {
  moveL1();
}

