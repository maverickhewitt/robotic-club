#include <main.h>

void setup() {
  Serial.begin(115200);
  setupLeg();
  // setupGyro();
  // standingLeg(); 
}

void loop() {
  generateLegPath();
  moveL1();
  // generateLegPath();
  moveBackwardL1();
  // moveL1();
}

