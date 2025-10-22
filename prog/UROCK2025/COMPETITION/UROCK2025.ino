#include <main.h>

void setup() {
  Serial.begin(115200);
  setupLeg();
  // setupGyro();
  // standingLeg(); 
  // generateLegPath();
}

void loop() {
  generateLegPath();
  moveAll();
  // standingLeg();
  // generateLegPathLeft();
  // moveL1Left();
  // generateLegPath();
  // moveBackwardL1();
  // moveL1();
  // settingServo();
}

