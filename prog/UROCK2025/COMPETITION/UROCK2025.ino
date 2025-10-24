#include <main.h>

void setup() {
  Serial.begin(115200);
  setupLeg();
  // setupGyro();
  // standingLeg(); 
  generateLegPathLeft();
  // generateLegPathRight();
}

void loop() {
  // generateLegPath();
  // generateLegPathRight();
  // moveAll();
  // standingLeg();
  // generateLegPathLeft();
  // moveL1Left();
  // generateLegPath();
  // moveBackwardL1();
  // moveL1();
  moveL2();
  moveBackL2();
  // settingServo();
  // delay(500);
}

