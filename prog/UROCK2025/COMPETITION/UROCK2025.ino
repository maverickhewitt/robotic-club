#include <main.h>

void setup() {
  Serial.begin(115200);
  setupLeg();
  // setupGyro();
  generateLegAngle();
  delay(200);
  standingLeg(); 
  delay(200);
  readyMove();
  delay(200);
}

void loop() {
  moveCycle();
}

