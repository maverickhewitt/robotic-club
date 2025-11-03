#include "leg.h"

void setup() {
  Serial.begin(115200);
  setupServo();

  // leg_i();
  // leg_k();
  // leg_l();
}

void loop() {
  settingServo();
  // SKL.write(60);
  // SKR.write(130);

  // SJL.write(60);
  // SJR.write(130);

  // SIL.write(60);
  // SIR.write(130);

  // SLL.write(60);
  // SLR.write(130);

  // delay(200);

  // SKL.write(40);
  // SKR.write(110);

  // SJL.write(40);
  // SJR.write(110);
  
  // SIL.write(40);
  // SIR.write(110);

  // SLL.write(40);
  // SLR.write(110);

  // delay(200);
  // move_IK();
  // move_JL();
}
 
