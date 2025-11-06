#include "leg.h"
#include "legv2.h"
#include "ir.h"

unsigned long startTime;
bool isReady = true;

void setup() {
  Serial.begin(115200);
  setupServo();
  leg_i();
  leg_l();
  leg_k();
  // // leg_i_ramp();
  // // leg_l_ramp();W
  // // leg_k_ramp();
  // // leg_j_ramp();

  initLeg2();
  delay(1000);
  startTime = millis();
}

void loop() {
  if(millis() - startTime >= 32000){
    walkPatternGood();
  }else if(millis() - startTime >= 22000){
    walkPatternLow();
  }else{
    walkPatternGood();
  }
  // if(isReady){
  //   stompLeg();
  //   if(millis() - startTime >= 2000){
  //     isReady = false;
  //   }
  // }else{
  //   firstStep();
  //   // moveAll();
  // }
}
 
