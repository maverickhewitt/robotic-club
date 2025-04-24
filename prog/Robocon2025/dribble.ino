#include <CytronMotorDriver.h>
#define IR_PIN 32


int sensor;
int ballPassCounter = 0;
bool isBallPass = false;

CytronMD motorLeft(PWM_DIR, 18, 19); // PWM 1 = Pin 18, DIR 1 = Pin 19
CytronMD motorRight(PWM_DIR, 22, 23);

void setup() {
  Serial.begin(115200);
  pinMode(IR_PIN, INPUT);
}

void loop() {
  sensor = digitalRead(IR_PIN);
  Serial.print("Value:");
  Serial.println(sensor);

  motorLeft.setSpeed(-255);
  motorRight.setSpeed(255);
  
  if(sensor = 1){
    isBallPass = true;
  }

  if(isBallPass){
    ballPassCounter++;
    isBallPass = false;
  }

  if(ballPassCounter == 1){
    motorLeft.setSpeed(255);
    motorRight.setSpeed(-255);
  }
  
  if(ballPassCounter == 2){
    delay(200);
    motorLeft.setSpeed(0);
    motorRight.setSpeed(0);
  }
}
