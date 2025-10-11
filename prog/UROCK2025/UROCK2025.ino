#include <ESP32Servo.h>

#define LEG1PIN1 32
#define LEG1PIN2 33

#define LEG2PIN1 25
#define LEG2PIN2 26

#define LEG3PIN1 19
#define LEG3PIN2 18

#define LEG4PIN1 17
#define LEG4PIN2 16

#define IR_PIN 12

#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_HERTZ 300

int err[4][2] = {{0,0}, {0,0}, {0,0}, {0,0}};

// ini jalan normal

int ANGLE_LEG1S1[4] = {75, 68, 59, 64}; 
int ANGLE_LEG1S2[4] = {45, 49, 24, 14};

int ANGLE_LEG2S1[3] = {0, 0, 0}; 
int ANGLE_LEG2S2[3] = {0, 0, 0}; 

int ANGLE_LEG3S1[3] = {0, 0, 0}; 
int ANGLE_LEG3S2[3] = {0, 0, 0}; 

int ANGLE_LEG4S1[3] = {0, 0, 0}; 
int ANGLE_LEG4S2[3] = {0, 0, 0}; 

// Bawah ni angle untuk climbing ramp suda, nanti cari belum cari lagi kan
//-----------------------------------------------
//nanti tukar ni dua ja kalau mau adjust cara jalan dia untuk leg 1
int ANGLE_CLIMBING_LEG1S1[4] = {75, 68, 59, 64}; 
int ANGLE_CLIMBING_LEG1S2[4] = {45, 49, 24, 14};
//-----------------------------------------------

int ANGLE_CLIMBING_LEG2S1[3] = {0, 0, 0}; 
int ANGLE_CLIMBING_LEG2S2[3] = {0, 0, 0}; 

int ANGLE_CLIMBING_LEG3S1[3] = {0, 0, 0}; 
int ANGLE_CLIMBING_LEG3S2[3] = {0, 0, 0}; 

int ANGLE_CLIMBING_LEG4S1[3] = {0, 0, 0}; 
int ANGLE_CLIMBING_LEG4S2[3] = {0, 0, 0}; 

unsigned long lastMoveTime = 0;
unsigned long currentMillis;
int stepIndex = 0;
bool isMoving = false;
bool isClimbingMode = false;

int customDelay = 300; //ubah ini kalau mau ubah delay dia
int sensor;

Servo LEG1S1;
Servo LEG1S2;

Servo LEG2S1;
Servo LEG2S2;

Servo LEG3S1;
Servo LEG3S2;

Servo LEG4S1;
Servo LEG4S2;

void setup() {
  Serial.begin(115200);
  setupLeg();
  standingLeg();
  delay(1000);
}

void loop() {
  
  sensor = digitalRead(IR_PIN);
  Serial.print("SENSOR: ");
  Serial.println(sensor);
  if(sensor == 0 && isClimbingMode){
    isClimbingMode = false;
    stepIndex = 0;
  } 
  else if(sensor == 1 && !isClimbingMode){
    isClimbingMode = true;
    stepIndex = 0;
  }

  if(isClimbingMode) {
    climbingL1();
  } else {
    moveL1();
  }

}

void setupLeg() {
  
  LEG1S1.setPeriodHertz(SERVO_HERTZ);
  LEG1S2.setPeriodHertz(SERVO_HERTZ);

  LEG2S1.setPeriodHertz(SERVO_HERTZ);
  LEG2S2.setPeriodHertz(SERVO_HERTZ);

  LEG3S1.setPeriodHertz(SERVO_HERTZ);
  LEG3S2.setPeriodHertz(SERVO_HERTZ);

  LEG4S1.setPeriodHertz(SERVO_HERTZ);
  LEG4S2.setPeriodHertz(SERVO_HERTZ);

  LEG1S1.attach(LEG1PIN1, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  LEG1S2.attach(LEG1PIN2, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  LEG2S1.attach(LEG2PIN1, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  LEG2S2.attach(LEG2PIN2, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  LEG3S1.attach(LEG3PIN1, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  LEG3S2.attach(LEG3PIN2, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  LEG4S1.attach(LEG4PIN1, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  LEG4S2.attach(LEG4PIN2, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

void standingLeg(){
  LEG1S1.write(80 + err[0][0]);
  LEG1S2.write(40 + err[0][1]);

  LEG2S1.write(80  + err[1][0]); 
  LEG2S2.write(40  + err[1][1]); 

  LEG3S1.write(80 + err[2][0]);
  LEG3S2.write(40 + err[2][1]);

  LEG4S1.write(80 + err[3][0]); 
  LEG4S2.write(40 + err[3][1]);
  delay(2000);
}

void startMoveL1() {
  stepIndex = 0;
  isMoving = true;
  lastMoveTime = millis();
}

void moveL1() {
  if (!isMoving) return;

  currentMillis = millis();
  
  if (currentMillis - lastMoveTime >= customDelay) { 
    lastMoveTime = currentMillis;

    LEG1S1.write(ANGLE_LEG1S1[stepIndex] - err[0][0]);
    LEG1S2.write(ANGLE_LEG1S2[stepIndex] - err[0][1]);

    stepIndex++;
    if (stepIndex >= 4) {
      stepIndex = 0;
      isMoving = false; 
    }
  }
}

void climbingL1() {
  if (!isMoving) return;

  currentMillis = millis();
  
  if (currentMillis - lastMoveTime >= customDelay) { 
    lastMoveTime = currentMillis;

    LEG1S1.write(ANGLE_CLIMBING_LEG1S1[stepIndex] - err[0][0]);
    LEG1S2.write(ANGLE_CLIMBING_LEG1S2[stepIndex] - err[0][1]);

    stepIndex++;
    if (stepIndex >= 4) {
      stepIndex = 0;
      isMoving = false; 
    }
  }
}