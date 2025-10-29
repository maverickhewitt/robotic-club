#include "ServoEasing.hpp"
#include <ESP32Servo.h>
#include <math.h>
#include <Ramp.h>

#define LEG1PIN1 26
#define LEG1PIN2 27

#define LEG2PIN1 26
#define LEG2PIN2 27

#define LEG3PIN1 26
#define LEG3PIN2 27

#define LEG4PIN1 26
#define LEG4PIN2 27

#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_HERTZ 300

#define RAD_TO_DEG 57.295779513

#define APD 20
#define SPEED 50
#define SPEED2 SPEED - 10
#define COND EASE_CUBIC_IN_OUT
#define ARRAY_NO 11

// #define a 15.0
// #define b 17.0
// #define h 25.0
// #define L 10.0
// #define x 5.00
// #define H 5.0

float a = 15.0;
float b = 17.0;
float h = 25.0;
float L = 10.0;
float L2 = -10.0;
float x = 5.0;
float H = 5.0;
float H_LOW = -2.0;

float x0 = 0;
float yZero = h;

float x1 = L/2 ;
float yOne = h - 2 * H;

float x2 = L;

float yTwo = h;

//26<<knee<<96
int err[4][2] = {{3,8}, {8,6}, {8,6}, {6,7}};

float ANGLE_LEG1S1[ARRAY_NO];
float ANGLE_LEG1S2[ARRAY_NO];

int ANGLE_LEG2S1[ARRAY_NO];
int ANGLE_LEG2S2[ARRAY_NO];
int BACK_ANGLE_LEG2S1[ARRAY_NO];
int BACK_ANGLE_LEG2S2[ARRAY_NO];

int SECANGLE_LEG2S1[3] = {27,32,55};
int SECANGLE_LEG2S2[3] = {64,51,78};
int SECBACK_ANGLE_LEG2S1[3] = {55,37,27};
int SECBACK_ANGLE_LEG2S2[3] = {78,64,64};

int ANGLE_LEG3S1[ARRAY_NO];
int ANGLE_LEG3S2[ARRAY_NO];

int ANGLE_LEG4S1[ARRAY_NO];
int ANGLE_LEG4S2[ARRAY_NO];

float LEFT_ANGLE_LEG1S1[ARRAY_NO];
float LEFT_ANGLE_LEG1S2[ARRAY_NO];

int CLIMB_ANGLE_LEG1S1[ARRAY_NO];
int CLIMB_ANGLE_LEG1S2[ARRAY_NO];

int CLIMB_ANGLE_LEG2S1[ARRAY_NO];
int CLIMB_ANGLE_LEG2S2[ARRAY_NO];

int CLIMB_ANGLE_LEG3S1[ARRAY_NO];
int CLIMB_ANGLE_LEG3S2[ARRAY_NO];

int CLIMB_ANGLE_LEG4S1[ARRAY_NO];
int CLIMB_ANGLE_LEG4S2[ARRAY_NO];

ServoEasing LEG1S1;
ServoEasing LEG1S2;

ServoEasing LEG2S1;
ServoEasing LEG2S2;

ServoEasing LEG3S1;
ServoEasing LEG3S2;

ServoEasing LEG4S1;
ServoEasing LEG4S2;

rampFloat LEG1RAMP;
rampFloat KNEE1RAMP;

rampFloat LEG2RAMP;
rampFloat KNEE2RAMP;

rampFloat LEG3RAMP;
rampFloat KNEE3RAMP;

rampFloat LEG4RAMP;
rampFloat KNEE4RAMP;

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

  LEG1S1.setEasingType(COND);
  LEG1S2.setEasingType(COND);

  LEG2S1.setEasingType(COND);
  LEG2S2.setEasingType(COND);

  LEG3S1.setEasingType(COND);
  LEG3S2.setEasingType(COND);

  LEG4S1.setEasingType(COND);
  LEG4S2.setEasingType(COND);
}

float bezierQuadratic(float p0, float p1, float p2, float t) { 
  return pow(1 - t, 2) * p0 +
         2 * (1 - t) * t * p1 +
         pow(t, 2) * p2;
}


// void findAngleLeft(float i, int stepIndex) {   
//   float t = (float)i / L ;
//   // float coorX = bezierQuadratic(x0, x1, x2, t);
//   // float coorY = bezierQuadratic(y0, y1, y2, t);
//   float coorX = L * (t - (sin(2 * PI * t) / (2 * PI)));
//   float coorY = (H / 2) * (1 - cos(2 * PI * t));

//   float d = sqrt(sq(x - coorX) + sq(h - coorY));

//   float knee = acos((sq(a) + sq(b) - sq(d)) / (2 * a * b));
//   float hip;
//   if(coorX > x) {
//     hip = (PI / 2) + atan(coorX / h) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
//   }else if (coorX < x) {
//     hip = (PI / 2) - atan(coorX / h) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
//   }else if (coorX == x) {
//     hip = (PI / 2) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
//   }
//   // float hip = (PI / 2) + asin(coorX / d) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
//   // float hip = (PI/2) + asin(coorX / d) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
//   // float hip = (PI/2) + atan(coorX / h);

//   float hipDeg = hip * RAD_TO_DEG;
//   float kneeDeg = knee * RAD_TO_DEG;

//   // float convHip =hipDeg * 2 / 3; //no need minus 120 kalau kaki right
//   float convHip = 120 - (hipDeg * 2 / 3); //no need minus 120 kalau kaki right
//   float convKnee =kneeDeg * 2 / 3;

//   ANGLE_LEG1S1[stepIndex] = convHip;
//   ANGLE_LEG1S2[stepIndex] = convKnee;

//   ANGLE_LEG3S1[stepIndex] = convHip;
//   ANGLE_LEG3S2[stepIndex] = convKnee;

//   Serial.print("Step ");
//   Serial.print(stepIndex);
//   Serial.print(" | d ");
//   Serial.print(d);
//   Serial.print("Coor X: ");
//   Serial.print(coorX);
//   Serial.print("Coor Y: ");
//   Serial.print(coorY);
//   Serial.print(" | Hip: ");
//   Serial.print(convHip);
//   Serial.print(" | Knee: ");
//   Serial.println(convKnee);
// }

void findAngleLeft(float i, int stepIndex) {   
  float t = (float)i / L;
  // float coorX = bezierQuadratic(x0, x1, x2, t);
  // float coorY = bezierQuadratic(yZero, yOne, yTwo, t);
  float coorX = L * (t - (sin(2 * PI * t) / (2 * PI)));
  float coorY = (H / 2) * (1 - cos(2 * PI * t));

  float d = sqrt(sq(x - coorX) + sq(h - coorY));

  float knee = acos((sq(a) + sq(b) - sq(d)) / (2 * a * b));
  float hip;

  if(coorX > x) {
    hip = (PI / 2) + atan(coorX / h) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  }else if (coorX < x) {
    hip = (PI / 2) - atan(coorX / h) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  }else if (coorX == x) {
    hip = (PI / 2) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  }
  // float hip = (PI / 2) + asin(coorX / d) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  // float hip = (PI/2) + asin(coorX / d) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  // float hip = (PI/2) + atan(coorX / h);

  float hipDeg = hip * RAD_TO_DEG;
  float kneeDeg = knee * RAD_TO_DEG;

  // float convHip =hipDeg * 2 / 3; //no need minus 120 kalau kaki right
  float convHip = 120 - (hipDeg * 2 / 3); //no need minus 120 kalau kaki right
  float convKnee =round(120 - (kneeDeg * 2 / 3));

  ANGLE_LEG1S1[stepIndex] = convHip;
  ANGLE_LEG1S2[stepIndex] = convKnee;

  ANGLE_LEG3S1[stepIndex] = convHip;
  ANGLE_LEG3S2[stepIndex] = convKnee;

  Serial.print("Step ");
  Serial.print(stepIndex);
  Serial.print(" | d ");
  Serial.print(d);
  Serial.print("Coor X: ");
  Serial.print(coorX);
  Serial.print("Coor Y: ");
  Serial.print(coorY);
  Serial.print(" | Hip: ");
  Serial.print(convHip);
  Serial.print(" | Knee: ");
  Serial.println(convKnee);
}

void findAngleRight(float i, int stepIndex) {   
  float t = (float)i / L;
  // float coorX = bezierQuadratic(x0, x1, x2, t);
  // float coorY = bezierQuadratic(yZero, yOne, yTwo, t);
  float coorX = L * (t - (sin(2 * PI * t) / (2 * PI)));
  float coorY = (H / 2) * (1 - cos(2 * PI * t));

  float d = sqrt(sq(x - coorX) + sq(h - coorY));

  float knee = acos((sq(a) + sq(b) - sq(d)) / (2 * a * b));
  float hip;

  if(coorX > x) {
    hip = (PI / 2) + atan(coorX / h) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  }else if (coorX < x) {
    hip = (PI / 2) - atan(coorX / h) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  }else if (coorX == x) {
    hip = (PI / 2) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  }
  // float hip = (PI / 2) + asin(coorX / d) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  // float hip = (PI/2) + asin(coorX / d) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  // float hip = (PI/2) + atan(coorX / h);

  float hipDeg = hip * RAD_TO_DEG;
  float kneeDeg = knee * RAD_TO_DEG;

  float convHip = hipDeg * 2 / 3; //no need minus 120 kalau kaki right
  // float convHip = 120 - (hipDeg * 2 / 3); //no need minus 120 kalau kaki right
  float convKnee = round(kneeDeg * 2 / 3);

  ANGLE_LEG2S1[stepIndex] = convHip;
  ANGLE_LEG2S2[stepIndex] = convKnee;

  ANGLE_LEG4S1[stepIndex] = convHip;
  ANGLE_LEG4S2[stepIndex] = convKnee;

  Serial.print("Step ");
  Serial.print(stepIndex);
  Serial.print(" | d ");
  Serial.print(d);
  Serial.print("Coor X: ");
  Serial.print(coorX);
  Serial.print("Coor Y: ");
  Serial.print(coorY);
  Serial.print(" | Hip: ");
  Serial.print(convHip);
  Serial.print(" | Knee: ");
  Serial.println(convKnee);
}

void generateLegAngle() {
  for (int i = 0; i <= L; i++) {
    findAngleRight(i, i);  
    findAngleLeft(i, i); 
  }
}


void standingLeg() {
  int startIndex = 0;  

  LEG1S1.write(ANGLE_LEG1S1[startIndex] + err[0][0]);
  LEG1S2.write(ANGLE_LEG1S2[startIndex] + err[0][1]);

  LEG2S1.write(ANGLE_LEG2S1[startIndex] + err[1][0]);
  LEG2S2.write(ANGLE_LEG2S2[startIndex] - err[1][1]);

  LEG3S1.write(ANGLE_LEG3S1[startIndex] + err[2][0]);
  LEG3S2.write(ANGLE_LEG3S2[startIndex] + err[2][1]);

  LEG4S1.write(ANGLE_LEG4S1[startIndex] + err[3][0]);
  LEG4S2.write(ANGLE_LEG4S2[startIndex] - err[3][1]);

}

void readyMove() {
  int frontReady = 4;
  int backReady = L;

  int L1_target = ANGLE_LEG1S1[frontReady] + err[0][0];
  int K1_target = ANGLE_LEG1S2[frontReady] + err[0][1];

  int L2_target = ANGLE_LEG2S1[backReady] + err[1][0];
  int K2_target = ANGLE_LEG2S2[backReady] - err[1][1];

  int L3_target = ANGLE_LEG3S1[backReady] + err[2][0];
  int K3_target = ANGLE_LEG3S2[backReady] + err[2][1];

  int L4_target = ANGLE_LEG4S1[frontReady] + err[3][0];
  int K4_target = ANGLE_LEG4S2[frontReady] - err[3][1];

  LEG1RAMP.go(L1_target, SPEED);
  KNEE1RAMP.go(K1_target, SPEED);
  LEG2RAMP.go(L2_target, SPEED);
  KNEE2RAMP.go(K2_target, SPEED);
  LEG3RAMP.go(L3_target, SPEED);
  KNEE3RAMP.go(K3_target, SPEED);
  LEG4RAMP.go(L4_target, SPEED);
  KNEE4RAMP.go(K4_target, SPEED);

  while (!LEG1RAMP.isFinished() || !LEG2RAMP.isFinished() ||
         !LEG3RAMP.isFinished() || !LEG4RAMP.isFinished()) {

    LEG1RAMP.update(); KNEE1RAMP.update();
    LEG2RAMP.update(); KNEE2RAMP.update();
    LEG3RAMP.update(); KNEE3RAMP.update();
    LEG4RAMP.update(); KNEE4RAMP.update();

    LEG1S1.write(LEG1RAMP.getValue());
    LEG1S2.write(KNEE1RAMP.getValue());
    LEG2S1.write(LEG2RAMP.getValue());
    LEG2S2.write(KNEE2RAMP.getValue());
    LEG3S1.write(LEG3RAMP.getValue());
    LEG3S2.write(KNEE3RAMP.getValue());
    LEG4S1.write(LEG4RAMP.getValue());
    LEG4S2.write(KNEE4RAMP.getValue());
  }
}


void moveCycle() {
  //PHASE 1: L1 + L4 move forward while L2 + L3 hold or slightly drag
  for (int i = 4; i <= L; i++) {
    int backIndex = constrain((L - 1) - (i - 4), 0, L);
    int L1 = ANGLE_LEG1S1[i] + err[0][0];
    int K1 = ANGLE_LEG1S2[i] + err[0][1];
    int L4 = ANGLE_LEG4S1[i] + err[3][0];
    int K4 = ANGLE_LEG4S2[i] + err[3][1];

    // Legs that stay (L2, L4) use neutral or backward dragging positions
    int L2 = ANGLE_LEG2S1[backIndex] + err[1][0];  
    int K2 = 74 - err[1][1];                  
    int L3 = ANGLE_LEG3S1[backIndex] + err[2][0];  
    int K3 = 46 - err[2][1];

    Serial.print("Reverse Index: ");                 
    Serial.println(backIndex);                

    // Start all ramps
    LEG1RAMP.go(L1, SPEED);
    KNEE1RAMP.go(K1, SPEED);
    LEG2RAMP.go(L2, SPEED);
    KNEE2RAMP.go(K2, SPEED);
    LEG3RAMP.go(L3, SPEED);
    KNEE3RAMP.go(K3, SPEED);
    LEG4RAMP.go(L4, SPEED);
    KNEE4RAMP.go(K4, SPEED);

    while (!LEG1RAMP.isFinished() || !LEG2RAMP.isFinished() ||
           !LEG3RAMP.isFinished() || !LEG4RAMP.isFinished()) {

      LEG1RAMP.update(); KNEE1RAMP.update();
      LEG2RAMP.update(); KNEE2RAMP.update();
      LEG3RAMP.update(); KNEE3RAMP.update();
      LEG4RAMP.update(); KNEE4RAMP.update();

      LEG1S1.write(LEG1RAMP.getValue());
      LEG1S2.write(KNEE1RAMP.getValue());
      LEG2S1.write(LEG2RAMP.getValue());
      LEG2S2.write(KNEE2RAMP.getValue());
      LEG3S1.write(LEG3RAMP.getValue());
      LEG3S2.write(KNEE3RAMP.getValue());
      LEG4S1.write(LEG4RAMP.getValue());
      LEG4S2.write(KNEE4RAMP.getValue());
    }
  }

  // PHASE 2: L2 + L3 move forward while L1 + L4 drag or hold 
  for (int i = 4; i <= L; i++) {
    int backIndex = constrain((L - 1) - (i - 4), 0, L);
    int L2 = ANGLE_LEG2S1[i] + err[1][0];
    int K2 = ANGLE_LEG2S2[i] - err[1][1];
    int L3 = ANGLE_LEG3S1[i] + err[2][0];
    int K3 = ANGLE_LEG3S2[i] - err[2][1];

    // Supporting legs move backward (drag)
    int L1 = ANGLE_LEG1S1[backIndex] + err[0][0];
    int K1 = 46 + err[0][1];
    int L4 = ANGLE_LEG4S1[backIndex] + err[3][0];
    int K4 = 74 + err[3][1];

    Serial.print("Reverse Index: ");                 
    Serial.println(backIndex);  

    LEG1RAMP.go(L1, SPEED);
    KNEE1RAMP.go(K1, SPEED);
    LEG2RAMP.go(L2, SPEED);
    KNEE2RAMP.go(K2, SPEED);
    LEG3RAMP.go(L3, SPEED);
    KNEE3RAMP.go(K3, SPEED);
    LEG4RAMP.go(L4, SPEED);
    KNEE4RAMP.go(K4, SPEED);

    while (!LEG1RAMP.isFinished() || !LEG2RAMP.isFinished() ||
           !LEG3RAMP.isFinished() || !LEG4RAMP.isFinished()) {

      LEG1RAMP.update(); KNEE1RAMP.update();
      LEG2RAMP.update(); KNEE2RAMP.update();
      LEG3RAMP.update(); KNEE3RAMP.update();
      LEG4RAMP.update(); KNEE4RAMP.update();

      LEG1S1.write(LEG1RAMP.getValue());
      LEG1S2.write(KNEE1RAMP.getValue());
      LEG2S1.write(LEG2RAMP.getValue());
      LEG2S2.write(KNEE2RAMP.getValue());
      LEG3S1.write(LEG3RAMP.getValue());
      LEG3S2.write(KNEE3RAMP.getValue());
      LEG4S1.write(LEG4RAMP.getValue());
      LEG4S2.write(KNEE4RAMP.getValue());
    }
  }
}

void moveFoward() {
  for (int i = 4; i <= L; i++) {
    int L1 = ANGLE_LEG1S1[i] + err[0][0];
    int K1 = ANGLE_LEG1S2[i] + err[0][1];
    int L2 = ANGLE_LEG2S1[i] + err[1][0];
    int K2 = ANGLE_LEG2S2[i] - err[1][1];
    int L3 = ANGLE_LEG3S1[i] + err[2][0];
    int K3 = ANGLE_LEG3S2[i] + err[2][1];
    int L4 = ANGLE_LEG4S1[i] + err[3][0];
    int K4 = ANGLE_LEG4S2[i] - err[3][1];

    LEG1RAMP.go(L1, SPEED);
    KNEE1RAMP.go(K1, SPEED);
    LEG2RAMP.go(L2, SPEED);
    KNEE2RAMP.go(K2, SPEED);
    LEG3RAMP.go(L3, SPEED);
    KNEE3RAMP.go(K3, SPEED);
    LEG4RAMP.go(L4, SPEED);
    KNEE4RAMP.go(K4, SPEED);

    while (!LEG1RAMP.isFinished() || !LEG2RAMP.isFinished() ||
           !LEG3RAMP.isFinished() || !LEG4RAMP.isFinished()) {

      LEG1RAMP.update(); KNEE1RAMP.update();
      LEG2RAMP.update(); KNEE2RAMP.update();
      LEG3RAMP.update(); KNEE3RAMP.update();
      LEG4RAMP.update(); KNEE4RAMP.update();

      LEG1S1.write(LEG1RAMP.getValue());
      LEG1S2.write(KNEE1RAMP.getValue());
      LEG2S1.write(LEG2RAMP.getValue());
      LEG2S2.write(KNEE2RAMP.getValue());
      LEG3S1.write(LEG3RAMP.getValue());
      LEG3S2.write(KNEE3RAMP.getValue());
      LEG4S1.write(LEG4RAMP.getValue());
      LEG4S2.write(KNEE4RAMP.getValue());
    }
  }
}

void moveBack() {
  for (int i = L- 1; i >= 5; i--) {
    int L1 = ANGLE_LEG1S1[i] + err[0][0];
    int K1 = 46 + err[0][1];
    int L2 = ANGLE_LEG2S1[i] + err[1][0];
    int K2 = 74 - err[1][1];
    int L3 = ANGLE_LEG3S1[i] + err[2][0];
    int K3 = 46 + err[2][1];
    int L4 = ANGLE_LEG4S1[i] + err[3][0];
    int K4 = 74 - err[3][1];

    LEG1RAMP.go(L1, SPEED2);
    KNEE1RAMP.go(K1, SPEED2);
    LEG2RAMP.go(L2, SPEED2);
    KNEE2RAMP.go(K2, SPEED2);
    LEG3RAMP.go(L3, SPEED2);
    KNEE3RAMP.go(K3, SPEED2);
    LEG4RAMP.go(L4, SPEED2);
    KNEE4RAMP.go(K4, SPEED2);

    while (!LEG1RAMP.isFinished() || !LEG2RAMP.isFinished() ||
           !LEG3RAMP.isFinished() || !LEG4RAMP.isFinished() ||
           !KNEE1RAMP.isFinished() || !KNEE2RAMP.isFinished() ||
           !KNEE3RAMP.isFinished() || !KNEE4RAMP.isFinished()) {

      LEG1RAMP.update(); KNEE1RAMP.update();
      LEG2RAMP.update(); KNEE2RAMP.update();
      LEG3RAMP.update(); KNEE3RAMP.update();
      LEG4RAMP.update(); KNEE4RAMP.update();

      LEG1S1.write(LEG1RAMP.getValue());
      LEG1S2.write(KNEE1RAMP.getValue());
      LEG2S1.write(LEG2RAMP.getValue());
      LEG2S2.write(KNEE2RAMP.getValue());
      LEG3S1.write(LEG3RAMP.getValue());
      LEG3S2.write(KNEE3RAMP.getValue());
      LEG4S1.write(LEG4RAMP.getValue());
      LEG4S2.write(KNEE4RAMP.getValue());
    }
  }
}

void allLeg() {
  moveFoward();
  moveBack();
}

void moveL2L3(){
  for (int i = 4; i <= L; i++) { 
    int LEG2TARGET = ANGLE_LEG2S1[i] + err[1][0];
    int KNEE2TARGET = ANGLE_LEG2S2[i] - err[1][1];
    int LEG3TARGET = ANGLE_LEG3S1[i] + err[2][0];
    int KNEE3TARGET = ANGLE_LEG3S2[i] + err[2][1];

    LEG2RAMP.go(LEG2TARGET, SPEED);   
    KNEE2RAMP.go(KNEE2TARGET, SPEED);
    LEG3RAMP.go(LEG3TARGET, SPEED);   
    KNEE3RAMP.go(KNEE3TARGET, SPEED);

    while (!LEG2RAMP.isFinished() && !LEG3RAMP.isFinished() || !KNEE2RAMP.isFinished() && !KNEE3RAMP.isFinished()) {
      LEG2RAMP.update();   
      KNEE2RAMP.update();
      LEG3RAMP.update();   
      KNEE3RAMP.update();

      LEG2S1.write(LEG2RAMP.getValue());
      LEG2S2.write(KNEE2RAMP.getValue());
      LEG3S1.write(LEG3RAMP.getValue());
      LEG3S2.write(KNEE3RAMP.getValue());
    }
  }
}

void moveBackL2L3(){
  for (int i = 9 ; i >= 5; i--) { 
    int LEG2TARGET = ANGLE_LEG2S1[i] + err[1][0];
    int KNEE2TARGET = 74 - err[1][1];
    int LEG3TARGET = ANGLE_LEG3S1[i] + err[2][0];
    int KNEE3TARGET = 46 + err[2][1];
    
    LEG2RAMP.go(LEG2TARGET, SPEED2);   
    KNEE2RAMP.go(KNEE2TARGET, SPEED2);
    LEG3RAMP.go(LEG3TARGET, SPEED2);   
    KNEE3RAMP.go(KNEE3TARGET, SPEED2);

    while (!LEG2RAMP.isFinished() && !LEG3RAMP.isFinished() || !KNEE2RAMP.isFinished() && !KNEE3RAMP.isFinished()) {
      LEG2RAMP.update();   
      KNEE2RAMP.update();
      LEG3RAMP.update();   
      KNEE3RAMP.update();

      LEG2S1.write(LEG2RAMP.getValue());
      LEG2S2.write(KNEE2RAMP.getValue());
      LEG3S1.write(LEG3RAMP.getValue());
      LEG3S2.write(KNEE3RAMP.getValue());
    }
  }
}

void moveL1L4(){
  for (int i = 4; i <= L; i++) { 
    int LEG1TARGET = ANGLE_LEG1S1[i] + err[0][0];
    int KNEE1TARGET = ANGLE_LEG1S2[i] + err[0][1];
    int LEG4TARGET = ANGLE_LEG4S1[i] + err[3][0];
    int KNEE4TARGET = ANGLE_LEG4S2[i] - err[3][1];

    LEG1RAMP.go(LEG1TARGET, SPEED);   
    KNEE1RAMP.go(KNEE1TARGET, SPEED);
    LEG4RAMP.go(LEG4TARGET, SPEED);   
    KNEE4RAMP.go(KNEE4TARGET, SPEED);

   while (!LEG1RAMP.isFinished() && !LEG1RAMP.isFinished() || !KNEE4RAMP.isFinished() && !KNEE4RAMP.isFinished()) {
    LEG1RAMP.update();   
    KNEE1RAMP.update();
    LEG4RAMP.update();   
    KNEE4RAMP.update();

    LEG1S1.write(LEG1RAMP.getValue());
    LEG1S2.write(KNEE1RAMP.getValue());
    LEG4S1.write(LEG4RAMP.getValue());
    LEG4S2.write(KNEE4RAMP.getValue());
    }
  }
}

void moveBackL1L4(){
  for (int i = 9 ; i >= 4; i--) { 
    int LEG1TARGET = ANGLE_LEG1S1[i] + err[0][0];
    int KNEE1TARGET = 46 + err[0][1];
    int LEG4TARGET = ANGLE_LEG4S1[i] + err[3][0];
    int KNEE4TARGET = 74 - err[3][1];

    LEG1RAMP.go(LEG1TARGET, SPEED2);   
    KNEE1RAMP.go(KNEE1TARGET, SPEED2);
    LEG4RAMP.go(LEG4TARGET, SPEED2);   
    KNEE4RAMP.go(KNEE4TARGET, SPEED2);

    while (!LEG1RAMP.isFinished() && !LEG1RAMP.isFinished() || !KNEE4RAMP.isFinished() && !KNEE4RAMP.isFinished()) {
      LEG1RAMP.update();   
      KNEE1RAMP.update();
      LEG4RAMP.update();   
      KNEE4RAMP.update();

      LEG1S1.write(LEG1RAMP.getValue());
      LEG1S2.write(KNEE1RAMP.getValue());
      LEG4S1.write(LEG4RAMP.getValue());
      LEG4S2.write(KNEE4RAMP.getValue());
    }
  }
}

void testWalkCycle(){
  if(Serial.available() > 0){
    int input = Serial.parseInt();
    if (input >= 0 && input <= 20) {
      // LEG2S1.write(ANGLE_LEG2S1[input] + err[1][0]);
      // LEG2S2.write(ANGLE_LEG2S2[input] - err[1][1]);
      LEG4S1.write(ANGLE_LEG4S1[input] + err[3][0]);
      LEG4S2.write(ANGLE_LEG4S2[input] - err[3][1]);

      Serial.print("Index ");
      Serial.print(input);
      Serial.print(": Hip=");
      Serial.print(ANGLE_LEG4S1[input] + err[3][0]);
      Serial.print("  Knee=");
      Serial.println(ANGLE_LEG4S2[input] - err[3][1]);
    }
    while(Serial.available() > 0){
      Serial.read();
    }
  }
}

void testWalkCycleL3(){
  if(Serial.available() > 0){
    int input = Serial.parseInt();
    if (input >= 0 && input <= 20) {
      // LEG2S1.write(ANGLE_LEG2S1[input] + err[1][0]);
      // LEG2S2.write(ANGLE_LEG2S2[input] - err[1][1]);
      LEG3S1.write(ANGLE_LEG3S1[input] + err[2][0]);
      LEG3S2.write(ANGLE_LEG3S2[input] - err[2][1]);

      Serial.print("Index ");
      Serial.print(input);
      Serial.print(": Hip=");
      Serial.print(ANGLE_LEG3S1[input] + err[2][0]);
      Serial.print("  Knee=");
      Serial.println(ANGLE_LEG3S2[input] - err[2][1]);
    }
    while(Serial.available() > 0){
      Serial.read();
    }
  }
}


void settingServo(){
  if(Serial.available() > 0){
    char ch = Serial.read();
    int input = Serial.parseInt();
    if(ch == 'a'){
      LEG2S1.write(input);
      LEG3S1.write(input);
      LEG4S1.write(input);
      LEG1S1.write(input);
      Serial.print("Hip:");
      Serial.println(input);
    }else if(ch == 'b'){
      LEG2S2.write(input);
      LEG1S2.write(input);
      LEG4S2.write(input);
      LEG3S2.write(input);
      Serial.print("Knee:");
      Serial.println(input);
    }
    while(Serial.available() > 0){
      Serial.read();
    }
  }
}  
