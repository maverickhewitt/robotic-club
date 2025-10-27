#include "ServoEasing.hpp"
#include <ESP32Servo.h>
#include <math.h>
#include <Ramp.h>

#define LEG1PIN1 10
#define LEG1PIN2 11

#define LEG2PIN1 12
#define LEG2PIN2 27

#define LEG3PIN1 26
#define LEG3PIN2 25

#define LEG4PIN1 14
#define LEG4PIN2 13

#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_HERTZ 300

#define RAD_TO_DEG 57.295779513

#define APD 20
#define SPEED 300
#define SPEED2 SPEED - 10
#define COND EASE_LINEAR
#define ARRAY_NO 11

float a = 15.0;
float b = 17.0;
float h = 25.0;
float L = 10.0;
float x = 5.00;
float H = 5.0;
float H_LOW = -2.0;

//26<<knee<<96
int err[4][2] = {{8,8}, {8,6}, {16,8}, {9,1}};

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

rampFloat hipRamp;
rampFloat kneeRamp;

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

void findAngleLeft(float i, int stepIndex) {   
  float t = (float)i / L;
  float coorX = L * (t - (sin(2 * PI * t) / (2 * PI)));
  float coorY = (H / 2) * (1 - cos(2 * PI * t));

  float d = sqrt(sq(x - coorX) + sq(h - coorY));

  float knee = acos((sq(a) + sq(b) - sq(d)) / (2 * a * b));
  // float hip = (PI / 2) + asin(coorX / d) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  float hip = (PI/2) + asin(coorX / d) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  // float hip = (PI/2) + atan(coorX / h);

  float hipDeg = hip * RAD_TO_DEG;
  float kneeDeg = knee * RAD_TO_DEG;

  // float convHip =hipDeg * 2 / 3; //no need minus 120 kalau kaki right
  float convHip = 120 - (hipDeg * 2 / 3); //no need minus 120 kalau kaki right
  float convKnee =kneeDeg * 2 / 3;

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
  float convKnee = kneeDeg * 2 / 3;

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

void generateLegPathRight() {
  for (int i = 0; i <= L; i++) {
    findAngleRight(i, i);  
  }
}

void generateLegPathLeft() {
  for (int i = 0; i <= L; i++) {
    findAngleLeft(i, i);  
  }
}

void standingLeg() {
  int midIndex = 6;  

  float hipAngle = ANGLE_LEG1S1[midIndex];
  float kneeAngle = ANGLE_LEG1S2[midIndex];

  LEG1S1.write(hipAngle + err[0][0]);
  LEG1S2.write(kneeAngle - err[0][1]);

  LEG2S1.write(ANGLE_LEG2S1[midIndex] + err[1][0]);
  LEG2S2.write(ANGLE_LEG2S2[midIndex] - err[1][1]);

  LEG3S1.write(ANGLE_LEG3S1[midIndex] + err[2][0]);
  LEG3S2.write(ANGLE_LEG3S2[midIndex] - err[2][1]);

  LEG4S1.write(ANGLE_LEG4S1[midIndex] + err[3][0]);
  LEG4S2.write(ANGLE_LEG4S2[midIndex] - err[3][1]);

  Serial.println("Standing Position");
  Serial.print("HIP: "); Serial.println(hipAngle);
  Serial.print("KNEE: "); Serial.println(kneeAngle);
}

void moveL2(){
  for (int i = 0; i <= L; i++) { 
      hipRamp.go(ANGLE_LEG2S1[i] + err[1][0], SPEED);   
      kneeRamp.go(ANGLE_LEG2S2[i] - err[1][1], SPEED);
      // hipRamp.go(ANGLE_LEG3S1[i] - err[2][0], SPEED);   
      // kneeRamp.go(ANGLE_LEG3S2[i] + err[2][1], SPEED);

      while (!hipRamp.isFinished() || !kneeRamp.isFinished()) {
        hipRamp.update();
        kneeRamp.update();

        LEG2S1.write(hipRamp.getValue());
        LEG2S2.write(kneeRamp.getValue());
        // LEG3S1.write(hipRamp.getValue());
        // LEG3S2.write(kneeRamp.getValue());
    }
  }
}

void moveBackL2(){
  for (int i = L ; i >= 0; i--) { 
      hipRamp.go(ANGLE_LEG2S1[i] + err[1][0], SPEED2);   
      kneeRamp.go(60 - err[1][1], SPEED2);
      // hipRamp.go(ANGLE_LEG3S1[i] - err[2][0], SPEED2);   
      // kneeRamp.go(ANGLE_LEG3S1[i] + err[2][1], SPEED2);

      while (!hipRamp.isFinished() || !kneeRamp.isFinished()) {
        hipRamp.update();
        kneeRamp.update();

        LEG2S1.write(hipRamp.getValue());
        LEG2S2.write(kneeRamp.getValue());
        // LEG3S1.write(hipRamp.getValue());
        // LEG3S2.write(kneeRamp.getValue());
    }
  }
}

void testWalkCycle(){
  if(Serial.available() > 0){
    int input = Serial.parseInt();
    if (input >= 0 && input <= 10) {
      LEG2S1.write(ANGLE_LEG2S1[input] + err[1][0]);
      LEG2S2.write(ANGLE_LEG2S2[input] - err[1][1]);

      Serial.print("Index ");
      Serial.print(input);
      Serial.print(": Hip=");
      Serial.print(ANGLE_LEG2S1[input] + err[1][0]);
      Serial.print("  Knee=");
      Serial.println(ANGLE_LEG2S2[input] - err[1][1]);
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
