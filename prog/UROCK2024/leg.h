#ifndef LEG_H
#define LEG_H

#include <ESP32Servo.h>
#include "ServoEasing.hpp"

// int err[4][2] = {{5, 7}, {3, 3}, {0, 14}, {9, 12}}; err lama
int err[4][2] = {{6, 9}, {2, 10}, {3, 4}, {4, 0}}; //err baru
//int invKinA[10] = {29, 16, 7, 3, 6, 15, 21, 25, 28, 29};
int invKinA[3] = {90, 94, 117};  
//int invKinB[10] = {29, 27, 30, 36, 45, 57, 54, 50, 44, 37};
int invKinB[3] = {26, 43, 40};

//leg 1 - OKE SUDA --> angle leg 4
int invKinL1A[4] = {80, 70, 94, 67};//stand-raise-foward-backward  
int invKinL1B[4] = {40, 46, 46, 32};

//leg 2 -- angle leg 3
int invKinL2A[4] = {80,73,73,88};  //stand-raise-foward-backward
int invKinL2B[4] = {40,49,24,49};

//leg 3 -- angle leg 2 
int invKinL3A[4] = {80, 70, 66, 93};  //stand-raise-foward-backward
int invKinL3B[4] = {40, 46, 31, 44};

//leg 4 - OKE SUDA -- angle leg 1
int invKinL4A[4] = {80, 73,88,75};  //stand-raise-foward-backward
int invKinL4B[4] = {40, 49 ,53, 26}; //75,73,88 26,49,53


int staticArrayA[2] = {90, 75};
int staticArrayB[2] = {30, 45};

// ServoEasing LEG1S1;
// ServoEasing LEG1S2;

// ServoEasing LEG2S1;
// ServoEasing LEG2S2;

// ServoEasing LEG3S1;
// ServoEasing LEG3S2;

// ServoEasing LEG4S1;
// ServoEasing LEG4S2;

//--------------------------------

Servo LEG1S1;
Servo LEG1S2;

Servo LEG2S1;
Servo LEG2S2;

Servo LEG3S1;
Servo LEG3S2;

Servo LEG4S1;
Servo LEG4S2;

#define LEG1PIN1 32
#define LEG1PIN2 33

#define LEG2PIN1 25
#define LEG2PIN2 26

#define LEG3PIN1 19
#define LEG3PIN2 18

#define LEG4PIN1 17
#define LEG4PIN2 16

#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_HERTZ 300

void setupLeg() {
  //ledcAttachChannel(LEG1PIN1, SERVO_HERTZ, 8, 0);
  //ledcAttachChannel(LEG1PIN2, SERVO_HERTZ, 8, 1);

  //  LEG1S1.setEasingType(EASE_CUBIC_IN_OUT);
  //  LEG1S2.setEasingType(EASE_CUBIC_IN_OUT);

  //  LEG2S1.setEasingType(EASE_CUBIC_IN_OUT);
  //  LEG2S2.setEasingType(EASE_CUBIC_IN_OUT);

  //  LEG3S1.setEasingType(EASE_CUBIC_IN_OUT);
  //  LEG3S2.setEasingType(EASE_CUBIC_IN_OUT);

  //  LEG4S1.setEasingType(EASE_CUBIC_IN_OUT);
  //  LEG4S2.setEasingType(EASE_CUBIC_IN_OUT);

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

void readyLeg() {
  LEG1S1.write(75 + err[0][0]);
  LEG1S2.write(45 + err[0][1]);

  LEG2S1.write((75 * 270 / 180) + err[1][0]);
  LEG2S2.write((45 * 270 / 180)+ err[1][1]);

  LEG3S1.write(75 + err[2][0]);
  LEG3S2.write(45 + err[2][1]);

  LEG4S1.write(75 + err[3][0]);
  LEG4S2.write(45 + err[3][1]);
}

void standLeg() {
  LEG1S1.write(90 + err[0][0]);
  LEG1S2.write(30 + err[0][1]);

  LEG2S1.write(135 + err[1][0]);
  LEG2S2.write(45 + err[1][1]);

  LEG3S1.write(90 + err[2][0]);
  LEG3S2.write(30 + err[2][1]);

  LEG4S1.write(90 + err[3][0]);
  LEG4S2.write(30 + err[3][1]);
}

void staticLeg(int index) {
  LEG1S1.write(staticArrayA[index] + err[0][0]);
  LEG1S2.write(staticArrayB[index] + err[0][1]);

  LEG2S1.write((staticArrayA[index] * 270 / 180) + err[1][0]);
  LEG2S2.write((staticArrayB[index] * 270 / 180) + err[1][1]);

  LEG3S1.write(staticArrayA[index] + err[2][0]);
  LEG3S2.write(staticArrayB[index] + err[2][1]);

  LEG4S1.write(staticArrayA[1 - index] + err[3][0]);
  LEG4S2.write(staticArrayB[1 - index] + err[3][1]);
}

void L1Move(int index) {
  
  LEG1S1.write(invKinA[index] + err[0][0]);
  LEG1S2.write(invKinB[index] + err[0][1]);
}

void L2Move(int index) {
  LEG2S1.write((invKinA[2 - index] * 270 / 180) + err[1][0]);
  LEG2S2.write((invKinB[2 - index] * 270 / 180) + err[1][1]);
}

void L3Move(int index) {
  LEG3S1.write(invKinA[2 - index] + err[2][0]);
  LEG3S2.write(invKinB[2 - index] + err[2][1]);
}

void L4Move(int index) {
  LEG4S1.write(invKinA[index] + err[3][0]);
  LEG4S2.write(invKinB[index] + err[3][1]);
}

//new movement
void newL1Move(int index) {
  
  LEG1S1.write(invKinL1A[index] + err[0][0]);
  LEG1S2.write(invKinL1B[index] + err[0][1]);
}

void newL2Move(int index) {
  LEG2S1.write((invKinL2A[index] * 270 / 180) + err[1][0]);
  LEG2S2.write((invKinL2B[index] * 270 / 180) + err[1][1]);
}

void newL3Move(int index) {
  LEG3S1.write(invKinL3A[index] + err[2][0]);
  LEG3S2.write(invKinL3B[index] + err[2][1]);
}

void newL4Move(int index) {
  LEG4S1.write(invKinL4A[index] + err[3][0]);
  LEG4S2.write(invKinL4B[index] + err[3][1]);
}

#endif
