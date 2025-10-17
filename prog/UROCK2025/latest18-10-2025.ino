#include <ESP32Servo.h>
#include "ServoEasing.hpp"

#define LEG1PIN1 12
#define LEG1PIN2 26

#define LEG2PIN1 25
#define LEG2PIN2 28

#define LEG3PIN1 19
#define LEG3PIN2 18

#define LEG4PIN1 17
#define LEG4PIN2 16

#define IR_PIN 27

#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_HERTZ 300

#define RAD_TO_DEG 57.295779513
#define PI 3.1415926535897932384626433832795

#define APD 60

int err[4][2] = {{4,10}, {0,0}, {0,0}, {0,0}};

int sensor;

float a = 15.0;
float b = 20.0;
float dConst = 25.0;

float h = 25.0;
float L = 10;
float H = 5.0;
float H_LOW = -2.0;
float x = 5.0;

ServoEasing LEG1S1;
ServoEasing LEG1S2;

Servo LEG2S1;
Servo LEG2S2;

Servo LEG3S1;
Servo LEG3S2;

Servo LEG4S1;
Servo LEG4S2;

void setup() {
  Serial.begin(115200);
  setupLeg();
  // standingLeg();
  // delay(1000);
}

void test(){
  LEG1S1.easeTo(11 + err[0][0], APD); 
  LEG1S2.easeTo(46 + err[0][1], APD);
  updateAndWaitForAllServosToStop();

  LEG1S1.easeTo(14 + err[0][0], APD); 
  LEG1S2.easeTo(45 + err[0][1], APD);
  updateAndWaitForAllServosToStop();

  LEG1S1.easeTo(26 + err[0][0], APD); 
  LEG1S2.easeTo(46 + err[0][1], APD);
  updateAndWaitForAllServosToStop();

  LEG1S1.easeTo(31 + err[0][0], APD); 
  LEG1S2.easeTo(50 + err[0][1], APD);
  updateAndWaitForAllServosToStop();

  LEG1S1.easeTo(35 + err[0][0], APD); 
  LEG1S2.easeTo(55 + err[0][1], APD);
  updateAndWaitForAllServosToStop();

  LEG1S1.easeTo(39 + err[0][0], APD); 
  LEG1S2.easeTo(60 + err[0][1], APD);
  updateAndWaitForAllServosToStop();
}

void IKSTATIC(){
  if(Serial.available() > 0){
    float input = Serial.parseFloat(); 
    float t = input;
    float coorX = L * (t - (sin(2 * PI * t) / (2 * PI)));
    float coorY = (H / 2) * (1 - cos(2 * PI * t));
    // float coorY = H / 2;

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

    float hipDeg = hip * RAD_TO_DEG;
    float kneeDeg = knee * RAD_TO_DEG;
    float convHip = hipDeg * 2/3;
    float convKnee;
    convKnee = kneeDeg * 2/3;

    Serial.print(" | CoorX: ");
    Serial.print(coorX);
    Serial.print(" | CoorY: ");
    Serial.print(coorY);
    Serial.print(" | d: ");
    Serial.print(d);
    Serial.print(" | Hip: ");
    Serial.print(convHip + err[0][0]);
    Serial.print(" | Knee: ");
    Serial.println(convKnee + err[0][1]);

    LEG1S2.write(convKnee + err[0][1]);
    delay(20);
    LEG1S1.write(convHip + err[0][0]);

    while(Serial.available() > 0){
      Serial.read();
    }
  }
}

void IK(float i){
  float t = (float)i / L;
    float coorX = L * (t - (sin(2 * PI * t) / (2 * PI)));
    float coorY = (H / 2) * (1 - cos(2 * PI * t));
    // float coorY = H / 2;

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

    float hipDeg = hip * RAD_TO_DEG;
    float kneeDeg = knee * RAD_TO_DEG;
    float convHip = hipDeg * 2/3;
    float convKnee;
    convKnee = kneeDeg * 2/3;

    Serial.print(" | CoorX: ");
    Serial.print(coorX);
    Serial.print(" | CoorY: ");
    Serial.print(coorY);
    Serial.print(" | d: ");
    Serial.print(d);
    Serial.print(" | Hip: ");
    Serial.print(convHip);
    Serial.print(" | Knee: ");
    Serial.println(convKnee);

    // LEG1S1.write(convHip + err[0][0]);
    // LEG1S2.write(convKnee + err[0][1]);
    LEG1S1.easeTo(convHip + err[0][0], 40); 
    LEG1S2.easeTo(convKnee + err[0][1], 40);
}

void IK2(float i){
  float t = (float)i / L;
  float coorX = L * (t - (sin(2 * PI * t) / (2 * PI)));
  float coorY = (H_LOW / 2) * (1 - cos(2 * PI * t));
  // float coorY = H / 2;

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

   float hipDeg = hip * RAD_TO_DEG;
   float kneeDeg = knee * RAD_TO_DEG;
   float convHip = hipDeg * 2/3;
   float convKnee;
   convKnee = kneeDeg * 2/3;

   Serial.print(" | CoorX: ");
   Serial.print(coorX);
   Serial.print(" | CoorY: ");
   Serial.print(coorY);
   Serial.print(" | d: ");
   Serial.print(d);
   Serial.print(" | Hip: ");
   Serial.print(convHip);
   Serial.print(" | Knee: ");
   Serial.println(convKnee);

   LEG1S1.write(convHip + err[0][0]);
   LEG1S2.write(convKnee + err[0][1]);
  //  delay(30);
}

void loop() {
  static int flag = 0;
  static float i = 0;

  test();

  // bool isWalking = true;

  // if(flag == 0){

  // }else if(flag == 1){

  // }
  // else if(flag == 2)


  // while(isWalking){
  //   for(i = 0; i <= L; i++){
  //     IK(i);
  //     delay(20);
  //     if(i >= L){
  //       for(i = L; i >= 0; i-- ){
  //         IK2(i);
  //         delay(20);
  //       }
  //     }
  //   }
  // }

  // if (flag == 0) {
  //   i += 1;
  //   IK(i);
  //   if (i >= L) {
  //     // flag = 1;
  //     i = 0;
  //   }
  // } 
  // else if (flag == 1) {
  //   i -= 1;
  //   IK2(i);
  //   if (i <= 0) {
  //     flag = 0; 
  //   }
  // }
  // delay(50);
  // for (int i = L; i >= 0; i--) {
  //   float t = (float)i / L;
  //   float coorX = L * (t - (sin(2 * PI * t) / (2 * PI)));
  //   float coorY = (H_LOW / 2) * (1 - cos(2 * PI * t));
  //   // float coorY = H / 2;

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

  //   float hipDeg = hip * RAD_TO_DEG;
  //   float kneeDeg = knee * RAD_TO_DEG;
  //   float convHip = hipDeg * 2/3;
  //   float convKnee;
  //   convKnee = kneeDeg * 2/3;

  //   Serial.print(" | CoorX: ");
  //   Serial.print(coorX);
  //   Serial.print(" | CoorY: ");
  //   Serial.print(coorY);
  //   Serial.print(" | d: ");
  //   Serial.print(d);
  //   Serial.print(" | Hip: ");
  //   Serial.print(convHip);
  //   Serial.print(" | Knee: ");
  //   Serial.println(convKnee);

  //   LEG1S1.write(convHip + err[0][0]);
  //   LEG1S2.write(convKnee + err[0][1]);
  //   delay(50);
  // }
  
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
  LEG1S1.setEasingType(EASE_CUBIC_IN_OUT);
  LEG1S2.setEasingType(EASE_CUBIC_IN_OUT);
  LEG1S2.attach(LEG1PIN2, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  LEG2S1.attach(LEG2PIN1, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  LEG2S2.attach(LEG2PIN2, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  LEG3S1.attach(LEG3PIN1, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  LEG3S2.attach(LEG3PIN2, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  LEG4S1.attach(LEG4PIN1, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  LEG4S2.attach(LEG4PIN2, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

void IKMAIN(){
  for (int i = 0; i <= L; i++) {
    float t = (float)i / L;
    float coorX = L * (t - (sin(2 * PI * t) / (2 * PI)));
    float coorY = (H / 2) * (1 - cos(2 * PI * t));
    // float coorY = H / 2;

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

    float hipDeg = hip * RAD_TO_DEG;
    float kneeDeg = knee * RAD_TO_DEG;
    float convHip = hipDeg * 2/3;
    float convKnee;
    convKnee = kneeDeg * 2/3;

    Serial.print(" | CoorX: ");
    Serial.print(coorX);
    Serial.print(" | CoorY: ");
    Serial.print(coorY);
    Serial.print(" | d: ");
    Serial.print(d);
    Serial.print(" | Hip: ");
    Serial.print(convHip);
    Serial.print(" | Knee: ");
    Serial.println(convKnee);

    LEG1S1.write(convHip + err[0][0]);
    LEG1S2.write(convKnee + err[0][1]);
    delay(50);
    
  }
}
