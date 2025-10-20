#include "ServoEasing.hpp"
#include <ESP32Servo.h>
#include <math.h>
#include <Ramp.h>

#define LEG1PIN1 12
#define LEG1PIN2 26

#define LEG2PIN1 25
#define LEG2PIN2 28

#define LEG3PIN1 19
#define LEG3PIN2 18

#define LEG4PIN1 17
#define LEG4PIN2 16

#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_HERTZ 300

#define RAD_TO_DEG 57.295779513

#define APD 60
#define COND EASE_LINEAR

float a = 15.0;
float b = 20.0;
float h = 25.0;
float L = 10;
float x = 5.0;
float H = 5.0;
float H_LOW = -2.0;

int err[4][2] = {{4,10}, {0,0}, {0,0}, {0,0}};

float ANGLE_LEG1S1[11];
float ANGLE_LEG1S2[11];

int ANGLE_LEG2S1[4] = {0,0,0,0};
int ANGLE_LEG2S2[4] = {0,0,0,0};

int ANGLE_LEG3S1[4] = {0,0,0,0};
int ANGLE_LEG3S2[4] = {0,0,0,0};

int ANGLE_LEG4S1[4] = {0,0,0,0};
int ANGLE_LEG4S2[4] = {0,0,0,0};

// ------------------------------------

int CLIMB_ANGLE_LEG1S1[4] = {0,0,0,0};
int CLIMB_ANGLE_LEG1S2[4] = {0,0,0,0};

int CLIMB_ANGLE_LEG2S1[4] = {0,0,0,0};
int CLIMB_ANGLE_LEG2S2[4] = {0,0,0,0};

int CLIMB_ANGLE_LEG3S1[4] = {0,0,0,0};
int CLIMB_ANGLE_LEG3S2[4] = {0,0,0,0};

int CLIMB_ANGLE_LEG4S1[4] = {0,0,0,0};
int CLIMB_ANGLE_LEG4S2[4] = {0,0,0,0};

// ------------------------------------

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

void findAngle(float i, int stepIndex) {   
  float t = (float)i / L;
  float coorX = L * (t - (sin(2 * PI * t) / (2 * PI)));
  float coorY = (H / 2) * (1 - cos(2 * PI * t));

  float d = sqrt(sq(x - coorX) + sq(h - coorY));

  float knee = acos((sq(a) + sq(b) - sq(d)) / (2 * a * b));
  float hip;
  
  if (coorX > x) {
    hip = (PI / 2) + atan(coorX / h) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  } else if (coorX < x) {
    hip = (PI / 2) - atan(coorX / h) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  } else {
    hip = (PI / 2) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  }

  float hipDeg = hip * RAD_TO_DEG;
  float kneeDeg = knee * RAD_TO_DEG;

  float convHip = hipDeg * 2 / 3;
  float convKnee = kneeDeg * 2 / 3;

  ANGLE_LEG1S1[stepIndex] = convHip;
  ANGLE_LEG1S2[stepIndex] = convKnee;

  Serial.print("Step ");
  Serial.print(stepIndex);
  Serial.print(" | Hip: ");
  Serial.print(convHip);
  Serial.print(" | Knee: ");
  Serial.println(convKnee);
}

void generateLegPath() {
  for (int i = 0; i <= L; i++) {
    findAngle(i, i);  
  }
}

void standingLeg(){
  LEG1S1.easeTo(ANGLE_LEG1S1[0] + err[0][0]);
  LEG1S2.easeTo(ANGLE_LEG1S2[0] + err[0][1]);

  LEG2S1.easeTo(ANGLE_LEG2S1[0] + err[1][0]);
  LEG2S2.easeTo(ANGLE_LEG2S2[0] + err[1][1]);

  LEG3S1.easeTo(ANGLE_LEG3S1[0] + err[2][0]);
  LEG3S2.easeTo(ANGLE_LEG3S2[0] + err[2][1]);

  LEG4S1.easeTo(ANGLE_LEG4S1[0] + err[3][0]);
  LEG4S2.easeTo(ANGLE_LEG4S2[0] + err[3][1]);
}

void moveL1(){
  for (int i = 0; i <= L; i++) { 
      hipRamp.go(ANGLE_LEG1S1[i], 150);   
      kneeRamp.go(ANGLE_LEG1S2[i], 150);
      // LEG1S1.easeTo(ANGLE_LEG1S1[i] + err[0][0], 20);
      // LEG1S2.easeTo(ANGLE_LEG1S2[i] + err[0][1], 20);

      while (!hipRamp.isFinished() || !kneeRamp.isFinished()) {
        hipRamp.update();
        kneeRamp.update();

        LEG1S1.write(hipRamp.getValue());
        LEG1S2.write(kneeRamp.getValue());
    }
  }
}

void moveL2(){
  LEG2S1.easeTo(ANGLE_LEG2S1[1] + err[1][0]);
  LEG2S2.easeTo(ANGLE_LEG2S2[1] + err[1][1]);

  LEG2S1.easeTo(ANGLE_LEG2S1[2] + err[1][0]);
  LEG2S2.easeTo(ANGLE_LEG2S2[2] + err[1][1]);

  LEG2S1.easeTo(ANGLE_LEG2S1[3] + err[1][0]);
  LEG2S2.easeTo(ANGLE_LEG2S2[3] + err[1][1]);
}

void moveL3(){
  LEG3S1.easeTo(ANGLE_LEG3S1[1] + err[2][0]);
  LEG3S2.easeTo(ANGLE_LEG3S2[1] + err[2][1]);

  LEG3S1.easeTo(ANGLE_LEG3S1[2] + err[2][0]);
  LEG3S2.easeTo(ANGLE_LEG3S2[2] + err[2][1]);

  LEG3S1.easeTo(ANGLE_LEG3S1[3] + err[2][0]);
  LEG3S2.easeTo(ANGLE_LEG3S2[3] + err[2][1]);
}

void moveL4(){
  LEG4S1.easeTo(ANGLE_LEG4S1[1] + err[3][0]);
  LEG4S2.easeTo(ANGLE_LEG4S2[1] + err[3][1]);

  LEG4S1.easeTo(ANGLE_LEG4S1[2] + err[3][0]);
  LEG4S2.easeTo(ANGLE_LEG4S2[2] + err[3][1]);

  LEG4S1.easeTo(ANGLE_LEG4S1[3] + err[3][0]);
  LEG4S2.easeTo(ANGLE_LEG4S2[3] + err[3][1]);
}

void climbL1(){
  LEG1S1.easeTo(CLIMB_ANGLE_LEG1S1[1] + err[0][0]);
  LEG1S2.easeTo(CLIMB_ANGLE_LEG1S2[1] + err[0][1]);

  LEG1S1.easeTo(CLIMB_ANGLE_LEG1S1[2] + err[0][0]);
  LEG1S2.easeTo(CLIMB_ANGLE_LEG1S2[2] + err[0][1]);

  LEG1S1.easeTo(CLIMB_ANGLE_LEG1S1[3] + err[0][0]);
  LEG1S2.easeTo(CLIMB_ANGLE_LEG1S2[3] + err[0][1]);
}

void climbL2(){
  LEG2S1.easeTo(CLIMB_ANGLE_LEG2S1[1] + err[1][0]);
  LEG2S2.easeTo(CLIMB_ANGLE_LEG2S2[1] + err[1][1]);

  LEG2S1.easeTo(CLIMB_ANGLE_LEG2S1[2] + err[1][0]);
  LEG2S2.easeTo(CLIMB_ANGLE_LEG2S2[2] + err[1][1]);

  LEG2S1.easeTo(CLIMB_ANGLE_LEG2S1[3] + err[1][0]);
  LEG2S2.easeTo(CLIMB_ANGLE_LEG2S2[3] + err[1][1]);
}

void climbL3(){
  LEG3S1.easeTo(CLIMB_ANGLE_LEG3S1[1] + err[2][0]);
  LEG3S2.easeTo(CLIMB_ANGLE_LEG3S2[1] + err[2][1]);

  LEG3S1.easeTo(CLIMB_ANGLE_LEG3S1[2] + err[2][0]);
  LEG3S2.easeTo(CLIMB_ANGLE_LEG3S2[2] + err[2][1]);

  LEG3S1.easeTo(CLIMB_ANGLE_LEG3S1[3] + err[2][0]);
  LEG3S2.easeTo(CLIMB_ANGLE_LEG3S2[3] + err[2][1]);
}

void climbL4(){
  LEG4S1.easeTo(CLIMB_ANGLE_LEG4S1[1] + err[3][0]);
  LEG4S2.easeTo(CLIMB_ANGLE_LEG4S2[1] + err[3][1]);

  LEG4S1.easeTo(CLIMB_ANGLE_LEG4S1[2] + err[3][0]);
  LEG4S2.easeTo(CLIMB_ANGLE_LEG4S2[2] + err[3][1]);

  LEG4S1.easeTo(CLIMB_ANGLE_LEG4S1[3] + err[3][0]);
  LEG4S2.easeTo(CLIMB_ANGLE_LEG4S2[3] + err[3][1]);
}

void readyClimb(){
  LEG1S1.easeTo(CLIMB_ANGLE_LEG1S1[0] + err[0][0]);
  LEG1S2.easeTo(CLIMB_ANGLE_LEG1S2[0] + err[0][1]);

  LEG2S1.easeTo(CLIMB_ANGLE_LEG2S1[0] + err[1][0]);
  LEG2S2.easeTo(CLIMB_ANGLE_LEG2S2[0] + err[1][1]);

  LEG3S1.easeTo(CLIMB_ANGLE_LEG3S1[0] + err[2][0]);
  LEG3S2.easeTo(CLIMB_ANGLE_LEG3S2[0] + err[2][1]);

  LEG4S1.easeTo(CLIMB_ANGLE_LEG4S1[0] + err[3][0]);
  LEG4S2.easeTo(CLIMB_ANGLE_LEG4S2[0] + err[3][1]);
}

void climbRamp(){
  climbL1();
  climbL2();
  climbL3();
  climbL4();
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

  void normalLoop(){
    for (int i = L; i >= 0; i--) {
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
    delay(50);
  }
  }

  void loopingManual(){
    static int flag = 0;
    static float i = 0;

    test();

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
  }
