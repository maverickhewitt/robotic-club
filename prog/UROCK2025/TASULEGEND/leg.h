#include <math.h>
#include <ESP32Servo.h>
#include <Ramp.h>

#define SILPIN1 12
#define SIRPIN2 13

#define SJLPIN1 25
#define SJRPIN2 26

#define SKLPIN1 33
#define SKRPIN2 32

#define SLLPIN1 14
#define SLRPIN2 27

#define SPEED 60

float pi = M_PI;  // M_PI is defined as 3.14159265358979323846
float x;
float y;

const float L1 = 130.0;
const float L2 = 195.0;
const float DistanceBetweenServo = 35;
const float ground_offset = 250.0;

const float step_length = 140;
const float step_height = 60;

float delta_t = 0.2;  //the best fast 0.4 0.3 slow but clear

const int maxSteps = 12;
// const int maxSteps = (int)(2 * (1.0 / delta_t) + 2);

float I_LA[maxSteps];
float I_RA[maxSteps];

float J_LA[maxSteps];
float J_RA[maxSteps];

float K_LA[maxSteps];
float K_RA[maxSteps];

float L_LA[maxSteps];
float L_RA[maxSteps];

// float x0 = -step_length/2;
// float yZero =  ground_offset;

// float xOne = 0;
// float yOne = ground_offset + 2 * step_height;

// float xTwo = step_length/2;
// float yTwo = ground_offset;

float xZero = 0;  //WHY
float yZero = ground_offset;

float xOne = step_length / 2 + 30;
float yOne = ground_offset - 2 * step_height;

float xTwo = step_length + 20;
float yTwo = ground_offset;

Servo SIL;
Servo SIR;

Servo SJL;
Servo SJR;

Servo SKL;
Servo SKR;

Servo SLL;
Servo SLR;

rampFloat LSRamp;
rampFloat RSRamp;
rampFloat LSRamp2;
rampFloat RSRamp2;

void setupServo() {
  SIL.attach(SILPIN1);
  SIR.attach(SIRPIN2);

  SJL.attach(SJLPIN1);
  SJR.attach(SJRPIN2);

  SKL.attach(SKLPIN1);
  SKR.attach(SKRPIN2);

  SLL.attach(SLLPIN1);
  SLR.attach(SLRPIN2);
}

struct ServoAngles {
  float left;
  float right;
};

// k j
// l i
ServoAngles IK_ThetaAngle(float x, float y) {

  ServoAngles result;

  float L3 = sqrt(x * x + y * y);
  float arg1 = (L1 * L1 + L3 * L3 - L2 * L2) / (2 * L1 * L3);
  if (arg1 < -1.0f) arg1 = -1.0f;
  if (arg1 > 1.0f) arg1 = 1.0f;
  float Gamma1 = acos(arg1);
  float Beta1 = atan2(y, x);
  float Theta1 = 180.0 - Gamma1 * 180.0 / pi - Beta1 * 180.0 / pi;

  float xTwo = x - DistanceBetweenServo;
  float L4 = sqrt(xTwo * xTwo + y * y);
  float arg2 = (L1 * L1 + L4 * L4 - L2 * L2) / (2 * L1 * L4);
  if (arg2 < -1.0f) arg2 = -1.0f;
  if (arg2 > 1.0f) arg2 = 1.0f;
  float Gamma2 = acos(arg2);
  float Beta2 = atan2(y, xTwo);
  float Theta2 = (Beta2 - Gamma2) * 180.0 / pi;

  // Serial.print("Theta1: ");
  // Serial.println(Theta1);

  // Serial.print("Theta2: ");
  // Serial.println(Theta2);

  result.left = (Theta1 + 90) * 2.0 / 3.0;
  result.right = (90 - Theta2) * 2.0 / 3.0;
  return result;
}

ServoAngles IK_ThetaAngle180(float x, float y) {

  ServoAngles result;

  float L3 = sqrt(x * x + y * y);
  float arg1 = (L1 * L1 + L3 * L3 - L2 * L2) / (2 * L1 * L3);
  if (arg1 < -1.0f) arg1 = -1.0f;
  if (arg1 > 1.0f) arg1 = 1.0f;
  float Gamma1 = acos(arg1);
  float Beta1 = atan2(y, x);
  float Theta1 = 180.0 - Gamma1 * 180.0 / pi - Beta1 * 180.0 / pi;

  float xTwo = x - DistanceBetweenServo;
  float L4 = sqrt(xTwo * xTwo + y * y);
  float arg2 = (L1 * L1 + L4 * L4 - L2 * L2) / (2 * L1 * L4);
  if (arg2 < -1.0f) arg2 = -1.0f;
  if (arg2 > 1.0f) arg2 = 1.0f;
  float Gamma2 = acos(arg2);
  float Beta2 = atan2(y, xTwo);
  float Theta2 = (Beta2 - Gamma2) * 180.0 / pi;

  Serial.print("Theta1: ");
  Serial.println(Theta1);

  Serial.print("Theta2: ");
  Serial.println(Theta2);

  result.left = (Theta1 + 90);
  result.right = (90 - Theta2);
  return result;
}

float bezierQuadratic(float p0, float p1, float p2, float t) {  //function curve
  return pow(1 - t, 2) * p0 + 2 * (1 - t) * t * p1 + pow(t, 2) * p2;
}

void leg_k() {
  float t;
  int index = 0;
  //ikd how but ts works RIGHT SIDE

  // 🟢 Air phase (foot up and forward)
  for (t = 0; t <= 1; t += delta_t) {
    x = bezierQuadratic(xTwo, xOne, xZero, t);
    y = bezierQuadratic(yTwo, yOne, yZero, t);

    // Serial.print("Air phase - x: ");
    // Serial.print(x);
    // Serial.print(", y: ");
    // Serial.println(y);

    ServoAngles angles = IK_ThetaAngle(x, y);
    float LeftServoAngle = angles.left;
    float RightServoAngle = angles.right;
    K_LA[index] = angles.left;
    K_RA[index] = angles.right;
    index++;

    // Serial.print("LeftServoAngle: ");
    // Serial.println(LeftServoAngle);

    // Serial.print("RightServoAngle ");
    // Serial.println(RightServoAngle);

    // Servo_iLeft.write(LeftServoAngle);
    // Servo_iRight.write(RightServoAngle);

    //     IK_ThetaAngle(x, y);
  }

  // 🔵 Ground phase (foot back and flat)
  for (t = 0; t <= 1; t += delta_t) {
    // move back linearly along the ground
    x = xZero + t * step_length;
    y = ground_offset;

    // Serial.print("Ground phase - x: ");
    // Serial.print(x);
    // Serial.print(", y: ");
    // Serial.println(y);
    ServoAngles angles = IK_ThetaAngle(x, y);
    float LeftServoAngle = angles.left;
    float RightServoAngle = angles.right;

    K_LA[index] = angles.left;
    K_RA[index] = angles.right;
    index++;

    // Serial.print("LeftServoAngle: ");
    // Serial.println(LeftServoAngle);

    // Serial.print("RightServoAngle ");
    // Serial.println(RightServoAngle);

    // Servo_iLeft.write(LeftServoAngle);
    // Servo_iRight.write(RightServoAngle);

    //     IK_ThetaAngle(x, y);

    //  delay(1000);
  }
  Serial.println("Stored Servo Angles");
  for (int i = 0; i < index; i++) {
    Serial.print("Step ");
    Serial.print(i);
    Serial.print(" | Left: ");
    Serial.print(K_LA[i]);
    Serial.print(" | Right: ");
    Serial.println(K_RA[i]);
  }

  Serial.print("Total steps stored: ");
  Serial.println(index);
}

void leg_i() {  //and j
  float t;
  int index = 0;

  //ikd how but ts works RIGHT SIDE

  // 🟢 Air phase (foot up and forward)
  for (t = 0; t <= 1; t += delta_t) {
    x = bezierQuadratic(xZero, xOne, xTwo, t);
    y = bezierQuadratic(yZero, yOne, yTwo, t);

    // Serial.print("Air phase - x: ");
    // Serial.print(x);
    // Serial.print(", y: ");
    // Serial.println(y);

    ServoAngles angles = IK_ThetaAngle(x, y);
    float LeftServoAngle = angles.left;
    float RightServoAngle = angles.right;
    I_LA[index] = angles.left;
    I_RA[index] = angles.right;
    J_LA[index] = angles.left;
    J_RA[index] = angles.right;
    index++;


    // Serial.print("LeftServoAngle: ");
    // Serial.println(LeftServoAngle);

    // Serial.print("RightServoAngle ");
    // Serial.println(RightServoAngle);

    // Servo_iLeft.write(LeftServoAngle);
    // Servo_iRight.write(RightServoAngle);

    //     IK_ThetaAngle(x, y);

    //  delay(1000);
  }

  // 🔵 Ground phase (foot back and flat)
  for (t = 0; t <= 1; t += delta_t) {
    // move back linearly along the ground
    x = xTwo - t * step_length;
    y = ground_offset;

    // Serial.print("Ground phase - x: ");
    // Serial.print(x);
    // Serial.print(", y: ");
    // Serial.println(y);
    ServoAngles angles = IK_ThetaAngle(x, y);
    float LeftServoAngle = angles.left;
    float RightServoAngle = angles.right;
    I_LA[index] = angles.left;
    I_RA[index] = angles.right;
    J_LA[index] = angles.left;
    J_RA[index] = angles.right;
    index++;

    // Serial.print("LeftServoAngle: ");
    // Serial.println(LeftServoAngle);

    // Serial.print("RightServoAngle ");
    // Serial.println(RightServoAngle);

    // Servo_iLeft.write(LeftServoAngle);
    // Servo_iRight.write(RightServoAngle);

    //     IK_ThetaAngle(x, y);

    //  delay(1000);
  }
}

void leg_l() {

  float t;
  int index = 0;
  //ikd how but ts works RIGHT SIDE

  // 🟢 Air phase (foot up and forward)
  for (t = 0; t <= 1; t += delta_t) {
    x = bezierQuadratic(xTwo, xOne, xZero, t);
    y = bezierQuadratic(yTwo, yOne, yZero, t);

    Serial.print("Air phase - x: ");
    Serial.print(x);
    Serial.print(", y: ");
    Serial.println(y);

    ServoAngles angles = IK_ThetaAngle180(x, y);  //mark
    float LeftServoAngle = angles.left;
    float RightServoAngle = angles.right;
    L_LA[index] = angles.left;
    L_RA[index] = angles.right;
    index++;


    // Serial.print("LeftServoAngle: ");
    // Serial.println(LeftServoAngle);

    // Serial.print("RightServoAngle ");
    // Serial.println(RightServoAngle);

    // Servo_iLeft.write(LeftServoAngle);
    // Servo_iRight.write(RightServoAngle);

    //     IK_ThetaAngle(x, y);

    //  delay(1000);
  }

  // 🔵 Ground phase (foot back and flat)
  for (t = 0; t <= 1; t += delta_t) {
    // move back linearly along the ground
    x = xZero + t * step_length;
    y = ground_offset;

    Serial.print("Ground phase - x: ");
    Serial.print(x);
    Serial.print(", y: ");
    Serial.println(y);
    ServoAngles angles = IK_ThetaAngle(x, y);
    float LeftServoAngle = angles.left;
    float RightServoAngle = angles.right;
    L_LA[index] = angles.left;
    L_RA[index] = angles.right;
    index++;

    // Serial.print("LeftServoAngle: ");
    // Serial.println(LeftServoAngle);

    // Serial.print("RightServoAngle ");
    // Serial.println(RightServoAngle);

    // Servo_iLeft.write(LeftServoAngle);
    // Servo_iRight.write(RightServoAngle);

    //     IK_ThetaAngle(x, y);

    //  delay(1000);
  }
}

void move_IK() {
  LSRamp.go(SIL.read(), SPEED);
  RSRamp.go(SIR.read(), SPEED);
  LSRamp2.go(SKL.read(), SPEED);
  RSRamp2.go(SKR.read(), SPEED);

  for (int i = 0; i < maxSteps; i++) {
    float leftTarget_I = I_LA[i];
    float rightTarget_I = I_RA[i];
    float leftTarget_K = K_LA[i];
    float rightTarget_K = K_RA[i];

    LSRamp.go(leftTarget_I, SPEED);
    RSRamp.go(rightTarget_I, SPEED);
    LSRamp2.go(leftTarget_K, SPEED);
    RSRamp2.go(rightTarget_K, SPEED);

    while (
      !LSRamp.isFinished() || !RSRamp.isFinished() || !LSRamp2.isFinished() || !RSRamp2.isFinished()) {

      LSRamp.update();
      RSRamp.update();
      LSRamp2.update();
      RSRamp2.update();

      SIL.write(LSRamp.getValue());
      SIR.write(RSRamp.getValue());
      SKL.write(LSRamp2.getValue());
      SKR.write(RSRamp2.getValue());

      // delay(10);
    }
  }
}

void move_JL() {
  LSRamp.go(SJL.read(), SPEED);
  RSRamp.go(SJR.read(), SPEED);
  LSRamp2.go(SLL.read(), SPEED);
  RSRamp2.go(SLR.read(), SPEED);

  for (int i = 0; i < maxSteps; i++) {
    float leftTarget_J = J_LA[i];
    float rightTarget_J = J_RA[i];
    float leftTarget_L = L_LA[i];
    float rightTarget_L = L_RA[i];

    LSRamp.go(leftTarget_J, SPEED);
    RSRamp.go(rightTarget_J, SPEED);
    LSRamp2.go(leftTarget_L, SPEED);
    RSRamp2.go(rightTarget_L, SPEED);

    while (
      !LSRamp.isFinished() || !RSRamp.isFinished() || !LSRamp2.isFinished() || !RSRamp2.isFinished()) {

      LSRamp.update();
      RSRamp.update();
      LSRamp2.update();
      RSRamp2.update();

      SJL.write(LSRamp.getValue());
      SJR.write(RSRamp.getValue());
      SLL.write(LSRamp2.getValue());
      SLR.write(RSRamp2.getValue());

      // delay(10);
    }
  }
}


void move_I() {
  LSRamp.go(SIL.read(), SPEED);
  RSRamp.go(SIR.read(), SPEED);

  for (int i = 0; i < maxSteps; i++) {
    float leftTarget = I_LA[i];
    float rightTarget = I_RA[i];

    LSRamp.go(leftTarget, SPEED);
    RSRamp.go(rightTarget, SPEED);

    while (!LSRamp.isFinished() || !RSRamp.isFinished()) {
      LSRamp.update();
      RSRamp.update();

      SIL.write(LSRamp.getValue());
      SIR.write(RSRamp.getValue());

      // delay(10);
    }
  }
}

void move_K() {
  LSRamp.go(SKL.read(), SPEED);
  RSRamp.go(SKR.read(), SPEED);

  for (int i = 0; i < maxSteps; i++) {
    float leftTarget = K_LA[i];
    float rightTarget = K_RA[i];

    LSRamp.go(leftTarget, SPEED);
    RSRamp.go(rightTarget, SPEED);

    while (!LSRamp.isFinished() || !RSRamp.isFinished()) {
      LSRamp.update();
      RSRamp.update();
      SKL.write(LSRamp.getValue());
      SKR.write(RSRamp.getValue());
      // delay(10);
    }
  }
}

void settingServo() {
  if (Serial.available() > 0) {
    char ch = Serial.read();        // read the command letter
    int angle = Serial.parseInt();  // read the number after it

    switch (ch) {
      case 'q':  // Left inner servo
        SIL.write(angle);
        Serial.print("SIL set to: ");
        Serial.println(angle);
        break;

      case 'w':  // Left joint servo
        SJL.write(angle);
        Serial.print("SJL set to: ");
        Serial.println(angle);
        break;

      case 'e':  // Left knee servo
        SKL.write(angle);
        Serial.print("SKL set to: ");
        Serial.println(angle);
        break;

      case 'r':  // Left lower servo
        SLL.write(angle);
        Serial.print("SLL set to: ");
        Serial.println(angle);
        break;

      case 't':  // Right inner servo
        SIR.write(angle);
        Serial.print("SIR set to: ");
        Serial.println(angle);
        break;

      case 'y':  // Right joint servo
        SJR.write(angle);
        Serial.print("SJR set to: ");
        Serial.println(angle);
        break;

      case 'u':  // Right knee servo
        SKR.write(angle);
        Serial.print("SKR set to: ");
        Serial.println(angle);
        break;

      case 'i':  // Right lower servo
        SLR.write(angle);
        Serial.print("SLR set to: ");
        Serial.println(angle);
        break;

      default:
        Serial.println("Invalid command! Use q,w,e,r,t,y,u,i + angle");
        break;
    }

    // Clear any extra input in Serial buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}