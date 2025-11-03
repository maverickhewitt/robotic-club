#include <math.h>
#include <ESP32Servo.h>
#include <Ramp.h>

#define SIL_PIN 13
#define SIR_PIN 14

#define SJL_PIN 27
#define SJR_PIN 26

#define SKL_PIN 25
#define SKR_PIN 33

#define SLL_PIN 32
#define SLR_PIN 21

#define SPEED 50
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_HERTZ 300
#define RESOLUTION 8
#define INIT 88
#define INIT2 31
#define INIT180 133
#define INIT1802 47

int err[4][2] = {{3,4}, {3,6}, {1,6}, {4,8}};

float pi = M_PI;  // M_PI is defined as 3.14159265358979323846
float x;
float y;

const float L1 = 130.0;
const float L2 = 195.0;
const float DistanceBetweenServo = 35;
const float ground_offset = 250.0;

const float step_length = 90;
const float step_height = 40;

float delta_t = 0.2;  //the best fast 0.4 0.3 slow but clear

const int maxSteps = 12;
// const int maxSteps = (int)(2 * (1.0 / delta_t) + 2);

float I_LA[maxSteps];
float I_RA[maxSteps];

float BACK_I_LA[maxSteps];
float BACK_I_RA[maxSteps];

float J_LA[maxSteps];
float J_RA[maxSteps];

float BACK_J_LA[maxSteps];
float BACK_J_RA[maxSteps];

float K_LA[maxSteps];
float K_RA[maxSteps];

float BACK_K_LA[maxSteps];
float BACK_K_RA[maxSteps];

float L_LA[maxSteps];
float L_RA[maxSteps];

float BACK_L_LA[maxSteps];
float BACK_L_RA[maxSteps];

// float x0 = -step_length/2;
// float yZero =  ground_offset;

// float xOne = 0;
// float yOne = ground_offset + 2 * step_height;

// float xTwo = step_length/2;
// float yTwo = ground_offset;

float xZero = -step_length / 2 - (34.8/2);  //WHY
float yZero = ground_offset;

float xOne = (34.8/2);
float yOne = ground_offset - 2 * step_height;

float xTwo = step_length / 2 + (34.8/2);
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

rampFloat LSRamp3;
rampFloat RSRamp3;

rampFloat LSRamp4;
rampFloat RSRamp4;

void setupServo() {
  // ledcAttachChannel(SIL_PIN, SERVO_HERTZ, RESOLUTION,1);
  // ledcAttachChannel(SIR_PIN, SERVO_HERTZ, RESOLUTION,2);

  // ledcAttachChannel(SJL_PIN, SERVO_HERTZ, RESOLUTION,3);
  // ledcAttachChannel(SJR_PIN, SERVO_HERTZ, RESOLUTION,4);

  // ledcAttachChannel(SKL_PIN, SERVO_HERTZ, RESOLUTION,5);
  // ledcAttachChannel(SKR_PIN, SERVO_HERTZ, RESOLUTION,6);

  // ledcAttachChannel(SLL_PIN, SERVO_HERTZ, RESOLUTION,7);
  // ledcAttachChannel(SLR_PIN, SERVO_HERTZ, RESOLUTION,8);

  SIL.setPeriodHertz(SERVO_HERTZ);
  SIR.setPeriodHertz(SERVO_HERTZ);
  
  SJL.setPeriodHertz(SERVO_HERTZ);
  SJR.setPeriodHertz(SERVO_HERTZ);

  SKL.setPeriodHertz(SERVO_HERTZ);
  SKR.setPeriodHertz(SERVO_HERTZ);

  SLL.setPeriodHertz(SERVO_HERTZ);
  SLR.setPeriodHertz(SERVO_HERTZ);

  SIL.attach(SIL_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  SIR.attach(SIR_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  SJL.attach(SJL_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  SJR.attach(SJR_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  SKL.attach(SKL_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  SKR.attach(SKR_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  SLL.attach(SLL_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  SLR.attach(SLR_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
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

  // Serial.print("left: ");
  // Serial.println(result.left);
  // Serial.print("right: ");
  // Serial.println(result.right);
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

  // Serial.print("Theta1: ");
  // Serial.println(Theta1);

  // Serial.print("Theta2: ");
  // Serial.println(Theta2);

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
    Serial.print("INDEX: ");
    Serial.println(index);
    Serial.print("SKL");
    Serial.println(K_LA[index]);
    Serial.print("SKR");
    Serial.println(K_RA[index]);
    index++;

    // Serial.print("LeftServoAngle: ");
    // Serial.println(LeftServoAngle);

    // Serial.print("RightServoAngle ");
    // Serial.println(RightServoAngle);

    // Servo_iLeft.write(LeftServoAngle);
    // Servo_iRight.write(RightServoAngle);

        // IK_ThetaAngle(x, y);

    
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

    BACK_K_LA[index] = angles.left;
    BACK_K_RA[index] = angles.right;
    Serial.print("INDEX: ");
    Serial.println(index);
    Serial.print("SKL");
    Serial.println(K_LA[index]);
    Serial.print("SKR");
    Serial.println(K_RA[index]);
    index++;

    // Serial.print("LeftServoAngle: ");
    // Serial.println(LeftServoAngle);

    // Serial.print("RightServoAngle ");
    // Serial.println(RightServoAngle);

    // Servo_iLeft.write(LeftServoAngle);
    // Servo_iRight.write(RightServoAngle);

        // IK_ThetaAngle(x, y);

    //  delay(1000);
    Serial.print("INDEX: ");
    Serial.println(index);
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
    Serial.print("INDEX: ");
    Serial.println(index);
    x = bezierQuadratic(xZero, xOne, xTwo, t);
    y = bezierQuadratic(yZero, yOne, yTwo, t);

    // Serial.print("Air phase - x: ");
    // Serial.print(x);
    // Serial.print(", y: ");
    // Serial.println(y);

    ServoAngles angles = IK_ThetaAngle(x, y);
    Serial.print("ANGLES LEFT: ");
    Serial.println(angles.left);
    Serial.print("ANGLES RIGHT: ");
    Serial.println(angles.right);
    float LeftServoAngle = angles.left;
    float RightServoAngle = angles.right;

    I_LA[index] = LeftServoAngle;
    I_RA[index] = RightServoAngle;
    J_LA[index] = angles.left;
    J_RA[index] = angles.right;
    Serial.print("INDEX: ");
    Serial.println(index);
    Serial.print("SIR");
    Serial.println(I_LA[index]);
    Serial.print("SIL");
    Serial.println(I_RA[index]);
    index++;

    // Serial.print("SIL: ");
    // Serial.println(I_LA[index]);

    // Serial.print("SIR: ");
    // Serial.println(I_RA[index]);

    // Servo_iLeft.write(LeftServoAngle);
    // Servo_iRight.write(RightServoAngle);

        // IK_ThetaAngle(x, y);

    //  delay(1000);
  }

  // 🔵 Ground phase (foot back and flat)
  for (t = 0; t <= 1; t += delta_t) {
    // Serial.print("Index: ");
    // Serial.println(index);
    // move back linearly along the ground
    x = xTwo - t * step_length;
    y = ground_offset;

    // Serial.print("Ground phase - x: ");
    // Serial.print(x);
    // Serial.print(", y: ");
    // Serial.println(y);
    ServoAngles angles = IK_ThetaAngle(x, y);
    // float LeftServoAngle = angles.left;
    // float RightServoAngle = angles.right;
    I_LA[index] = angles.left;
    I_RA[index] = angles.right;
    I_LA[index] = angles.left;
    I_RA[index] = angles.right;
    J_LA[index] = angles.left;
    J_RA[index] = angles.right;
    BACK_I_LA[index] = angles.left;
    BACK_I_RA[index] = angles.right;

    BACK_J_LA[index] = angles.left;
    BACK_J_RA[index] = angles.right;
    Serial.print("INDEX: ");
    Serial.println(index);
    Serial.print("SIR");
    Serial.println(I_LA[index]);
    Serial.print("SIL");
    Serial.println(I_RA[index]);
    index++;

    // Serial.print("LeftServoAngle: ");
    // Serial.println(LeftServoAngle);

    // Serial.print("RightServoAngle ");
    // Serial.println(RightServoAngle);

    // Servo_iLeft.write(LeftServoAngle);
    // Servo_iRight.write(RightServoAngle);

        // IK_ThetaAngle(x, y);

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
    Serial.print("INDEX: ");
    Serial.println(index);
    Serial.print("SLL");
    Serial.println(L_LA[index]);
    Serial.print("SLR");
    Serial.println(L_RA[index]);
    index++;


    // Serial.print("LeftServoAngle: ");
    // Serial.println(LeftServoAngle);

    // Serial.print("RightServoAngle ");
    // Serial.println(RightServoAngle);

    // Servo_iLeft.write(LeftServoAngle);
    // Servo_iRight.write(RightServoAngle);

        // IK_ThetaAngle(x, y);

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
    ServoAngles angles = IK_ThetaAngle180(x, y);
    float LeftServoAngle = angles.left;
    float RightServoAngle = angles.right;
    BACK_L_LA[index] = angles.left;
    BACK_L_RA[index] = angles.right;
    L_LA[index] = angles.left;
    L_RA[index] = angles.right;
    Serial.print("INDEX: ");
    Serial.println(index);
    Serial.print("SLL");
    Serial.println(L_LA[index]);
    Serial.print("SLR");
    Serial.println(L_RA[index]);
    index++;

    // Serial.print("LeftServoAngle: ");
    // Serial.println(LeftServoAngle);

    // Serial.print("RightServoAngle ");
    // Serial.println(RightServoAngle);

    // Servo_iLeft.write(LeftServoAngle);
    // Servo_iRight.write(RightServoAngle);

        // IK_ThetaAngle(x, y);

    //  delay(1000);
  }
}

void moveAll3(){
  for(int i=0; i < maxSteps;i++){
    SIL.write(I_LA[i] + err[0][0]);
    SIR.write(I_RA[i] + err[0][1]);
    delay(60);
  }
  for(int i=0; i < maxSteps;i++){
    SIL.write(BACK_I_LA[i] + err[0][0]);
    SIR.write(BACK_I_RA[i] + err[0][1]);
    delay(60);
  }
}

void moveAll(){
  int count = 5;
  LSRamp.go(SIL.read(), SPEED);
  RSRamp.go(SIR.read(), SPEED);
  LSRamp2.go(SKL.read(), SPEED);
  RSRamp2.go(SKR.read(), SPEED);
  LSRamp3.go(SJL.read(), SPEED);
  RSRamp3.go(SJR.read(), SPEED);
  LSRamp4.go(SLL.read(), SPEED);
  RSRamp4.go(SLR.read(), SPEED);

  for (int i = 0; i < maxSteps; i++) {
    float leftTarget_J;
    float rightTarget_J;
    float leftTarget_L;
    float rightTarget_L;

    float leftTarget_I = I_LA[i] + err[0][0];
    float rightTarget_I = I_RA[i] + err[0][1];

    float leftTarget_K = K_LA[i] + err[2][0];
    float rightTarget_K = K_RA[i] + err[2][1];

    if(i < 7){
      leftTarget_J = J_LA[i + count] + err[1][0];
      rightTarget_J = J_RA[i + count] + err[1][1];

      leftTarget_L = L_LA[i + count] + err[3][0];
      rightTarget_L = L_RA[i + count] + err[3][1];
    } else{
      leftTarget_J = J_LA[i - 7] + err[1][0];
      rightTarget_J = J_RA[i -  7] + err[1][1];

      leftTarget_L = L_LA[i - 7] + err[3][0];
      rightTarget_L = L_RA[i - 7] + err[3][1];
    }

    LSRamp.go(leftTarget_I, SPEED);
    RSRamp.go(rightTarget_I, SPEED);
    LSRamp2.go(leftTarget_K, SPEED);
    RSRamp2.go(rightTarget_K, SPEED);
    LSRamp3.go(leftTarget_J, SPEED);
    RSRamp3.go(rightTarget_J, SPEED);
    LSRamp4.go(leftTarget_L, SPEED);
    RSRamp4.go(rightTarget_L, SPEED);

    while (
      !LSRamp.isFinished() || !RSRamp.isFinished() || !LSRamp2.isFinished() || !RSRamp2.isFinished() || !LSRamp3.isFinished() || !RSRamp3.isFinished() || !LSRamp4.isFinished() || !RSRamp4.isFinished()) {

      LSRamp.update();
      RSRamp.update();
      LSRamp2.update();
      RSRamp2.update();
      LSRamp3.update();
      RSRamp3.update();
      LSRamp4.update();
      RSRamp4.update();

      SIL.write(LSRamp.getValue());
      SIR.write(RSRamp.getValue());
      SKL.write(LSRamp2.getValue());
      SKR.write(RSRamp2.getValue());
      SJL.write(LSRamp3.getValue());
      SJR.write(RSRamp3.getValue());
      SLL.write(LSRamp4.getValue());
      SLR.write(RSRamp4.getValue());
    }
  }
}

void moveAll2() {
  LSRamp.go(SIL.read(), SPEED);
  RSRamp.go(SIR.read(), SPEED);
  LSRamp2.go(SKL.read(), SPEED);
  RSRamp2.go(SKR.read(), SPEED);
  LSRamp3.go(SJL.read(), SPEED);
  RSRamp3.go(SJR.read(), SPEED);
  LSRamp4.go(SLL.read(), SPEED);
  RSRamp4.go(SLR.read(), SPEED);
  // Stage 1: Move SIL/SIR + SKL/SKR first
  for (int i = 0; i < maxSteps; i++) {
    float leftTarget_I = I_LA[i] + err[0][0];
    float rightTarget_I = I_RA[i] + err[0][1];
    float leftTarget_K = K_LA[i] + err[2][0];
    float rightTarget_K = K_RA[i] + err[2][1];

    float leftTarget_J = BACK_J_LA[i] + err[1][0];
    float rightTarget_J = BACK_J_RA[i] + err[1][1];
    float leftTarget_L = BACK_L_LA[i] + err[3][0];
    float rightTarget_L = BACK_L_RA[i] + err[3][1];

    LSRamp.go(leftTarget_I, SPEED);
    RSRamp.go(rightTarget_I, SPEED);
    LSRamp2.go(leftTarget_K, SPEED);
    RSRamp2.go(rightTarget_K, SPEED);

    LSRamp3.go(leftTarget_J, SPEED);
    RSRamp3.go(rightTarget_J, SPEED);
    LSRamp4.go(leftTarget_L, SPEED);
    RSRamp4.go(rightTarget_L, SPEED);

    while (!LSRamp.isFinished() || !RSRamp.isFinished() || 
           !LSRamp2.isFinished() || !LSRamp3.isFinished() || !LSRamp4.isFinished() || !RSRamp2.isFinished() || !RSRamp3.isFinished() || !RSRamp4.isFinished() ) {

      LSRamp.update();
      RSRamp.update();
      LSRamp2.update();
      RSRamp2.update();

      LSRamp3.update();
      RSRamp3.update();
      LSRamp4.update();
      RSRamp4.update();

      SIL.write(LSRamp.getValue());
      SIR.write(RSRamp.getValue());
      SKL.write(LSRamp2.getValue());
      SKR.write(RSRamp2.getValue());

      SJL.write(LSRamp3.getValue());
      SJR.write(RSRamp3.getValue());
      SLL.write(LSRamp4.getValue());
      SLR.write(RSRamp4.getValue());

      Serial.print("SIL: ");
      Serial.println(LSRamp.getValue());
      
      Serial.print("SIR: ");
      Serial.println(RSRamp.getValue());

      Serial.print("SKL: ");
      Serial.println(LSRamp2.getValue());
      
      Serial.print("SKR: ");
      Serial.println(RSRamp2.getValue());

      Serial.print("SJL: ");
      Serial.println(LSRamp3.getValue());
      
      Serial.print("SJR: ");
      Serial.println(RSRamp3.getValue());

      Serial.print("SKL: ");
      Serial.println(LSRamp4.getValue());
      
      Serial.print("SKR: ");
      Serial.println(RSRamp4.getValue());
      
    }
  }

  // Stage 2: After hips and knees finish, move SJL/SJR + SLL/SLR
  for (int i = 0; i < maxSteps; i++) {
    float leftTarget_J = J_LA[i] + err[1][0];
    float rightTarget_J = J_RA[i] + err[1][1];
    float leftTarget_L = L_LA[i] + err[3][0];
    float rightTarget_L = L_RA[i] + err[3][1];

    float leftTarget_I = BACK_I_LA[i] + err[0][0];
    float rightTarget_I = BACK_I_RA[i] + err[0][1];
    float leftTarget_K = BACK_K_LA[i] + err[2][0];
    float rightTarget_K = BACK_K_RA[i] + err[2][1];

    LSRamp3.go(leftTarget_J, SPEED);
    RSRamp3.go(rightTarget_J, SPEED);
    LSRamp4.go(leftTarget_L, SPEED);
    RSRamp4.go(rightTarget_L, SPEED);

    LSRamp.go(leftTarget_I, SPEED);
    RSRamp.go(rightTarget_I, SPEED);
    LSRamp2.go(leftTarget_K, SPEED);
    RSRamp2.go(rightTarget_K, SPEED);

    while (!LSRamp.isFinished() || !RSRamp.isFinished() || 
           !LSRamp2.isFinished() || !LSRamp3.isFinished() || !LSRamp4.isFinished() || !RSRamp2.isFinished() || !RSRamp3.isFinished() || !RSRamp4.isFinished()) {

      LSRamp3.update();
      RSRamp3.update();
      LSRamp4.update();
      RSRamp4.update();

      LSRamp.update();
      RSRamp.update();
      LSRamp2.update();
      RSRamp2.update();

      SJL.write(LSRamp3.getValue());
      SJR.write(RSRamp3.getValue());
      SLL.write(LSRamp4.getValue());
      SLR.write(RSRamp4.getValue());

      SIL.write(LSRamp.getValue());
      SIR.write(RSRamp.getValue());
      SKL.write(LSRamp2.getValue());
      SKR.write(RSRamp2.getValue());
    }
  }
}


void move_IK() {
  LSRamp.go(SIL.read(), SPEED);
  RSRamp.go(SIR.read(), SPEED);
  LSRamp2.go(SKL.read(), SPEED);
  RSRamp2.go(SKR.read(), SPEED);

  for (int i = 0; i < maxSteps; i++) {
    float leftTarget_I = I_LA[i] + err[0][0];
    float rightTarget_I = I_RA[i] + err[0][1];
    float leftTarget_K = K_LA[i] + err[2][0];
    float rightTarget_K = K_RA[i] + err[2][1];

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
    float leftTarget_J = J_LA[i] + err[1][0];
    float rightTarget_J = J_RA[i] + err[1][1];
    float leftTarget_L = L_LA[i] + err[3][0];
    float rightTarget_L = L_RA[i] + err[3][1];

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

void initLeg(){
  LSRamp.go(SIL.read(), SPEED);
  RSRamp.go(SIR.read(), SPEED);
  LSRamp2.go(SKL.read(), SPEED);
  RSRamp2.go(SKR.read(), SPEED);
  LSRamp3.go(SJL.read(), SPEED);
  RSRamp3.go(SJR.read(), SPEED);
  LSRamp4.go(SLL.read(), SPEED);
  RSRamp4.go(SLR.read(), SPEED);

  float leftTarget_I = INIT + err[0][0];
  float rightTarget_I = INIT2 + err[0][1];

  float leftTarget_K = INIT + err[2][0];
  float rightTarget_K = INIT2 + err[2][1];

  float leftTarget_J = INIT + err[1][0];
  float rightTarget_J = INIT2 + err[1][1];

  float leftTarget_L = INIT180 + err[3][0];
  float rightTarget_L = INIT1802 + err[3][1];

  LSRamp.go(leftTarget_I, SPEED);
  RSRamp.go(rightTarget_I, SPEED);
  LSRamp2.go(leftTarget_K, SPEED);
  RSRamp2.go(rightTarget_K, SPEED);
  LSRamp3.go(leftTarget_J, SPEED);
  RSRamp3.go(rightTarget_J, SPEED);
  LSRamp4.go(leftTarget_L, SPEED);
  RSRamp4.go(rightTarget_L, SPEED);

  while (
    !LSRamp.isFinished() || !RSRamp.isFinished() || !LSRamp2.isFinished() || !RSRamp2.isFinished() || !LSRamp3.isFinished() || !RSRamp3.isFinished() || !LSRamp4.isFinished() || !RSRamp4.isFinished()) {

    LSRamp.update();
    RSRamp.update();
    LSRamp2.update();
    RSRamp2.update();
    LSRamp3.update();
    RSRamp3.update();
    LSRamp4.update();
    RSRamp4.update();

    SIL.write(LSRamp.getValue());
    SIR.write(RSRamp.getValue());
    SKL.write(LSRamp2.getValue());
    SKR.write(RSRamp2.getValue());
    SJL.write(LSRamp3.getValue());
    SJR.write(RSRamp3.getValue());
    SLL.write(LSRamp4.getValue());
    SLR.write(RSRamp4.getValue());
  }
}


void move_I() {
  LSRamp.go(SIL.read(), SPEED);
  RSRamp.go(SIR.read(), SPEED);

  for (int i = 0; i < maxSteps; i++) {
    float leftTarget = I_LA[i] + err[0][0];
    float rightTarget = I_RA[i] + err[0][1];

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
    float leftTarget = K_LA[i] + err[2][0];
    float rightTarget = K_RA[i] + err[2][1];

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

void move_L() {
  LSRamp.go(SKL.read(), SPEED);
  RSRamp.go(SKR.read(), SPEED);

  for (int i = 0; i < maxSteps; i++) {
    float leftTarget = L_LA[i] + err[3][0];
    float rightTarget = L_RA[i] + err[3][1];

    LSRamp.go(leftTarget, SPEED);
    RSRamp.go(rightTarget, SPEED);

    while (!LSRamp.isFinished() || !RSRamp.isFinished()) {
      LSRamp.update();
      RSRamp.update();
      SLL.write(LSRamp.getValue());
      SLR.write(RSRamp.getValue());
      // delay(10);
    }
  }
}

void settingServo() {
  if (Serial.available() > 0) {
    char ch = Serial.read();        
    int angle = Serial.parseInt(); 

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

    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}
