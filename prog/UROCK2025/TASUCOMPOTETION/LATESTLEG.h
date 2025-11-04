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

#define SPEED 60
#define JUMPSPEED 20
#define RAMPSPEED 200
#define STOMPSPEED 140
#define READYSPEED 500
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_HERTZ 300
#define RESOLUTION 8
#define INIT 78
#define INIT2 42
#define INIT180 117
#define INIT1802 63

#define LIFTLEG 63
#define LIFTLEG2 56

#define LIFTLEG_L 95
#define LIFTLEG2_L 85

#define SIXTY 60
#define NINETY 90

#define IJOFFSET 22
#define RAMPGROUNDOFFSET 20
#define RAMPGROUNDOFFSETBACK 20
#define RAMPHEIGHTOFFSET 50

#define UP_RAMP 14

int err[4][2] = {{3,4}, {3,6}, {1,6}, {4,8}};

bool flag = true;

float pi = M_PI;  // M_PI is defined as 3.14159265358979323846
float x;
float y;

const float L1 = 130.0;
const float L2 = 195.0;
const float DistanceBetweenServo = 34.8;
const float ground_offset = 200.0;
const float ground_offset_ramp = ground_offset - RAMPGROUNDOFFSET;
const float ground_offset_rampback = ground_offset + RAMPGROUNDOFFSETBACK;


const float step_length = 90;
const float step_height = 50;
const float step_height_ramp = step_height;
const float small_step_length = 50;

float delta_t = 0.2;  //the best fast 0.4 0.3 slow but clear

const int maxSteps = 12;
// const int maxSteps = (int)(2 * (1.0 / delta_t) + 2);

float I_LA[maxSteps];
float I_RA[maxSteps];

float RAMP_I_LA[maxSteps];
float RAMP_I_RA[maxSteps];

float BACK_I_LA[maxSteps];
float BACK_I_RA[maxSteps];

float INIT_I_LA[1];
float INIT_I_RA[1];

float J_LA[maxSteps];
float J_RA[maxSteps];

float RAMP_J_LA[maxSteps];
float RAMP_J_RA[maxSteps];

float BACK_J_LA[maxSteps];
float BACK_J_RA[maxSteps];

float INIT_J_LA[1];
float INIT_J_RA[1];

float K_LA[maxSteps];
float K_RA[maxSteps];

float RAMP_K_LA[maxSteps];
float RAMP_K_RA[maxSteps];

float BACK_K_LA[maxSteps];
float BACK_K_RA[maxSteps];

float INIT_K_LA[1];
float INIT_K_RA[1];

float L_LA[maxSteps];
float L_RA[maxSteps];

float RAMP_L_LA[maxSteps];
float RAMP_L_RA[maxSteps];

float BACK_L_LA[maxSteps];
float BACK_L_RA[maxSteps];

float INIT_L_LA[1];
float INIT_L_RA[1];

// float x0 = -step_length/2;
// float yZero =  ground_offset;

// float xOne = 0;
// float yOne = ground_offset + 2 * step_height;

// float xTwo = step_length/2;
// float yTwo = ground_offset;

float xZero = -step_length / 2;  //WHY
float xZeroKSMALL = -step_length / 2;  //WHY
float xZeroIJ = -(step_length + IJOFFSET) / 2; 
float xZeroSMALL = -(small_step_length + IJOFFSET) / 2; //WHY
// float xZero = -step_length / 2;  //WHY
float yZero = ground_offset;
float yZeroRAMP = ground_offset_ramp;
float yZeroRAMPBACK = ground_offset_rampback;

// float xOne = (34.8/2);
float xOne = 0;
float yOne = ground_offset - 2 * step_height;
float yOneRAMP = ground_offset_ramp - 2 * step_height;
float yOneRAMPBACK = ground_offset_rampback - 2 * step_height;

// float xTwo = step_length / 2 - (34.8/2);
float xTwo = step_length / 2;
float xTwoKSMALL = small_step_length / 2;
float xTwoIJ = (step_length + IJOFFSET) / 2;
float xTwoSMALL = (small_step_length + IJOFFSET) / 2;
float yTwo = ground_offset;
float yTwoRAMP = ground_offset_ramp;
float yTwoRAMPBACK = ground_offset_rampback;

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
    // float LeftServoAngle = angles.left;
    // float RightServoAngle = angles.right;
    K_LA[index] = angles.left;
    K_RA[index] = angles.right;

    if(index == 0){
      INIT_K_LA[index] = angles.left;
      INIT_K_RA[index] = angles.right;
    }
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
    // Serial.print("INDEX: ");
    // Serial.println(index);
  }
}

void leg_k_ramp() {
  float t;
  int index = 0;

  // 🟢 Air phase (foot up and forward)
  for (t = 0; t <= 1; t += delta_t) {
    x = bezierQuadratic(xTwoKSMALL, xOne, xZeroKSMALL, t);
    y = bezierQuadratic(yTwoRAMP, yOneRAMP, yZeroRAMP, t);

    ServoAngles angles = IK_ThetaAngle(x, y);

    RAMP_K_LA[index] = angles.left;
    RAMP_K_RA[index] = angles.right;

    index++;
    
  }

  // 🔵 Ground phase (foot back and flat)
  for (t = 0; t <= 1; t += delta_t) {
    // move back linearly along the ground
    x = xZeroKSMALL + t * small_step_length;
    y = ground_offset_ramp;

    ServoAngles angles = IK_ThetaAngle(x, y);

    RAMP_K_LA[index] = angles.left;
    RAMP_K_RA[index] = angles.right;
    index++;
  }
}

void leg_i() {  //and j
  float t;
  int index = 0;

  //ikd how but ts works RIGHT SIDE

  // 🟢 Air phase (foot up and forward)
  for (t = 0; t <= 1; t += delta_t) {
    Serial.print("INDEX: ");
    Serial.println(index);
    x = bezierQuadratic(xZeroIJ, xOne, xTwoIJ, t);
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
    
    if(index == 0){
      INIT_I_LA[index] = angles.left;
      INIT_I_RA[index] = angles.right;
      INIT_J_LA[index] = angles.left;
      INIT_J_RA[index] = angles.right;
    }
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
    x = xTwoIJ - t * (step_length + IJOFFSET);
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

void leg_i_ramp() {  //and j
  float t;
  int index = 0;

  // 🟢 Air phase (foot up and forward)
  for (t = 0; t <= 1; t += delta_t) {
    Serial.print("INDEX: ");
    Serial.println(index);
    x = bezierQuadratic(xZeroSMALL, xOne, xTwoSMALL, t);
    y = bezierQuadratic(yZeroRAMPBACK, yOneRAMPBACK, yTwoRAMPBACK, t);

    ServoAngles angles = IK_ThetaAngle(x, y);

    RAMP_I_LA[index] = angles.left;
    RAMP_I_RA[index] = angles.right;

    // RAMP_J_LA[index] = angles.left;
    // RAMP_J_RA[index] = angles.right;
    
    index++;
  }

  // 🔵 Ground phase (foot back and flat)
  for (t = 0; t <= 1; t += delta_t) {
    x = xTwoSMALL - t * (small_step_length + IJOFFSET);
    y = ground_offset_rampback;

    ServoAngles angles = IK_ThetaAngle(x, y);
    RAMP_I_LA[index] = angles.left;
    RAMP_I_RA[index] = angles.right;

    RAMP_J_LA[index] = angles.left;
    RAMP_J_RA[index] = angles.right;
    index++;
  }
}

void leg_j_ramp() {  //and j
  float t;
  int index = 0;

  // 🟢 Air phase (foot up and forward)
  for (t = 0; t <= 1; t += delta_t) {
    Serial.print("INDEX: ");
    Serial.println(index);
    x = bezierQuadratic(xZeroSMALL, xOne, xTwoSMALL, t);
    y = bezierQuadratic(yZeroRAMP, yOneRAMP, yTwoRAMP, t);

    ServoAngles angles = IK_ThetaAngle(x, y);

    RAMP_J_LA[index] = angles.left;
    RAMP_J_RA[index] = angles.right;
    
    index++;
  }

  // 🔵 Ground phase (foot back and flat)
  for (t = 0; t <= 1; t += delta_t) {
    x = xTwoSMALL - t * (small_step_length + IJOFFSET);
    y = ground_offset_ramp;

    ServoAngles angles = IK_ThetaAngle(x, y);

    RAMP_J_LA[index] = angles.left;
    RAMP_J_RA[index] = angles.right;
    index++;
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
    if(index == 0){
      INIT_L_LA[index] = angles.left;
      INIT_L_RA[index] = angles.right;
    }
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

void leg_l_ramp() {

  float t;
  int index = 0;

  // 🟢 Air phase (foot up and forward)
  for (t = 0; t <= 1; t += delta_t) {
    x = bezierQuadratic(xTwoSMALL, xOne, xZeroSMALL, t);
    y = bezierQuadratic(yTwoRAMPBACK, yOneRAMPBACK, yZeroRAMPBACK, t);

    ServoAngles angles = IK_ThetaAngle180(x, y);  

    RAMP_L_LA[index] = angles.left;
    RAMP_L_RA[index] = angles.right;
    index++;
  }

  // 🔵 Ground phase (foot back and flat)
  for (t = 0; t <= 1; t += delta_t) {
    x = xZeroSMALL + t * small_step_length;
    y = ground_offset_rampback;

    ServoAngles angles = IK_ThetaAngle180(x, y);

    RAMP_L_LA[index] = angles.left;
    RAMP_L_RA[index] = angles.right;
    index++;
  }
}

void stompLeg60() {
  int LIFT_SPEED = STOMPSPEED;   
  // int DROP_SPEED = STOMPSPEED;       
  int DROP_SPEED = STOMPSPEED / 2;       

  LSRamp.go(SIL.read(), LIFT_SPEED);
  RSRamp.go(SIR.read(), LIFT_SPEED);
  LSRamp2.go(SKL.read(), LIFT_SPEED);
  RSRamp2.go(SKR.read(), LIFT_SPEED);
  LSRamp3.go(SJL.read(), LIFT_SPEED);
  RSRamp3.go(SJR.read(), LIFT_SPEED);
  LSRamp4.go(SLL.read(), LIFT_SPEED);
  RSRamp4.go(SLR.read(), LIFT_SPEED);

  float leftTarget_I = INIT + err[0][0];
  float rightTarget_I = INIT2 + err[0][1];

  float leftTarget_K = INIT + err[2][0];
  float rightTarget_K = INIT2 + err[2][1];

  float leftTarget_J = SIXTY + err[1][0];  
  float rightTarget_J = SIXTY + err[1][1];

  float leftTarget_L = NINETY + err[3][0];
  float rightTarget_L = NINETY + err[3][1];

  LSRamp.go(leftTarget_I, DROP_SPEED);
  RSRamp.go(rightTarget_I, DROP_SPEED);
  LSRamp2.go(leftTarget_K, DROP_SPEED);
  RSRamp2.go(rightTarget_K, DROP_SPEED);
  LSRamp3.go(leftTarget_J, LIFT_SPEED);
  RSRamp3.go(rightTarget_J, LIFT_SPEED);
  LSRamp4.go(leftTarget_L, LIFT_SPEED);
  RSRamp4.go(rightTarget_L, LIFT_SPEED);

  while (
    !LSRamp.isFinished() || !RSRamp.isFinished() ||
    !LSRamp2.isFinished() || !RSRamp2.isFinished() ||
    !LSRamp3.isFinished() || !RSRamp3.isFinished() ||
    !LSRamp4.isFinished() || !RSRamp4.isFinished()
  ) {
    LSRamp.update(); RSRamp.update();
    LSRamp2.update(); RSRamp2.update();
    LSRamp3.update(); RSRamp3.update();
    LSRamp4.update(); RSRamp4.update();

    SIL.write(LSRamp.getValue());
    SIR.write(RSRamp.getValue());
    SKL.write(LSRamp2.getValue());
    SKR.write(RSRamp2.getValue());
    SJL.write(LSRamp3.getValue());
    SJR.write(RSRamp3.getValue());
    SLL.write(LSRamp4.getValue());
    SLR.write(RSRamp4.getValue());
  }

  // delay(STOMPSPEED); 

  leftTarget_I = SIXTY + err[0][0];
  rightTarget_I = SIXTY + err[0][1];

  leftTarget_K = SIXTY + err[2][0];
  rightTarget_K = SIXTY + err[2][1];

  leftTarget_J = INIT + err[1][0];
  rightTarget_J = INIT2 + err[1][1];

  leftTarget_L = INIT180 + err[3][0];
  rightTarget_L = INIT1802 + err[3][1];

  LSRamp.go(leftTarget_I, LIFT_SPEED);
  RSRamp.go(rightTarget_I, LIFT_SPEED);
  LSRamp2.go(leftTarget_K, LIFT_SPEED);
  RSRamp2.go(rightTarget_K, LIFT_SPEED);
  LSRamp3.go(leftTarget_J, DROP_SPEED);
  RSRamp3.go(rightTarget_J, DROP_SPEED);
  LSRamp4.go(leftTarget_L, DROP_SPEED);
  RSRamp4.go(rightTarget_L, DROP_SPEED);

  while (
    !LSRamp.isFinished() || !RSRamp.isFinished() ||
    !LSRamp2.isFinished() || !RSRamp2.isFinished() ||
    !LSRamp3.isFinished() || !RSRamp3.isFinished() ||
    !LSRamp4.isFinished() || !RSRamp4.isFinished()
  ) {
    LSRamp.update(); RSRamp.update();
    LSRamp2.update(); RSRamp2.update();
    LSRamp3.update(); RSRamp3.update();
    LSRamp4.update(); RSRamp4.update();

    SIL.write(LSRamp.getValue());
    SIR.write(RSRamp.getValue());
    SKL.write(LSRamp2.getValue());
    SKR.write(RSRamp2.getValue());
    SJL.write(LSRamp3.getValue());
    SJR.write(RSRamp3.getValue());
    SLL.write(LSRamp4.getValue());
    SLR.write(RSRamp4.getValue());
  }

  // delay(STOMPSPEED);
}

void stompLegHike() {
  int LIFT_SPEED = STOMPSPEED;   
  int DROP_SPEED = STOMPSPEED;       
  // int DROP_SPEED = STOMPSPEED / 2;       

  LSRamp.go(SIL.read(), LIFT_SPEED);
  RSRamp.go(SIR.read(), LIFT_SPEED);
  LSRamp2.go(SKL.read(), LIFT_SPEED);
  RSRamp2.go(SKR.read(), LIFT_SPEED);
  LSRamp3.go(SJL.read(), LIFT_SPEED);
  RSRamp3.go(SJR.read(), LIFT_SPEED);
  LSRamp4.go(SLL.read(), LIFT_SPEED);
  RSRamp4.go(SLR.read(), LIFT_SPEED);

  float leftTarget_I = I_LA[0] + err[0][0];
  float rightTarget_I = I_RA[0] + err[0][1];

  float leftTarget_K = K_LA[0] + err[2][0];
  float rightTarget_K = K_RA[0] + err[2][1];

  float leftTarget_J = SIXTY + err[1][0];  
  float rightTarget_J = SIXTY + err[1][1];

  float leftTarget_L = NINETY + err[3][0];
  float rightTarget_L = NINETY + err[3][1];

  LSRamp.go(leftTarget_I, DROP_SPEED);
  RSRamp.go(rightTarget_I, DROP_SPEED);
  LSRamp2.go(leftTarget_K, DROP_SPEED);
  RSRamp2.go(rightTarget_K, DROP_SPEED);
  LSRamp3.go(leftTarget_J, LIFT_SPEED);
  RSRamp3.go(rightTarget_J, LIFT_SPEED);
  LSRamp4.go(leftTarget_L, LIFT_SPEED);
  RSRamp4.go(rightTarget_L, LIFT_SPEED);

  while (
    !LSRamp.isFinished() || !RSRamp.isFinished() ||
    !LSRamp2.isFinished() || !RSRamp2.isFinished() ||
    !LSRamp3.isFinished() || !RSRamp3.isFinished() ||
    !LSRamp4.isFinished() || !RSRamp4.isFinished()
  ) {
    LSRamp.update(); RSRamp.update();
    LSRamp2.update(); RSRamp2.update();
    LSRamp3.update(); RSRamp3.update();
    LSRamp4.update(); RSRamp4.update();

    SIL.write(LSRamp.getValue());
    SIR.write(RSRamp.getValue());
    SKL.write(LSRamp2.getValue());
    SKR.write(RSRamp2.getValue());
    SJL.write(LSRamp3.getValue());
    SJR.write(RSRamp3.getValue());
    SLL.write(LSRamp4.getValue());
    SLR.write(RSRamp4.getValue());
  }

  // delay(STOMPSPEED); 

  leftTarget_I = SIXTY + err[0][0];
  rightTarget_I = SIXTY + err[0][1];

  leftTarget_K = SIXTY + err[2][0];
  rightTarget_K = SIXTY + err[2][1];

  leftTarget_J = J_LA[0] + err[1][0];
  rightTarget_J = J_RA[0] + err[1][1];

  leftTarget_L = L_LA[0] + err[3][0];
  rightTarget_L = L_RA[0] + err[3][1];

  LSRamp.go(leftTarget_I, LIFT_SPEED);
  RSRamp.go(rightTarget_I, LIFT_SPEED);
  LSRamp2.go(leftTarget_K, LIFT_SPEED);
  RSRamp2.go(rightTarget_K, LIFT_SPEED);
  LSRamp3.go(leftTarget_J, DROP_SPEED);
  RSRamp3.go(rightTarget_J, DROP_SPEED);
  LSRamp4.go(leftTarget_L, DROP_SPEED);
  RSRamp4.go(rightTarget_L, DROP_SPEED);

  while (
    !LSRamp.isFinished() || !RSRamp.isFinished() ||
    !LSRamp2.isFinished() || !RSRamp2.isFinished() ||
    !LSRamp3.isFinished() || !RSRamp3.isFinished() ||
    !LSRamp4.isFinished() || !RSRamp4.isFinished()
  ) {
    LSRamp.update(); RSRamp.update();
    LSRamp2.update(); RSRamp2.update();
    LSRamp3.update(); RSRamp3.update();
    LSRamp4.update(); RSRamp4.update();

    SIL.write(LSRamp.getValue());
    SIR.write(RSRamp.getValue());
    SKL.write(LSRamp2.getValue());
    SKR.write(RSRamp2.getValue());
    SJL.write(LSRamp3.getValue());
    SJR.write(RSRamp3.getValue());
    SLL.write(LSRamp4.getValue());
    SLR.write(RSRamp4.getValue());
  }

  // delay(STOMPSPEED);
}

void stompLeg() {
  int LIFT_SPEED = STOMPSPEED;   
  int DROP_SPEED = STOMPSPEED;       
  // int DROP_SPEED = STOMPSPEED / 2;       

  LSRamp.go(SIL.read(), LIFT_SPEED);
  RSRamp.go(SIR.read(), LIFT_SPEED);
  LSRamp2.go(SKL.read(), LIFT_SPEED);
  RSRamp2.go(SKR.read(), LIFT_SPEED);
  LSRamp3.go(SJL.read(), LIFT_SPEED);
  RSRamp3.go(SJR.read(), LIFT_SPEED);
  LSRamp4.go(SLL.read(), LIFT_SPEED);
  RSRamp4.go(SLR.read(), LIFT_SPEED);

  float leftTarget_I = INIT + err[0][0];
  float rightTarget_I = INIT2 + err[0][1];

  float leftTarget_K = INIT + err[2][0];
  float rightTarget_K = INIT2 + err[2][1];

  float leftTarget_J = LIFTLEG + err[1][0];  
  float rightTarget_J = LIFTLEG2 + err[1][1];

  float leftTarget_L = LIFTLEG_L + err[3][0];
  float rightTarget_L = LIFTLEG_L + err[3][1];

  LSRamp.go(leftTarget_I, DROP_SPEED);
  RSRamp.go(rightTarget_I, DROP_SPEED);
  LSRamp2.go(leftTarget_K, DROP_SPEED);
  RSRamp2.go(rightTarget_K, DROP_SPEED);
  LSRamp3.go(leftTarget_J, LIFT_SPEED);
  RSRamp3.go(rightTarget_J, LIFT_SPEED);
  LSRamp4.go(leftTarget_L, LIFT_SPEED);
  RSRamp4.go(rightTarget_L, LIFT_SPEED);

  while (
    !LSRamp.isFinished() || !RSRamp.isFinished() ||
    !LSRamp2.isFinished() || !RSRamp2.isFinished() ||
    !LSRamp3.isFinished() || !RSRamp3.isFinished() ||
    !LSRamp4.isFinished() || !RSRamp4.isFinished()
  ) {
    LSRamp.update(); RSRamp.update();
    LSRamp2.update(); RSRamp2.update();
    LSRamp3.update(); RSRamp3.update();
    LSRamp4.update(); RSRamp4.update();

    SIL.write(LSRamp.getValue());
    SIR.write(RSRamp.getValue());
    SKL.write(LSRamp2.getValue());
    SKR.write(RSRamp2.getValue());
    SJL.write(LSRamp3.getValue());
    SJR.write(RSRamp3.getValue());
    SLL.write(LSRamp4.getValue());
    SLR.write(RSRamp4.getValue());
  }

  // delay(STOMPSPEED); 

  leftTarget_I = LIFTLEG + err[0][0];
  rightTarget_I = LIFTLEG2 + err[0][1];

  leftTarget_K = LIFTLEG + err[2][0];
  rightTarget_K = LIFTLEG2 + err[2][1];

  leftTarget_J = INIT + err[1][0];
  rightTarget_J = INIT2 + err[1][1];

  leftTarget_L = INIT180 + err[3][0];
  rightTarget_L = INIT1802 + err[3][1];

  LSRamp.go(leftTarget_I, LIFT_SPEED);
  RSRamp.go(rightTarget_I, LIFT_SPEED);
  LSRamp2.go(leftTarget_K, LIFT_SPEED);
  RSRamp2.go(rightTarget_K, LIFT_SPEED);
  LSRamp3.go(leftTarget_J, DROP_SPEED);
  RSRamp3.go(rightTarget_J, DROP_SPEED);
  LSRamp4.go(leftTarget_L, DROP_SPEED);
  RSRamp4.go(rightTarget_L, DROP_SPEED);

  while (
    !LSRamp.isFinished() || !RSRamp.isFinished() ||
    !LSRamp2.isFinished() || !RSRamp2.isFinished() ||
    !LSRamp3.isFinished() || !RSRamp3.isFinished() ||
    !LSRamp4.isFinished() || !RSRamp4.isFinished()
  ) {
    LSRamp.update(); RSRamp.update();
    LSRamp2.update(); RSRamp2.update();
    LSRamp3.update(); RSRamp3.update();
    LSRamp4.update(); RSRamp4.update();

    SIL.write(LSRamp.getValue());
    SIR.write(RSRamp.getValue());
    SKL.write(LSRamp2.getValue());
    SKR.write(RSRamp2.getValue());
    SJL.write(LSRamp3.getValue());
    SJR.write(RSRamp3.getValue());
    SLL.write(LSRamp4.getValue());
    SLR.write(RSRamp4.getValue());
  }

  // delay(STOMPSPEED);
}

void firstStep(){
  int count = 6;
  int SPEED_FOWARD = SPEED / 2;
  while(flag){
  LSRamp.go(SIL.read(), SPEED);
  RSRamp.go(SIR.read(), SPEED);
  LSRamp2.go(SKL.read(), SPEED);
  RSRamp2.go(SKR.read(), SPEED);
  LSRamp3.go(SJL.read(), SPEED);
  RSRamp3.go(SJR.read(), SPEED);
  LSRamp4.go(SLL.read(), SPEED);
  RSRamp4.go(SLR.read(), SPEED);

  for (int i = 3; i < maxSteps; i++) {
    if(i == 11){
      flag = false;
    }
    float leftTarget_J;
    float rightTarget_J;
    float leftTarget_L;
    float rightTarget_L;

    float leftTarget_I = I_LA[i] + err[0][0];
    float rightTarget_I = I_RA[i] + err[0][1];

    float leftTarget_K = K_LA[i] + err[2][0];
    float rightTarget_K = K_RA[i] + err[2][1];

    if(i < 6){
      leftTarget_J = J_LA[i + count] + err[1][0];
      rightTarget_J = J_RA[i + count] + err[1][1];

      leftTarget_L = L_LA[i + count] + err[3][0];
      rightTarget_L = L_RA[i + count] + err[3][1];

      LSRamp.go(leftTarget_I, SPEED_FOWARD);
      RSRamp.go(rightTarget_I, SPEED_FOWARD);
      LSRamp2.go(leftTarget_K, SPEED_FOWARD);
      RSRamp2.go(rightTarget_K, SPEED_FOWARD);

      LSRamp3.go(leftTarget_J, SPEED);
      RSRamp3.go(rightTarget_J, SPEED);
      LSRamp4.go(leftTarget_L, SPEED);
      RSRamp4.go(rightTarget_L, SPEED);
    } else{
      leftTarget_J = J_LA[i - 6] + err[1][0];
      rightTarget_J = J_RA[i -  6] + err[1][1];

      leftTarget_L = L_LA[i - 6] + err[3][0];
      rightTarget_L = L_RA[i - 6] + err[3][1];

      LSRamp.go(leftTarget_I, SPEED);
      RSRamp.go(rightTarget_I, SPEED);
      LSRamp2.go(leftTarget_K, SPEED);
      RSRamp2.go(rightTarget_K, SPEED);

      LSRamp3.go(leftTarget_J, SPEED_FOWARD);
      RSRamp3.go(rightTarget_J, SPEED_FOWARD);
      LSRamp4.go(leftTarget_L, SPEED_FOWARD);
      RSRamp4.go(rightTarget_L, SPEED_FOWARD);
    }

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
}

void moveAll(){
  bool isReady = false; 
  int count = 5;
  int SPEED_FOWARD = SPEED / 4;
  int SPEED_SLOW = SPEED * 2;
  // LSRamp.go(SIL.read(), SPEED);
  // RSRamp.go(SIR.read(), SPEED);
  // LSRamp2.go(SKL.read(), SPEED_SLOW);
  // RSRamp2.go(SKR.read(), SPEED_SLOW);
  // LSRamp3.go(SJL.read(), SPEED);
  // RSRamp3.go(SJR.read(), SPEED);
  // LSRamp4.go(SLL.read(), SPEED);
  // RSRamp4.go(SLR.read(), SPEED);

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

      LSRamp.go(leftTarget_I, SPEED_FOWARD);
      RSRamp.go(rightTarget_I, SPEED_FOWARD);
      LSRamp2.go(leftTarget_K, SPEED);
      RSRamp2.go(rightTarget_K, SPEED);

      LSRamp3.go(leftTarget_J, SPEED_FOWARD);
      RSRamp3.go(rightTarget_J, SPEED_FOWARD);
      LSRamp4.go(leftTarget_L, SPEED);
      RSRamp4.go(rightTarget_L, SPEED);
    } else{
      leftTarget_J = J_LA[i - 7] + err[1][0];
      rightTarget_J = J_RA[i -  7] + err[1][1];

      leftTarget_L = L_LA[i - 7] + err[3][0];
      rightTarget_L = L_RA[i - 7] + err[3][1];

      LSRamp.go(leftTarget_I, SPEED);
      RSRamp.go(rightTarget_I, SPEED);
      LSRamp2.go(leftTarget_K, SPEED);
      RSRamp2.go(rightTarget_K, SPEED);

      LSRamp3.go(leftTarget_J, SPEED_FOWARD);
      RSRamp3.go(rightTarget_J, SPEED_FOWARD);
      LSRamp4.go(leftTarget_L, SPEED_FOWARD);
      RSRamp4.go(rightTarget_L, SPEED_FOWARD);
    }

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

void moveAllLow(){
  int count = 5;
  LSRamp.go(SIL.read(), RAMPSPEED);
  RSRamp.go(SIR.read(), RAMPSPEED);
  LSRamp2.go(SKL.read(), RAMPSPEED);
  RSRamp2.go(SKR.read(), RAMPSPEED);
  LSRamp3.go(SJL.read(), RAMPSPEED);
  RSRamp3.go(SJR.read(), RAMPSPEED);
  LSRamp4.go(SLL.read(), RAMPSPEED);
  RSRamp4.go(SLR.read(), RAMPSPEED);

  for (int i = 0; i < maxSteps; i++) {
    float leftTarget_J;
    float rightTarget_J;
    float leftTarget_L;
    float rightTarget_L;

    float leftTarget_I = RAMP_I_LA[i] + err[0][0] + UP_RAMP;
    float rightTarget_I = RAMP_I_RA[i] + err[0][1] + UP_RAMP;

    float leftTarget_K = RAMP_K_LA[i] + err[2][0];
    float rightTarget_K = RAMP_K_RA[i] + err[2][1];

    if(i < 7){
      leftTarget_J = RAMP_J_LA[i + count] + err[1][0];
      rightTarget_J = RAMP_J_RA[i + count] + err[1][1];

      leftTarget_L = RAMP_L_LA[i + count] + err[3][0] + UP_RAMP;
      rightTarget_L = RAMP_L_RA[i + count] + err[3][1] + UP_RAMP;

      LSRamp.go(leftTarget_I, RAMPSPEED);
      RSRamp.go(rightTarget_I, RAMPSPEED);
      LSRamp2.go(leftTarget_K, RAMPSPEED);
      RSRamp2.go(rightTarget_K, RAMPSPEED);

      LSRamp3.go(leftTarget_J, RAMPSPEED);
      RSRamp3.go(rightTarget_J, RAMPSPEED);
      LSRamp4.go(leftTarget_L, RAMPSPEED);
      RSRamp4.go(rightTarget_L, RAMPSPEED);
    } else{
      leftTarget_J = RAMP_J_LA[i - 7] + err[1][0];
      rightTarget_J = RAMP_J_RA[i -  7] + err[1][1];

      leftTarget_L = RAMP_L_LA[i - 7] + err[3][0] + UP_RAMP;
      rightTarget_L = RAMP_L_RA[i - 7] + err[3][1] + UP_RAMP;

      LSRamp.go(leftTarget_I, RAMPSPEED);
      RSRamp.go(rightTarget_I, RAMPSPEED);
      LSRamp2.go(leftTarget_K, RAMPSPEED);
      RSRamp2.go(rightTarget_K, RAMPSPEED);

      LSRamp3.go(leftTarget_J, RAMPSPEED);
      RSRamp3.go(rightTarget_J, RAMPSPEED);
      LSRamp4.go(leftTarget_L, RAMPSPEED);
      RSRamp4.go(rightTarget_L, RAMPSPEED);
    }

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

  float leftTarget_J = INIT  + err[1][0];
  float rightTarget_J = INIT2 + err[1][1];

  float leftTarget_L = INIT180  + err[3][0];
  float rightTarget_L = INIT1802  + err[3][1];

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

void move_I(){
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

void move_J(){
  LSRamp2.go(SJL.read(), SPEED);
  RSRamp2.go(SJR.read(), SPEED);

  for (int i = 0; i < maxSteps; i++) {
    float leftTarget = J_LA[i] + err[1][0];
    float rightTarget = J_RA[i] + err[1][1];

    LSRamp2.go(leftTarget, SPEED);
    RSRamp2.go(rightTarget, SPEED);

    while (!LSRamp2.isFinished() || !RSRamp2.isFinished()) {
      LSRamp2.update();
      RSRamp2.update();

      SJL.write(LSRamp2.getValue());
      SJR.write(RSRamp2.getValue());

      // delay(10);
    }
  }
}

void move_K() {
  LSRamp3.go(SKL.read(), SPEED);
  RSRamp3.go(SKR.read(), SPEED);

  for (int i = 0; i < maxSteps; i++) {
    float leftTarget = K_LA[i] + err[2][0];
    float rightTarget = K_RA[i] + err[2][1];

    LSRamp3.go(leftTarget, SPEED);
    RSRamp3.go(rightTarget, SPEED);

    while (!LSRamp3.isFinished() || !RSRamp3.isFinished()) {
      LSRamp3.update();
      RSRamp3.update();
      SKL.write(LSRamp3.getValue());
      SKR.write(RSRamp3.getValue());
      // delay(10);
    }
  }
}

void move_L() {
  LSRamp4.go(SLL.read(), SPEED);
  RSRamp4.go(SLR.read(), SPEED);

  for (int i = 0; i < maxSteps; i++) {
    float leftTarget = L_LA[i] + err[3][0];
    float rightTarget = L_RA[i] + err[3][1];

    LSRamp4.go(leftTarget, SPEED);
    RSRamp4.go(rightTarget, SPEED);

    while (!LSRamp4.isFinished() || !RSRamp4.isFinished()) {
      LSRamp4.update();
      RSRamp4.update();
      SLL.write(LSRamp4.getValue());
      SLR.write(RSRamp4.getValue());
      // delay(10);
    }
  }
}

void moveUpRamp(){
  for(int i = 0; i < 6; i++){
    SKL.write(RAMP_K_LA[i]);
    SKR.write(RAMP_K_RA[i]);
    delay(25);
  }
  delay(300);
  for(int i = 0; i < 6; i++){
    SJL.write(RAMP_J_LA[i]);
    SJR.write(RAMP_J_RA[i]);
    delay(25);
  }
  delay(300);
  for(int i = 0; i < 6; i++){
    SLL.write(RAMP_L_LA[i]);
    SLR.write(RAMP_L_RA[i]);
    delay(25);
  }
  delay(300);
  for(int i = 0; i < 6; i++){
    SIL.write(RAMP_I_LA[i]);
    SIR.write(RAMP_I_LA[i]);
    delay(25);
  }
  delay(300);
  for(int i = 6; i < maxSteps; i++){
    SKL.write(RAMP_K_LA[i]);
    SKR.write(RAMP_K_RA[i]);
    SJL.write(RAMP_J_LA[i]);
    SJR.write(RAMP_J_RA[i]);
    SLL.write(RAMP_L_LA[i]);
    SLR.write(RAMP_L_RA[i]);
    SIL.write(RAMP_I_LA[i]);
    SIR.write(RAMP_I_LA[i]);
    delay(25);
  }
}

void settingServo() {
  if (Serial.available() > 0) {
    char ch = Serial.read();        
    int angle = Serial.parseInt(); 

    // Determine angle limit based on servo
    int limit = (ch == 'r' || ch == 'i') ? 142 : 120;

    if (angle >= 0 && angle <= limit) {
      switch (ch) {
        case 'q':  
          SIL.write(angle + err[0][0]);
          Serial.print("SIL set to: ");
          Serial.println(angle);
          break;

        case 'w': 
          SJL.write(angle + err[1][0]);
          Serial.print("SJL set to: ");
          Serial.println(angle);
          break;

        case 'e':  
          SKL.write(angle + err[2][0]);
          Serial.print("SKL set to: ");
          Serial.println(angle);
          break;

        case 'r':  
          SLL.write(angle + err[3][0]);
          Serial.print("SLL set to: ");
          Serial.println(angle);
          break;

        case 't': 
          SIR.write(angle + err[0][1]);
          Serial.print("SIR set to: ");
          Serial.println(angle);
          break;

        case 'y': 
          SJR.write(angle + err[1][1]);
          Serial.print("SJR set to: ");
          Serial.println(angle);
          break;

        case 'u':  
          SKR.write(angle + err[2][1]);
          Serial.print("SKR set to: ");
          Serial.println(angle);
          break;

        case 'i': 
          SLR.write(angle + err[3][1]);
          Serial.print("SLR set to: ");
          Serial.println(angle);
          break;

        default:
          Serial.println("Invalid command! Use q,w,e,r,t,y,u,i + angle");
          break;
      }
    } else {
      Serial.print("Angle out of range! Max for this servo is ");
      Serial.println(limit);
    }

    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}
