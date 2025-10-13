#include <ESP32Servo.h>

#define LEG1PIN1 26
#define LEG1PIN2 12

#define LEG2PIN1 25
#define LEG2PIN2 26

#define LEG3PIN1 19
#define LEG3PIN2 18

#define LEG4PIN1 17
#define LEG4PIN2 16

#define IR_PIN 27

#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_HERTZ 300

#define RAD_TO_DEG 57.295779513

int err[4][2] = {{8,0}, {0,0}, {0,0}, {0,0}};

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

float a = 15.0;
float b = 20.0;
float d = 25.0;

float x[4] = {5.0, 0.0, 0.0, 0.0};
float h[4] = {5.0, 0.0, 0.0, 0.0};

float L = 2.0;
float H = 2.0;

// float teta1 = (atan(x/h) * RAD_TO_DEG);
float teta2 = (acos((sq(d)+sq(a)-sq(b))/(2*a*d)) * RAD_TO_DEG);

// float hip = 90 - teta1 - teta2;
float knee = (acos((sq(a)+sq(b)-sq(d))/(2*a*b)) * RAD_TO_DEG);

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
  // standingLeg();
  // delay(1000);
}

void loop() {
  sensor = digitalRead(IR_PIN);
  Serial.print("Sensor:");
  Serial.println(sensor);
  delay(200);

  // for(stepIndex = 0; i<5;i++){
  //   float teta1 = (atan(x[stepIndex]/h[stepIndex]) * RAD_TO_DEG);
  //   float hip = 90 - teta1 - teta2;
  // }
  // stepIndex = 0;
  
  // Serial.print("Hip:");
  // Serial.println(hip);
  // Serial.print("Knee:");
  // Serial.println(knee);
  // delay(200);

  // LEG1S1.write(hip - err[0][0]);
  // LEG1S2.write(knee - err[0][1]);
  // delay(200);
  
  // LEG1S1.write(45 - err[0][0]);
  // if (Serial.available() > 0) {
  //   char code = Serial.read(); 
  //   int angle = Serial.parseInt();

  //   if (angle >= 0 && angle <= 270) {
  //     if (code == 'A' || code == 'a') { 

  //       LEG1S1.write(angle);

  //       Serial.print("Servo S1 set to angle: ");
  //       Serial.println(angle);
  //     } else if (code == 'B' || code == 'b') {

  //       LEG1S2.write(angle);
        
  //       Serial.print("Servo S2 set to angle: ");
  //       Serial.println(angle);
  //     } 
  //     else {
  //       Serial.println("Invalid command. Use A<angle> or B<angle>");
  //     }
  //   } else {
  //     Serial.println("Please input an angle between 0 and 180.");
  //   }

  //   while (Serial.available() > 0) {
  //     Serial.read();
  //   }
  // }
  // LEG1S1.write(75 + err[0][0]);
  // LEG1S2.write(45 + err[0][1]);
  // delay(300);
  // LEG1S1.write(68 + err[0][0]);
  // LEG1S2.write(49 + err[0][1]);
  // delay(300);
  // LEG1S1.write(59 + err[0][0]);
  // LEG1S2.write(24 + err[0][1]);
  // delay(300);
  // LEG1S1.write(64 + err[0][0]);
  // LEG1S2.write(19 + err[0][1]);
  // delay(300);
  // sensor = digitalRead(IR_PIN);
  // Serial.print("SENSOR: ");
  // Serial.println(sensor);
  // if(sensor == 0 && isClimbingMode){
  //   isClimbingMode = false;
  //   stepIndex = 0;
  // } 
  // else if(sensor == 1 && !isClimbingMode){
  //   isClimbingMode = true;
  //   stepIndex = 0;
  // }

  // if(isClimbingMode) {
  //   climbingL1();
  // } else {
  //   moveL1();
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
