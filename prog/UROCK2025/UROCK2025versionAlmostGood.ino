#include <ESP32Servo.h>

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

int err[4][2] = {{8,3}, {0,0}, {0,0}, {0,0}};

unsigned long lastMoveTime = 0;
unsigned long currentMillis;

int sensor;

float totalTime = 2.0;  // seconds
int steps = 50;
float dt = totalTime / steps * 1000; // conver to ms that is why mau darab dnegan 1000
unsigned long prevMillis = 0;
unsigned long startTime = 0;
bool forward = true;

float a = 15.0;
float b = 20.0;
float dConst = 25.0;

float h = 25.0;
float L = 10;
float H = 6.0;
float x = 5.0;

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

float ramp(float current, float target, float step) {
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
  return current;
}

void loop() {
  // // Forward motion
  // LEG1S2.write(64);
  // delay(40);
  // LEG1S1.write(41);
  // delay(300);
  // LEG1S2.write(64);
  // delay(40);
  // LEG1S1.write(26);
  // delay(300);

  // LEG1S2.write(63);
  // delay(40);
  // LEG1S1.write(33);
  // delay(300);
  // LEG1S2.write(48); //more than 60 plus 3, less -
  // delay(40);
  // LEG1S1.write(23);
  // delay(300);

  for (int i = 0; i <= L; i++) {
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

    float hipDeg = hip * RAD_TO_DEG;
    float kneeDeg = knee * RAD_TO_DEG;
    float convHip = hipDeg * 2/3;
    float convKnee = kneeDeg * 2/3;

    // Serial.print("Step: ");
    // Serial.print(i);
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
    // LEG1S2.write(kneeDeg + err[0][1]);
    LEG1S2.write(convKnee + err[0][1]);
    delay(45);
  }

  // for (int i = 0; i <= steps; i++) {
  //   float t = (float)i / steps; 

  //   float coorX = L * (1.0 - t); 
  //   float coorY = 0.0;  

  //   float d = sqrt(sq(x - coorX) + sq(h - coorY));
    
  //   d = constrain(d, 0.1, a + b - 0.1);

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

  //   // Serial.print("Step: ");
  //   // Serial.print(i);
  //   Serial.print(" | CoorX: ");
  //   Serial.print(coorX);
  //   Serial.print(" | CoorY: ");
  //   Serial.print(coorY);
  //   Serial.print(" | d: ");
  //   Serial.print(d);
  //   Serial.print(" | Hip: ");
  //   Serial.print(hipDeg);
  //   Serial.print(" | Knee: ");
  //   Serial.println(kneeDeg);

  //   LEG1S1.write(hipDeg + err[0][0]);
  //   LEG1S2.write(kneeDeg + err[0][1]);
  //   delay(dt);
  // }
      
  // for (int i = 0; i <= steps; i++) {
  //     float t = (float)i / steps;

  //     float coorX = L * (t - (sin(2 * PI * t) / (2 * PI)));
  //     float coorY = (H / 2) * (1 - cos(2 * PI * t));

  //     float d = sqrt(sq(5 - coorX) + sq(25 - coorY));

  //     float knee = acos((sq(a) + sq(b) - sq(d)) / (2 * a * b));
  //     float hip;

  //     if (abs(d - 25.0) < 0.01) {
  //       hip = (PI / 2) - atan(coorX / h) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  //     } else {
  //       hip = (PI / 2) + atan(coorX / h) - acos((sq(d) + sq(a) - sq(b)) / (2 * a * d));
  //     }

  //     float hipDeg = hip * RAD_TO_DEG;
  //     float kneeDeg = knee * RAD_TO_DEG;

  //     Serial.print("Step: ");
  //     Serial.print(i);
  //     Serial.print(" | CoorX: ");
  //     Serial.print(coorX);
  //     Serial.print(" | CoorY: ");
  //     Serial.print(coorY);
  //     Serial.print(" | d: ");
  //     Serial.print(d);
  //     Serial.print(" | Hip: ");
  //     Serial.print(hipDeg);
  //     Serial.print(" | Knee: ");
  //     Serial.println(kneeDeg);

  //     LEG1S1.write(hipDeg - err[0][0]);
  //     LEG1S2.write(kneeDeg - err[0][1]);
      

  //     delay(dt); 
  //   }

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
