#include <CytronMotorDriver.h>
#define IR_PIN 32
#define PIN1 18
#define PIN2 19


int sensor;
int ballPassCounter = 0;
bool isBallPass = false;

CytronMD Dribble(PWM_DIR, PIN1, PIN2, 1);

void setup() {
  Serial.begin(115200);
  pinMode(IR_PIN, INPUT);
}

void loop() {
  sensor = digitalRead(IR_PIN);
  Serial.print("Value:");
  Serial.println(sensor);

  if(sensor = 1){
   Dribble.setSpeed(0);
   delay(500);
   Dribble.setSpeed(255);
   delay(500);
   Dribble.setSpeed(0);
  }
}
