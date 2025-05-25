#include <CytronMotorDriver.h>
#include <ps5Controller.h>

#define IR_PIN 17
#define DRIBBLE_PIN1 21
#define DRIBBLE_PIN2 22

#define CLAW_PIN1 18
#define CLAW_PIN2 19

#define EXTEND_PIN1 27
#define EXTEND_PIN2 26

int sensor;
int input;

int defaultDelay = 200;

unsigned long previousMillis = 0;
unsigned long previousSpeedUpdate = 0;
unsigned long speedUpdateInterval = 300;

CytronMD Dribble(PWM_DIR, DRIBBLE_PIN1, DRIBBLE_PIN2, 1);
CytronMD Claw(PWM_DIR, CLAW_PIN1, CLAW_PIN2, 2);
CytronMD Extender(PWM_DIR, EXTEND_PIN1, EXTEND_PIN2, 3);

void setup() {
  Serial.begin(115200);
  pinMode(IR_PIN, INPUT);

  ps5.begin("64:17:CD:54:0D:AF");
  if (ps5.isConnected() == true){
     Serial.println("connected");
     delay(3000);
  }
  
  //pegang bole sedia atau close the hand
  Dribble.setSpeed(0);
  Claw.setSpeed(0);
}

void loop() {
  while (ps5.isConnected() == true){
    sensor = digitalRead(IR_PIN);

    if (millis() - previousSpeedUpdate >= speedUpdateInterval) {
      if (ps5.L1()) {
        defaultDelay += 20;
        previousSpeedUpdate = millis();
        Serial.print("Delay:");
        Serial.println(defaultDelay);
      }
      else if (ps5.L2()) {
        defaultDelay -= 20;
        previousSpeedUpdate = millis();
        Serial.print("Delay:");
        Serial.println(defaultDelay);
      }
    }

    if(ps5.Cross()) {
      Dribble.setSpeed(255);
      Claw.setSpeed(255);
      Serial.println("Claw Buka");
      delay(100);
      Dribble.setSpeed(0);

      while (digitalRead(IR_PIN) == 0) {
        //tunggu detect bola, jadi memang kosong
      }

      delay(defaultDelay);
      Claw.setSpeed(0);
      Serial.println("Tangkap Bola");
    }
  }
}
