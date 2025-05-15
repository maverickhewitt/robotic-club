#include <CytronMotorDriver.h>

#define IR_PIN 32
#define DRIBBLE_PIN1 18
#define DRIBBLE_PIN2 19

#define EXTEND_PIN1 21
#define EXTEND_PIN2 22

int sensor;
CytronMD Dribble(PWM_DIR, DRIBBLE_PIN1, DRIBBLE_PIN2, 1);
CytronMD Extender(PWM_DIR, EXTEND_PIN1, EXTEND_PIN2, 2);

void setup() {
  Serial.begin(115200);
  pinMode(IR_PIN, INPUT);
  
  //pegang bole sedia atau close the hand
  Dribble.setSpeed(0);
}

void loop() {
  sensor = digitalRead(IR_PIN);
  Serial.println(sensor);

  if(sensor == 1) {
    //keluar extend tangan
    Extender.setSpeed(255);
    delay(600);
    Extender.setSpeed(0);
    
    //push bola pigi bawa dribbling
    Dribble.setSpeed(255);
    delay(300);
    //catxh bola only guna delay sikit
    Dribble.setSpeed(0);
    delay(300);
    
    //tarik balik tangan masuk
    Extender.setSpeed(-255);
    delay(600);
    Extender.setSpeed(0);
    delay(500);
  }
}