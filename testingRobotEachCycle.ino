#include <ESP32Servo.h>
#include <L298N.h>
#include <ezButton.h>

int servoPin1 = 25;
int servoPin2 = 27;
int ENA = 18;
int IN1 = 19;
int IN2 = 21;
int buttonPin1 = 22;
int buttonPin2 = 23;
int buttonPin3 = 13;
int buttonPin4 = 32;

int gripDeg = 25;
int releaseDeg = 45;
int openDeg = 80;

Servo servo1;
Servo servo2;
L298N dcMotor1(ENA, IN1, IN2);
ezButton button1(buttonPin1);
ezButton button2(buttonPin2);
ezButton button3(buttonPin3);
ezButton button4(buttonPin4);

void setup() {
  Serial.begin(115200);
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  button1.setDebounceTime(20);
  button2.setDebounceTime(20);
  button3.setDebounceTime(20);
  button4.setDebounceTime(20);
  ledcAttachChannel(ENA, 50, 8, 4);
  dcMotor1.setSpeed(255);
}

void loop() {
  button1.loop();
  button2.loop();
  button3.loop();
  button4.loop();

  if (button1.isPressed() || button2.isPressed()) {
    servo1.write(releaseDeg);
    servo2.write(gripDeg);
    Serial.println("Button 1 and 2 Pressed");
  }

  if (button3.isPressed() || button4.isPressed()) {
    servo1.write(gripDeg);
    servo2.write(releaseDeg);
    Serial.println("Button 3 and 4 Pressed");
  }

  if(Serial.available()>0){
    int input = Serial.parseInt();

    if(input == 1){
      dcMotor1.forward();
    }else if(input == 2){
      dcMotor1.backward();
    }else if(input == 3){
      servo1.write(gripDeg);
      servo2.write(releaseDeg);
    }else if(input == 4){
      servo1.write(releaseDeg);
      servo2.write(gripDeg);
    }else if(input == 5){
      servo1.write(gripDeg);
      servo2.write(gripDeg);
    }else if(input == 6){
      servo1.write(releaseDeg);
      servo2.write(releaseDeg);
    }else if(input == 7){
      servo1.write(openDeg);
      servo2.write(openDeg);
    }else if(input == 8){
      dcMotor1.stop();
    }
    }
  }
