#include <ESP32Servo.h>
#include <L298N.h>
#include <ezButton.h>

int servoPin1 = 27;
int servoPin2 = 25;
int ENA = 18;
int IN1 = 19;
int IN2 = 21;
int buttonPin1 = 22;
int buttonPin2 = 32;
int buttonPin3 = 13;
int buttonPin4 = 23;

int grip25Deg = 85;
int grip35Deg = 95  ;
int release25Deg = 110;
int release35Deg = 75;
int open35Deg = 10;
int open25Deg = 160;
int delayGripper = 100;

int flag = 0;
int flag2 = 0;

Servo servo1;
Servo servo2;
L298N dcMotor1(ENA, IN1, IN2);
ezButton button1(buttonPin1);
ezButton button2(buttonPin2);
ezButton button3(buttonPin3);
ezButton button4(buttonPin4);

void setup() {
  Serial.begin(115200);
  servo1.attach(servoPin1); //servo 35
  servo2.attach(servoPin2);
  button1.setDebounceTime(20);
  button2.setDebounceTime(20);
  button3.setDebounceTime(20);
  button4.setDebounceTime(20);
  ledcAttachChannel(ENA, 5000, 8, 8);
  dcMotor1.setSpeed(255);
}

void loop() {
  button1.loop();
  button2.loop();
  button3.loop();
  button4.loop();

  

  if(flag == 0){
    if (button1.isPressed() || button2.isPressed()) {
    servo2.write(grip25Deg);
    delay(delayGripper);
    servo1.write(release35Deg);
    Serial.println("Button 1 and 2 Pressed");
  }

    if (button3.isPressed() || button4.isPressed()) {
      servo1.write(grip35Deg);
      delay(delayGripper);
      servo2.write(release25Deg);
      Serial.println("Button 3 and 4 Pressed");
    }
  }else{
    if (button1.isPressed() || button2.isPressed()) {
      servo1.write(grip35Deg);
      delay(delayGripper);
      servo2.write(release25Deg);
      Serial.println("Button 1 and 2 Pressed");
  }

  if (button3.isPressed() || button4.isPressed()) {
      servo2.write(grip25Deg);
      delay(delayGripper);
      servo1.write(release35Deg);
      Serial.println("Button 3 and 4 Pressed");
    }
  }

  if(Serial.available()>0){
    int input = Serial.parseInt();

    if(input == 1){
      dcMotor1.forward();
      Serial.println("Move motor forward");
      flag = 0;
    }else if(input == 2){
      dcMotor1.backward();
      Serial.println("Move motor backward");
      flag = 1;
    }else if(input == 3){
      servo1.write(grip35Deg);
      delay(250);
      servo2.write(release25Deg);
      Serial.println("Gripper 1 grip, Gripper 2 Release");
    }else if(input == 4){
      servo2.write(grip25Deg);
      delay(250);
      servo1.write(release35Deg);
      Serial.println("Gripper 1 release, Gripper 2 grip");
    }else if(input == 5){
      servo1.write(grip35Deg);
      servo2.write(grip25Deg);
      Serial.println("both gripper grip");
    }else if(input == 6){
      servo1.write(release35Deg);
      servo2.write(release25Deg);
      Serial.println("both gripper release");
    }else if(input == 7){
      servo1.write(open35Deg);
      servo2.write(open25Deg);
      Serial.println("both gripper open");
    }else if(input == 8){
      dcMotor1.stop();
      Serial.println("Motor Stop");
    }
  }
}
