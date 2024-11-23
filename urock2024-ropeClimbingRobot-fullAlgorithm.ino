#include <ESP32Servo.h>
#include <L298N.h>
#include <ezButton.h>

#define ENA 14
#define IN1 27 
#define IN2 26 

#define ENA2 32
#define IN3 25
#define IN4 33

#define servo1 23
#define servo2 22
#define servo3 21
#define servo4 19

#define button1Pin 35 //perlu buat pinMode as input
#define button2Pin 34 //perlu buat pinMode as input
#define buttonF1F2 4
#define buttonB1B2 18
#define button2F1F2 15
#define button2B1B2 13

Servo gripper1;
Servo gripper2; 
Servo gripper3;
Servo dartGripper;

L298N DCMotor1(ENA, IN1, IN2);
L298N DCMotor2(ENA2, IN3, IN4);

ezButton button1(button1Pin);
ezButton button2(button2Pin);
ezButton buttonF1(buttonF1F2);
ezButton buttonF2(buttonF1F2);
ezButton buttonB1(buttonB1B2);
ezButton buttonB2(buttonB1B2);
ezButton button2F1(button2F1F2);
ezButton button2F2(button2F1F2);
ezButton button2B1(button2B1B2);
ezButton button2B2(button2B1B2);

int flag = 0;
int defaultDelay = 300;
int motorSpeed = 1023;
int backToCenterDelay = 5000;
int debounceTime = 20;

int grip35kg = 95; //gripper 1
int grip25kg = 85; //gripper 2
int release35kg = 75;
int release25kg = 110;
int open35kg = 10;
int open25kg = 160;
int holdDart = 0;
int releaseDart = 90;
int delayGripper = 100;

int freq = 10000;
int reso = 10;

void setup() {
  Serial.begin(115200);

  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);

  button1.setDebounceTime(debounceTime); 
  button2.setDebounceTime(debounceTime); 
  buttonF1.setDebounceTime(debounceTime); 
  buttonF2.setDebounceTime(debounceTime); 
  buttonB1.setDebounceTime(debounceTime); 
  buttonB2.setDebounceTime(debounceTime); 
  button2F1.setDebounceTime(debounceTime); 
  button2F2.setDebounceTime(debounceTime); 
  button2B1.setDebounceTime(debounceTime); 
  button2B2.setDebounceTime(debounceTime); 

  ledcAttachChannel(ENA, freq, reso, 5);
  ledcAttachChannel(ENA2, freq, reso, 6);

  gripper1.attach(servo1);
  gripper2.attach(servo2);
  gripper3.attach(servo3);
  dartGripper.attach(servo4);

  gripper1.write(open35kg);
  gripper2.write(open25kg);
  gripper3.write(open25kg);
  dartGripper.write(holdDart);

  DCMotor1.setSpeed(motorSpeed);
  DCMotor2.setSpeed(motorSpeed);
}

void loop() {
  buttonLoop();

  if (button1.isPressed()) {
    Serial.println("Limit Switch 1 PRESSED");
    flag += 1;
    Serial.print("Flag: ");
    Serial.println(flag);
  }

  if (button2.isPressed()) {
    Serial.println("Limit Switch 1 PRESSED");
    flag += 1;
    Serial.print("Flag: ");
    Serial.println(flag);
  }

  //-----------------------------------------------------------

  if(buttonF1.isPressed() || buttonF2.isPressed()){
    gripper2.write(grip25kg);
    delay(delayGripper);
    gripper1.write(release35kg);
    Serial.println("Button F1 and Button F2 Pressed");
    Serial.println("gripper 2 gripped - gripper 1 released");
  }

  if(buttonB1.isPressed() || buttonB2.isPressed()){
    gripper1.write(grip35kg);
    delay(delayGripper);
    gripper2.write(release25kg);
    Serial.println("Button B1 and Button B2 Pressed");
    Serial.println("gripper 1 gripped - gripper 2 released");
  }

  if(flag == 5){
    if(button2F1.isPressed() || button2F2.isPressed()){
      gripper3.write(grip25kg);
      Serial.println("Button 2F1 and Button 2F2 Pressed");
      Serial.println("gripper 3 gripped");
    }

    if(button2B1.isPressed() || button2B2.isPressed()){
      gripper3.write(release25kg);
      Serial.println("Button2 B1 and Button2 B2 Pressed");
      Serial.println("gripper 3 released");
    }
  }
  else if(flag == 8){
    if(button2F1.isPressed() || button2F2.isPressed()){
      gripper3.write(release25kg);
      Serial.println("Button 2F1 and Button 2F2 Pressed");
      Serial.println("gripper 3 gripped");
    }

    if(button2B1.isPressed() || button2B2.isPressed()){
      gripper3.write(grip25kg);
      Serial.println("Button2 B1 and Button2 B2 Pressed");
      Serial.println("gripper 3 released");
    }
  }

  //-----------------------------------------------------------

  if(flag == 1){
    gripG1G2();
  }
  else if(flag == 2){
    climbUp();
  }
  else if(flag == 5){
    stop1();
    Serial.println("Robot Stopped");
    delay(defaultDelay);
    moveToRope2();
    Serial.println("Moving to Rope 2");
    delay(defaultDelay);
    climbUp2();
  }
  else if(flag == 8){
    stop2();
    Serial.println("Robot reached ENDPLATE");
    backToCenter();
    Serial.println("Back to center");
    delay(backToCenterDelay);
    stop2();
    Serial.println("Ready to drop dart");
    delay(2000);
    dropDart();
    Serial.println("Dart dropped successfully!");
  }
  else if(flag == 10){
    openGrip();
  }
  else if(flag == 12){
    reset();
  }
}

void buttonLoop(){
  button1.loop();
  button2.loop();
  buttonF1.loop();
  buttonF2.loop();
  buttonB1.loop();
  buttonB2.loop();
  button2F1.loop();
  button2F2.loop();
  button2B1.loop();
  button2B2.loop();
}

void gripG1G2(){
  gripper1.write(open35kg);
  gripper2.write(release25kg);
}

void openGrip(){
  gripper1.write(open35kg);
  gripper2.write(open25kg);
  gripper3.write(open25kg);
}

void climbUp(){
  DCMotor1.forward();
}

void climbUp2(){
  DCMotor2.forward();
}

void stop1(){
  DCMotor1.stop();
}

void stop2(){
  DCMotor2.stop();
}

void moveToRope2(){
  gripper3.write(open35kg);
  delay(defaultDelay);
  gripper3.write(grip35kg);
  delay(defaultDelay);
  DCMotor2.forward();
  delay(50);
  DCMotor2.stop();
  delay(defaultDelay);
  gripper1.write(open35kg);
  gripper2.write(open25kg);
}

void backToCenter(){
  DCMotor2.backward();
}

void dropDart(){
  dartGripper.write(releaseDart);
}

void reset(){
  flag = 0;
}
