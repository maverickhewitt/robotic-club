#include <ESP32Servo.h>
#include <L298N.h>
#include <ezButton.h>

int servoPin1 = 23 ;
int servoPin2 = 22;
int servoPin3 = 21;

int ENA = 14;
int IN1 = 27;
int IN2 = 26;

int ENA2 = 32;
int IN3 = 25;
int IN4 = 33;

int buttonPin1 = 4; //F1
int buttonPin2 = 15; //F2
int buttonPin3 = 18; //B1
int buttonPin4 = 13 ; //B2

int button1CondongPin = 35;
int button2LurusPin = 34;

int grip35Deg = 95;
int grip25Deg = 85;
int release25Deg = 125;
int release35Deg = 55; 
int open35Deg = 10;
int open25Deg = 160;
int gripper3DegRELEASE = 150; 
int gripper3DegGRIP = 100 ; 
int delayGripper = 100;

int flag = 0;
int flag2 = 0;

bool check = false;
int defaultDelay = 300;
int motorSpeed = 255;
int backToCenterDelay = 5000;
int debounceTime = 20;

Servo servo1;
Servo servo2;
Servo servo3;

L298N dcMotor1(ENA, IN1, IN2);
L298N dcMotor2(ENA2, IN3, IN4);

ezButton buttonF1(buttonPin1);
ezButton buttonF2(buttonPin2);
ezButton buttonB1(buttonPin3);
ezButton buttonB2(buttonPin4);

ezButton button1Condong(button1CondongPin);
ezButton button2Lurus(button2LurusPin);

void setup() {
  Serial.begin(115200);
  servo1.attach(servoPin1); //servo 35
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);

  buttonF1.setDebounceTime(20);
  buttonF2.setDebounceTime(20);
  buttonB1.setDebounceTime(20);
  buttonB2.setDebounceTime(20);

  button1Condong.setDebounceTime(20);
  button2Lurus.setDebounceTime(20);

  ledcAttachChannel(ENA, 5000, 8, 8);
  ledcAttachChannel(ENA2, 5000, 8, 9);
  dcMotor1.setSpeed(255);
  dcMotor2.setSpeed(255);

  pinMode(button1CondongPin, INPUT);
  pinMode(button2LurusPin, INPUT);
  pinMode(buttonPin2, INPUT);

  servo1.write(open35Deg);
  servo2.write(open25Deg);
  servo3.write(open25Deg);
}

void loop() {
  button1Condong.loop();
  button2Lurus.loop();

  buttonF1.loop();
  buttonB1.loop();

  buttonF2.loop();
  buttonB2.loop();

  if (button1Condong.isPressed() || button2Lurus.isPressed()) {
    if (!check) { 
        Serial.println("Limit Switch 1 PRESSED");
        flag++;
        Serial.print("Flag: ");
        Serial.println(flag);
        check = true; 
    }
  } else if (button1Condong.isReleased() || button2Lurus.isReleased()) {
      Serial.println("Limit Switch 1 RELEASED");
      check = false; 
  }

  if(buttonF1.isPressed()) {
    servo2.write(grip25Deg);
    delay(delayGripper);
    servo1.write(release35Deg);
    Serial.println("Button F1");
  }

  if(buttonB1.isPressed()) {
      servo1.write(grip35Deg);
      delay(delayGripper);
      servo2.write(release25Deg);
      Serial.println("Button B1");
    }

    if(buttonF2.isPressed()) {
    servo3.write(grip25Deg);
    Serial.println("Button F2");
  }

  if(buttonB2.isPressed()) {
      servo3.write(release25Deg);
      Serial.println("Button B2");
    }

    if(flag == 0){
      servo1.write(open35Deg);
      servo2.write(open25Deg);
      servo3.write(open25Deg);
    }else if(flag == 1){
      servo1.write(grip35Deg);
      servo2.write(grip25Deg);
      servo3.write(grip25Deg);
    }else if(flag == 2){
      dcMotor1.forward();
    }else if(flag == 5){
      dcMotor1.stop();
      delay(500);
      servo3.write(open25Deg);
      delay(500);
      servo3.write(grip25Deg);
      delay(500);
      dcMotor2.backward();
      delay(200);
      dcMotor2.stop();
      servo1.write(open35Deg);
      servo2.write(open25Deg);
      delay(500);
      flag = 6;
    }else if(flag == 6){
      dcMotor2.backward();
    }else if(flag == 8){
      dcMotor2.stop();
      delay(500);
      dcMotor2.backward();
      delay(6000);
      dcMotor2.stop();
      delay(1000);
      //dropDart disini
      flag = 9;
    }else if(flag == 11){
      servo1.write(open35Deg);
      servo2.write(open25Deg);
      servo3.write(open25Deg);
      delay(500);
      flag = 0;
    }
}
