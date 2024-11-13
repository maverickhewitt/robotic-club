#include <ESP32Servo.h>
#include <L298N.h>
#include <ezButton.h>

#define ENA 18
#define IN1 19 
#define IN2 21 

#define ENA2 25
#define IN3 26
#define IN4 27

#define servo1 14
#define servo2 16
#define servo3 17
#define servo4 12

Servo gripper1;
Servo gripper2; 
Servo gripper3;
Servo dartGripper;
L298N DCMotor1(ENA, IN1, IN2);
L298N DCMotor2(ENA2, IN3, IN4);
ezButton button1(2);
ezButton button2(4);
ezButton buttonF1(4);
ezButton buttonF2(4);
ezButton buttonB1(4);
ezButton buttonB2(4);
ezButton button2F1(4);
ezButton button2F2(4);
ezButton button2B1(4);
ezButton button2B2(4);

int flag = 0;
int defaultDelay = 300;
int motorSpeed = 255;
int backToCenterDelay = 5000;
int debounceTime = 20;

void setup() {
  Serial.begin(115200);
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
  ledcAttachChannel(ENA, 5000, 8, 1);
  ledcAttachChannel(ENA2, 5000, 8, 2);

  gripper1.attach(servo1);
  gripper2.attach(servo2);
  gripper3.attach(servo3);
  dartGripper.attach(servo4);

  gripper1.write(50);
  gripper2.write(50);
  gripper3.write(50);
  dartGripper.write(0);

  DCMotor1.setSpeed(motorSpeed);
  DCMotor2.setSpeed(motorSpeed);
}

void loop() {
  buttonLoop()

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

  if(flag == 1){
    delay(3000);
    climbUp();
  }else if(flag == 3){
    stop1();
    Serial.println("Robot Stopped");
    delay(defaultDelay);
    moveToRope2();
    Serial.println("Moving to Rope 2");
    delay(defaultDelay);
    climbUp2();
  }else if(flag == 6){
    Serial.println("Robot reached ENDPLATE");
    backToCenter();
    Serial.println("Back to center");
    delay(backToCenterDelay);
    stop2();
    Serial.println("Ready to drop dart");
    delay(1000);
    dropDart();
    Serial.println("Dart dropped successfully!");
  }

  // switch(flag){
  //   case 1:
  //     checkFlag();
  //     climbUp();
  //     delay(defaultDelay);
  //     break;
  //   case 3:
  //     checkFlag();
  //     stop1();
  //     Serial.println("Robot Stopped");
  //     delay(defaultDelay);
  //     moveToRope2();
  //     Serial.println("Moving to Rope 2");
  //     delay(defaultDelay);
  //     climbUp2();
  //     break;
  //   case 6:
  //     Serial.println("Robot reached ENDPLATE");
  //     backToCenter();
  //     Serial.println("Back to center");
  //     delay(backToCenterDelay);
  //     stop2();
  //     Serial.println("Ready to drop dart");
  //     delay(1000);
  //     dropDart();
  //     Serial.println("Dart dropped successfully!");
  //     break;
  //   default: 
  //     stop2();
  // }
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

void climbUp(){
  DCMotor1.forward();
  gripper1.write(0);
  delay(defaultDelay);
  gripper1.write(20);
  delay(defaultDelay);
}

void climbUp2(){
  DCMotor2.forward();
  if(buttonF1.isPressed() || buttonF2.isPressed()){
    gripper1.write(50);
    gripper2.write(30);
    Serial.println("Button F1 and Button F2 Pressed");
    Serial.println("gripper 1 released - gripper 2 gripped");
  }

  if(buttonB1.isPressed() || buttonB2.isPressed()){
    gripper1.write(30);
    gripper2.write(50);
    Serial.println("Button B1 and Button B2 Pressed");
    Serial.println("gripper 1 gripped - gripper 2 released");
  }
}

void stop1(){
  DCMotor1.stop();
  gripper1.write(30);
  gripper2.write(30);
}

void stop2(){
  DCMotor2.stop();
  gripper3.write(30);
}

void moveToRope2(){
  if(button2F1.isPressed() || button2F2.isPressed()){
    gripper3.write(30);
    Serial.println("Button 2F1 and Button 2F2 Pressed");
    Serial.println("gripper 3 gripped");
  }

  if(button2B1.isPressed() || button2B2.isPressed()){
    gripper3.write(50);
    Serial.println("Button B1 and Button B2 Pressed");
    Serial.println("gripper 1 gripped - gripper 2 released");
  }
}

void backToCenter(){
  DCMotor2.backward();
  if(button2F1.isPressed() || button2F2.isPressed()){
    gripper3.write(50);
    Serial.println("Button 2F1 and Button 2F2 Pressed");
    Serial.println("gripper 3 gripped");
  }

  if(button2B1.isPressed() || button2B2.isPressed()){
    gripper3.write(30);
    Serial.println("Button B1 and Button B2 Pressed");
    Serial.println("gripper 1 gripped - gripper 2 released");
  }
}

void dropDart(){
  dartGripper.write(90);
}
