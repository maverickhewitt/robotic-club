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

Servo gripper1;
Servo gripper2; 
Servo dartGripper;
L298N DCMotor1(ENA, IN1, IN2);
L298N DCMotor2(ENA2, IN3, IN4);
ezButton button1(2);
ezButton button2(4);

int flag = 0;
int defaultDelay = 300;
int motorSpeed = 255;
int backToCenterDelay = 5000;

void setup() {
  Serial.begin(115200);
  button1.setDebounceTime(1); 
  button2.setDebounceTime(1); 
  ledcAttachChannel(ENA, 5000, 8, 1);
  ledcAttachChannel(ENA2, 5000, 8, 2);

  gripper1.attach(servo1);
  gripper2.attach(servo2);
  dartGripper.attach(servo3);
  gripper1.write(50);
  gripper2.write(50);
  dartGripper.write(0);

  DCMotor1.setSpeed(motorSpeed);
  DCMotor2.setSpeed(motorSpeed);
}

void loop() {
  button1.loop();
  button2.loop();

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

void climbUp(){
  DCMotor1.forward();
  gripper1.write(0);
  delay(defaultDelay);
  gripper1.write(20);
  delay(defaultDelay);
}

void climbUp2(){
  DCMotor2.forward();
  gripper1.write(0);
  delay(defaultDelay);
  gripper1.write(20);
  delay(defaultDelay);
}

void stop1(){
  DCMotor1.stop();
  gripper1.write(0);
}

void stop2(){
  DCMotor2.stop();
  gripper2.write(0);
}

void moveToRope2(){
  gripper1.write(50);
  delay(defaultDelay);
  gripper2.write(0);
  delay(defaultDelay);
}

void backToCenter(){
  DCMotor2.backward();
  gripper1.write(0);
  delay(defaultDelay);
  gripper1.write(20);
  delay(defaultDelay);
}

void dropDart(){
  dartGripper.write(90);
}


