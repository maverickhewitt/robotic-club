// When Coding a Dc motor with Servo, the servo will not move. 
// The possible issue is Dc Motor and Servo shared the same PWM Channel and caused overlapping in ESP32. 
// So the way to solve this is we need to change the PWMChannel for Motor/Servo. 
// The easier way is to change PWMChannel for the Motor using ledcAttachChannel.
// analogwrite may be not a good way to use for dc motor.

#include <ESP32Servo.h>
#include <L298N.h>

int motor1Pin1 = 21; 
int motor1Pin2 = 19; 
int enable1Pin = 25; 

int servoPin = 4;

Servo gripper1;
L298N myMotor(enable1Pin, motor1Pin1, motor1Pin2);

// Setting PWM properties
const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

void setup() {
  // sets the pins as outputs
  
  // configure LEDC PWM
  // ledcAttachChannel(enable1Pin, freq, resolution, pwmChannel);
  ledcAttachChannel(enable1Pin, freq, resolution,1); // this is how to set the pwm channel for the motor
  gripper1.attach(servoPin);
  myMotor.setSpeed(255);


  Serial.begin(115200);

  // testing
  Serial.print("Testing DC Motor...");
}

void loop() {

  myMotor.forward();

  if (Serial.available() > 0) {   
    char input = Serial.read();   

    if(input == '1'){
      gripper1.write(90);       
    }else if(input == '2'){
      gripper1.write(0);      
    }
   }
}
