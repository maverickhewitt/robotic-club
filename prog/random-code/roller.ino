#include <ESP32Servo.h>
#include <L298N.h>

int motor1Pin1 = 21; 
int motor1Pin2 = 19; 
int enable1Pin = 25; 

int servoPin = 4;

Servo gripper1;
L298N myMotor(enable1Pin, motor1Pin1, motor1Pin2);

const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 8;

void setup() {
  
ledcAttachChannel(enable1Pin, freq, resolution,1);
tangan1.attach(servoPin);
myMotor.setSpeed(255);
Serial.begin(115200);
}

void loop() {

//if sensor sense the ball {
  tangan1.write(turun bawa);
  motor1.forward()
// } else{
  tangan1.write(naik);
  motor1.stop();
 }
}