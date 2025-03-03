#include <ESP32Servo.h>

int servoPin = 25;
int servoPin2 = 27;

Servo servo1;
Servo servo2;
 
void setup() {
  Serial.begin(9600);
  servo1.attach(servoPin);
  servo2.attach(servoPin2);
}

void loop() {
  if (Serial.available() > 0) {
    int angle = Serial.parseInt();

    if (angle >= 0 && angle <= 90) {
      servo1.write(angle);
      servo2.write(angle);
      Serial.print("Set angle: ");
      Serial.println(angle);
    } else {
      Serial.println("Please input an angle between 40 and 50");
    }
    
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}
