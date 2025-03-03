#include <ESP32Servo.h>
#include <L298N.h>
#include <ezButton.h>

int ENA = 13;
int IN1 = 14;
int IN2 = 27;
int servo = 16;
int servoPin2 = 25;

ezButton button1(19);
ezButton button2(21);
ezButton button3(22);
ezButton button4(23);
L298N motor1(ENA, IN1, IN2);

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(115200);
  servo1.attach(servo);
  servo2.attach(servoPin2);

  ledcAttachChannel(ENA, 5000, 8, 2);
  motor1.setSpeed(255);
  servo1.write(50);
  servo2.write(50);

  button1.setDebounceTime(20);
  button2.setDebounceTime(20);
  button3.setDebounceTime(20);
  button4.setDebounceTime(20);

}

void loop() {
  button1.loop();
  button2.loop();
  button3.loop();
  button4.loop();

  motor1.forward();

  if(button1.isPressed() || button2.isPressed()){
    servo1.write(70);
    // servo2.write(0);
    Serial.println("Button 1 and Button 2 Pressed");
    Serial.println("Servo 70 deg");
  }

  if(button3.isPressed() || button4.isPressed()){
    servo1.write(0);

    // servo2.write(20);  
    Serial.println("Button 3 and Button 4 Pressed");
    Serial.println("Servo 0 deg");
  }
}
