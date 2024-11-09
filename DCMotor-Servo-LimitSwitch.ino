#include <ezButton.h>
#include <ESP32Servo.h>
#include <L298N.h>

#define ENA 18
#define IN1 19 
#define IN2 21 

int servo = 22;

Servo dart;
ezButton button(17);  
L298N motor(ENA, IN1, IN2);

int flag = 0;

void setup() {
  Serial.begin(9600);
  button.setDebounceTime(1); 

  ledcAttachChannel(ENA, 5000, 8, 1);

  dart.attach(servo);
  dart.write(0);

  motor.setSpeed(255);
  motor.forward();
}

void loop() {
  button.loop();
  if (button.isPressed()) {
      Serial.println(F("The limit switch: TOUCHED"));
      Serial.print("Flag: ");
      Serial.println(flag);
      flag += 1;
    }

  if(flag == 3){
    motor.stop();
    delay(2000);
    motor.backward();
    delay(2000);
    motor.stop();
    delay(1000);
    dart.write(90);
  }
}
