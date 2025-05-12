#include <ESP32Encoder.h>
#include <L298N.h>

#define ENA1 13
#define IN1 12
#define IN2 14

#define ENA2 13
#define IN3 12
#define IN4 14

#define ENA3 27
#define IN5 26
#define IN6 25

#define ENA4 27
#define IN7 26
#define IN8 25

#define COUNTS_PER_METER 21640.0
#define TARGET_DISTANCE 1.0
#define SPEED 60

L298N motor1(ENA1,IN1,IN2);
L298N motor2(ENA2,IN3,IN4);
L298N motor3(ENA3,IN5,IN6);
L298N motor4(ENA4,IN7,IN8);

double input;
ESP32Encoder MotorEncoder;

void setup() {
  Serial.begin(115200);
  ESP32Encoder::useInternalWeakPullResistors = puType::none;
  MotorEncoder.attachFullQuad(33, 32);
  MotorEncoder.clearCount();

  motor1.setSpeed(SPEED);
  motor2.setSpeed(SPEED);
  motor3.setSpeed(SPEED);
  motor4.setSpeed(SPEED);

}

void loop() {
  input = MotorEncoder.getCount();
  // Serial.print("Input: ");
  // Serial.println(input);

  //untuk check distance
  double distance = input / COUNTS_PER_METER;
  // Serial.print("Distance (m): ");
  // Serial.println(distance);

  if (distance < TARGET_DISTANCE) {  
    motor1.forward();
    motor2.forward();
    motor3.backward();
    motor4.backward();
  } else {
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
    Serial.println("Target reached.");
    while(1); 
  }

  Serial.print("Distance (m): ");
  Serial.println(distance);
  delay(100);

}
