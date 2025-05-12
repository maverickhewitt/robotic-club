#include <ESP32Encoder.h>

double input;
ESP32Encoder MotorEncoder;

void setup() {
  Serial.begin(115200);
  ESP32Encoder::useInternalWeakPullResistors = puType::none;
  MotorEncoder.attachFullQuad(27, 26);
  MotorEncoder.clearCount();

}

void loop() {
  input = MotorEncoder.getCount();
  Serial.print("Input: ");
  Serial.println(input);
}
