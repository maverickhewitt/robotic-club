#include <Encoder.h>
// #include <L298N.h>

// #define IN1 25
// #define IN2 26
// #define ENA 27

#define ENCODER_PIN_A 34
#define ENCODER_PIN_B 35

// #define ENCODER_PPR 400 

// L298N motor(IN1, IN2, ENA);
Encoder motorEncoder(ENCODER_PIN_A, ENCODER_PIN_B);

void setup() {
  Serial.begin(115200);
}

void loop() {
  long pulses = motorEncoder.read();
  Serial.print("Encoder Pulses: ");
  Serial.println(pulses);
  delay(100);
}

//kasi pusing motor manual and read the pulses value, change di ENCODER_PPR based on what you like
