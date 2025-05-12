#define IR_PIN 32

int sensor;

void setup() {
  Serial.begin(115200);
  pinMode(IR_PIN, INPUT);

}

void loop() {
  sensor = digitalRead(IR_PIN);
  Serial.print("Value:");
  Serial.println(sensor);
}
