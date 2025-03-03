#define sensor 18

void setup() {
  Serial.begin(115200);
  pinMode(sensor, INPUT);
}

void loop() {
  int state = digitalRead(sensor);

  if (state == LOW)
    Serial.println("got obstacle");
  else
    Serial.println("no obstacle");

  delay(100);
}
