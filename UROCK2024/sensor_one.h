#ifndef SENSOR_ONE_H
#define SENSOR_ONE_H

#define OBJDP1 27
#define OBJDP2 13
#define OBJDP3 4

int OBJD1 = 0;
int OBJD2 = 0;
int OBJD3 = 0;

void setupOBJD() {
  pinMode(OBJDP1, INPUT);
  pinMode(OBJDP2, INPUT);
  pinMode(OBJDP3, INPUT);
}

void getOBJD() {
  OBJD1 = digitalRead(OBJDP1);
  OBJD2 = digitalRead(OBJDP2);
  OBJD3 = digitalRead(OBJDP3);
}

int isRamp() {
  return (OBJD1 && OBJD2 && OBJD3);
}

#endif
