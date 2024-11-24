#ifndef SENSOR_TWO_H
#define SENSOR_TWO_H

#include <Wire.h>
#include <Adafruit_ICM20948.h>

Adafruit_ICM20948 icm;

// Flag to indicate if the initial reference direction is set
bool initialReferenceSet = false;
float referenceAngle = 0.0;

float avgAngle[5] = {0, 0, 0, 0, 0};
int avgCounter = 0;

void setupGyro() {
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip!");
    while (1) delay(10);
  }

  while (!initialReferenceSet && referenceAngle == 0) {
    sensors_event_t accel, gyro, temp, mag;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    float currentAngle = atan2(mag.magnetic.y, mag.magnetic.x);
    currentAngle = currentAngle * 180.0 / PI;
    referenceAngle = currentAngle;
  }

  initialReferenceSet = true;  
}

void updateGyro() {
  sensors_event_t accel, gyro, temp, mag;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  float currentAngle = atan2(mag.magnetic.y, mag.magnetic.x);
  currentAngle = currentAngle * 180.0 / PI;

  float angleDifference = referenceAngle - currentAngle;

  avgAngle[avgCounter++] = angleDifference;
  if (avgCounter == 5) avgCounter = 0;

  delay(1000); 
}

int getAvgGyro() {
  int average = 0;

  for (int i = 0; i < 5; i++) {
    average += avgAngle[i];
  }

  return (average/5);
}


#endif
