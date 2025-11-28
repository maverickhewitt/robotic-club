#include <Wire.h>
#include <Adafruit_ICM20948.h>

Adafruit_ICM20948 icm;

struct Orientation {
  float yaw;   // Z (compass heading)
  float pitch; // X (forward/back tilt)
  float roll;  // Y (left/right tilt)
};

Orientation orientation;

float yawRef = 0;
bool yawRefSet = false;

void setupOrientation() {
  if (!icm.begin_I2C()) {
    Serial.println("IMU not detected!");
    while (1) delay(10);
  }

  while (!yawRefSet) {
    sensors_event_t accel, gyro, temp, mag;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    float yaw = atan2(mag.magnetic.y, mag.magnetic.x) * 180.0 / PI;
    yawRef = yaw;
    yawRefSet = true;
  }
}

void updateOrientation() {
  sensors_event_t accel, gyro, temp, mag;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // YAW (compass)
  orientation.yaw = (atan2(mag.magnetic.y, mag.magnetic.x) * 180.0 / PI) - yawRef;

  // PITCH (forward tilt)
  orientation.pitch = atan2(accel.acceleration.x, sqrt(accel.acceleration.y*accel.acceleration.y + accel.acceleration.z*accel.acceleration.z)) * 180.0 / PI;

  // ROLL (side tilt)
  orientation.roll = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;
}

float getYaw(){ 
  return orientation.yaw;   
}

float getPitch(){ 
  return orientation.pitch; 
}

float getRoll(){ 
  return orientation.roll; 
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  setupOrientation();
  Serial.println("Orientation system initialized!");
}

void loop() {
  updateOrientation();

  Serial.print("Yaw: ");   
  Serial.print(getYaw());

  Serial.print("Pitch: "); 
  Serial.print(getPitch());
  
  Serial.print("Roll: ");  
  Serial.println(getRoll());

  delay(100);  // bole buang kalau tidamau ada stop stop update tapi dia sentiasa update la kalau teda delay
}
