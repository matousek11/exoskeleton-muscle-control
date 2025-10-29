#include "Gyroscope.h"

#include "I2Cdev.h"
#include "MPU6050.h"

Gyroscope::Gyroscope(uint8_t addrOfMPU6050) {
  this->addrOfMPU6050 = addrOfMPU6050;
  this->mpu = new MPU6050(addrOfMPU6050);

  initialize();
}

void Gyroscope::initialize() {
  Serial.print("Gyroscope startup on address 0x");
  Serial.println(addrOfMPU6050, HEX);
  mpu->initialize();

  if (!mpu->testConnection()) {
    Serial.println("Gyroscope (MPU6050) not connected!");

    while (1) {
      delay(1000);
    }
  }
  Serial.println("Gyroscope initialized");

  Serial.println("Aligning gyroscope angle with physical device...");
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    updateValues();
  }
  Serial.println("Gyroscope aligned");
}

void Gyroscope::updateValues() {
  static unsigned long lastTime = 0;
  unsigned long now = millis();

  // calculate time difference
  float dt = (lastTime == 0) ? 0.01f : (now - lastTime) / 1000.0f;
  lastTime = now;

  const float accelerometerWeight = 0.1f;
  const float gyroscopeWeight = 0.90f;

  int16_t ax, ay, az, gx, gy, gz;
  mpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // convert from raw data to g and deg/s
  float accelerometerX = ax / 16384.0f;
  float accelerometerY = ay / 16384.0f;
  float accelerometerZ = az / 16384.0f;
  float gyroscopeX = gx / 131.0f;
  float gyroscopeY = gy / 131.0f;
  float gyroscopeZ = gz / 131.0f;

  // angle from accelerometer (axis X)
  float accAngleX = atan2(accelerometerY, accelerometerZ) * 180.0f / PI;
  float accAngleY =
      atan2(-accelerometerX, sqrt(accelerometerY * accelerometerY + accelerometerZ * accelerometerZ)) * 180.0f / PI;
  float accAngleZ =
      atan2(sqrt(accelerometerX * accelerometerX + accelerometerY * accelerometerY), accelerometerZ) * 180.0f / PI;

  // complementary filters for sensor data fusion
  angleX = gyroscopeWeight * (angleX + gyroscopeX * dt) + accelerometerWeight * accAngleX;
  angleY = gyroscopeWeight * (angleY + gyroscopeY * dt) + accelerometerWeight * accAngleY;
  angleZ = gyroscopeWeight * (angleZ + gyroscopeZ * dt) + accelerometerWeight * accAngleZ;
}

void Gyroscope::printValues() {
  Serial.print("Angle X: ");
  Serial.print(getXAngle());
  Serial.print(", Angle Y: ");
  Serial.print(getYAngle());
  Serial.print(", Angle Z: ");
  Serial.println(getZAngle());
}

float Gyroscope::getXAngle(bool invert) {
  float value = !invert ? angleX : -angleX;
  return value - referenceAngleX;
}

float Gyroscope::getYAngle(bool invert) {
  float value = !invert ? angleY : -angleY;
  return value - referenceAngleY;
}

float Gyroscope::getZAngle(bool invert) {
  float value = !invert ? angleZ : -angleZ;
  return value - referenceAngleZ;
}

void Gyroscope::calibrateXAngle() {  // maybe it has a bug when inverted
  referenceAngleX = angleX;
}

void Gyroscope::calibrateYAngle() {  // maybe it has a bug when inverted
  referenceAngleY = angleY;
}

void Gyroscope::calibrateZAngle() {  // maybe it has a bug when inverted
  referenceAngleZ = angleZ;
}