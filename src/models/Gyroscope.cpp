#include "Gyroscope.h"

#include "I2Cdev.h"
#include "MPU6050.h"

unsigned long microsPerReading, microsPrevious;
#define FILTER_RATE 60

Gyroscope::Gyroscope(uint8_t addrOfMPU6050) {
  this->addrOfMPU6050 = addrOfMPU6050;
  this->mpu = new MPU6050(addrOfMPU6050);
  this->filter = new Madgwick();

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

  mpu->setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu->setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  filter->begin(FILTER_RATE);
  microsPerReading = 1000000 / FILTER_RATE;
  microsPrevious = micros();

  Serial.println("Aligning gyroscope angle with physical device...");
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    updateValues(nullptr);
  }
  Serial.println("Gyroscope aligned");
}

void Gyroscope::updateValues(Gyroscope* referenceGyroscope) {
  unsigned long lastTime = 0;
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

void Gyroscope::printValues(Gyroscope* referenceGyroscope) {
  Serial.print("Angle X: ");
  Serial.print(getXAngle(false, referenceGyroscope));
  Serial.print(", Angle Y: ");
  Serial.print(getYAngle(true, referenceGyroscope));
  Serial.print(", Angle Z: ");
  Serial.println(getZAngle(false, referenceGyroscope));
}

void Gyroscope::printValues(int length, Gyroscope* referenceGyroscope) {
  Serial.println("Show gyroscope output");

  while (true) {
    // stop command
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'c') {
        Serial.println("--- Stop ---");
        break;
      }
    }

    updateValues(referenceGyroscope);
    printValues(referenceGyroscope);
  }
}

float Gyroscope::getXAngle(bool invert, Gyroscope* referenceGyroscope) {
  float value = !invert ? angleX : -angleX;
  float referencedFixAngle = value - referenceAngleX;

  // Serial.println(referencedFixAngle);
  if (referenceGyroscope == nullptr) {
    if (referencedFixAngle > 180) {
      return referencedFixAngle - 360;
    }
    return referencedFixAngle;
  }

  float referenceValue = referenceGyroscope->getXAngle();
  return referencedFixAngle - referenceValue;
}

float Gyroscope::getYAngle(bool invert, Gyroscope* referenceGyroscope) {
  float value = !invert ? angleY : -angleY;
  float referencedFixAngle = value - referenceAngleY;

  if (referenceGyroscope == nullptr) {
    return referencedFixAngle;
  }

  float referenceValue = referenceGyroscope->getYAngle(true);
  // Serial.println(value);
  // Serial.println(referenceValue);
  return referencedFixAngle - referenceValue;
  // return value - referenceAngleY;
}

float Gyroscope::getZAngle(bool invert, Gyroscope* referenceGyroscope) {
  float value = !invert ? angleZ : -angleZ;
  float referencedFixAngle = value - referenceAngleZ;

  if (referenceGyroscope == nullptr) {
    return referencedFixAngle;
  }

  return referencedFixAngle - referenceGyroscope->getZAngle();
  // return value - referenceAngleZ;
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