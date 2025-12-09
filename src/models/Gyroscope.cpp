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
  unsigned long microsNow = micros();
  if (referenceGyroscope != nullptr) {
    referenceGyroscope->updateValues(nullptr);
  }

  unsigned long now = millis();

  if (microsNow - microsPrevious >= microsPerReading) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;

    float gx_deg = gx / 131.0;
    float gy_deg = gy / 131.0;
    float gz_deg = gz / 131.0;

    filter->updateIMU(gx_deg, gy_deg, gz_deg, ax_g, ay_g, az_g);

    // Získání hodnot
    float rawX = filter->getRoll();
    angleY = filter->getPitch();

    // --- NORMALIZACE 0 až 360 (Vše v jednom kroku) ---

    // Osa X
    angleX = fmod(rawX, 360.0);
    if (angleX < 0) angleX += 360.0;

    // Osa Y
    // angleY = fmod(rawY, 360.0);
    // if (angleY < 0) angleY += 360.0;

    microsPrevious = microsPrevious + microsPerReading;
  }
}

void Gyroscope::printValues(Gyroscope* referenceGyroscope) {
  float baseX = 0.0f;
  float baseY = 0.0f;
  float baseZ = 0.0f;

  if (referenceGyroscope != nullptr) {
    baseX = referenceGyroscope->getXAngle(false, nullptr);
    baseY = referenceGyroscope->getYAngle(true, nullptr);
    baseZ = referenceGyroscope->getZAngle(false, nullptr);
  }

  Serial.print("Angle X: ");
  Serial.print(getXAngle(false, referenceGyroscope));
  Serial.print(", Plain X: ");
  Serial.print(getXAngle(false, nullptr));
  Serial.print(", Plain reference X: ");
  Serial.print(baseX);
  Serial.print(", Angle Y: ");
  Serial.print(getYAngle(true, nullptr));
  Serial.print(", Angle Z: ");
  Serial.println(getZAngle(false, nullptr));
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

  // 2. Normalization to 0-360 deg
  referencedFixAngle = fmod(referencedFixAngle, 360.0);
  if (referencedFixAngle < 0) {
    referencedFixAngle += 360.0;
  }

  if (referenceGyroscope == nullptr) {
    return referencedFixAngle;
  }

  float refAngle = referenceGyroscope->getXAngle(invert, nullptr);

  // Shortest angle path calculation
  // When angle is 300 deg and ref angle is 40 deg relative angle should be 100 deg.
  float diff = fabs(referencedFixAngle - refAngle);

  if (diff > 180.0) {
    diff = 360.0 - diff;  // If path is longer than half of circle go other way
  }

  return diff;
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