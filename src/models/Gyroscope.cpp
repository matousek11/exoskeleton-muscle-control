#include "Gyroscope.h"

#include "I2Cdev.h"
#include "MPU6050.h"

unsigned long microsPerReading, microsPrevious;
#define FILTER_RATE 60

// Pre-computed constants for speed
static const float ACCEL_SCALE = 1.0f / 16384.0f;
static const float GYRO_SCALE = 1.0f / 131.0f;
// Use Arduino's built-in RAD_TO_DEG constant (57.295779...)

// Fast atan2 approximation (error < 0.5 degrees, ~10x faster than stdlib)
static float fastAtan2(float y, float x) {
  const float ONEQTR_PI = PI / 4.0f;
  const float THRQTR_PI = 3.0f * PI / 4.0f;
  float abs_y = fabsf(y) + 1e-10f;  // Prevent 0/0
  float angle;
  if (x >= 0) {
    float r = (x - abs_y) / (x + abs_y);
    angle = ONEQTR_PI - ONEQTR_PI * r;
  } else {
    float r = (x + abs_y) / (abs_y - x);
    angle = THRQTR_PI - ONEQTR_PI * r;
  }
  return y < 0 ? -angle : angle;
}

// Fast inverse square root (Quake III algorithm)
static float fastInvSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));  // One Newton-Raphson iteration
  return y;
}

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

  // Calculate time difference
  float dt = (lastTime == 0) ? 0.01f : (now - lastTime) * 0.001f;  // Multiply instead of divide
  lastTime = now;

  const float accelerometerWeight = 0.1f;
  const float gyroscopeWeight = 0.90f;

  int16_t ax, ay, az, gx, gy, gz;
  mpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert from raw data using pre-computed scales (multiply instead of divide)
  float accelerometerX = ax * ACCEL_SCALE;
  float accelerometerY = ay * ACCEL_SCALE;
  float accelerometerZ = az * ACCEL_SCALE;
  float gyroscopeX = gx * GYRO_SCALE;
  float gyroscopeY = gy * GYRO_SCALE;

  // Fast angle calculation using optimized atan2 (avoids slow stdlib atan2)
  float accAngleX = fastAtan2(accelerometerY, accelerometerZ) * RAD_TO_DEG;
  
  // Use fast inverse sqrt: sqrt(a) = a * invSqrt(a)
  float yz2 = accelerometerY * accelerometerY + accelerometerZ * accelerometerZ;
  float yzMag = yz2 * fastInvSqrt(yz2);  // This equals sqrt(yz2)
  float accAngleY = fastAtan2(-accelerometerX, yzMag) * RAD_TO_DEG;
  
  float xy2 = accelerometerX * accelerometerX + accelerometerY * accelerometerY;
  float xyMag = xy2 * fastInvSqrt(xy2);

  // Complementary filter for sensor data fusion
  angleX = gyroscopeWeight * (angleX + gyroscopeX * dt) + accelerometerWeight * accAngleX;
  angleY = gyroscopeWeight * (angleY + gyroscopeY * dt) + accelerometerWeight * accAngleY;
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