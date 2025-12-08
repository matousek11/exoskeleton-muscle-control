#include "Gyroscope.h"

#include "I2Cdev.h"
#include "MPU6050.h"

// keeps angle in [-180, 180] to avoid wrap jumps
static float wrapTo180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

// chooses representation of target closest to reference (prevents 180° flips)
static float unwrapNearest(float target, float reference) {
  float wrapped = wrapTo180(target);
  float diff = wrapped - wrapTo180(reference);
  if (diff > 180.0f)
    wrapped -= 360.0f;
  else if (diff < -180.0f)
    wrapped += 360.0f;
  return wrapped;
}

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

  int16_t ax, ay, az, gx, gy, gz;
  mpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // convert from raw data to g and rad/s
  float accelerometerX = ax / 16384.0f;
  float accelerometerY = ay / 16384.0f;
  float accelerometerZ = az / 16384.0f;
  float gyroscopeX = (gx / 131.0f) * DEG_TO_RAD;
  float gyroscopeY = (gy / 131.0f) * DEG_TO_RAD;
  float gyroscopeZ = (gz / 131.0f) * DEG_TO_RAD;

  // normalize accelerometer (protect against divide by zero)
  float accNorm =
      sqrt(accelerometerX * accelerometerX + accelerometerY * accelerometerY + accelerometerZ * accelerometerZ);
  if (accNorm > 1e-6f) {
    accelerometerX /= accNorm;
    accelerometerY /= accNorm;
    accelerometerZ /= accNorm;
  }

  // estimated direction of gravity based on current quaternion
  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  // error between measured and estimated gravity (Mahony)
  float ex = (accelerometerY * vz - accelerometerZ * vy);
  float ey = (accelerometerZ * vx - accelerometerX * vz);
  float ez = (accelerometerX * vy - accelerometerY * vx);

  // apply proportional feedback
  gyroscopeX += twoKp * ex;
  gyroscopeY += twoKp * ey;
  gyroscopeZ += twoKp * ez;

  // integrate quaternion rate and normalize
  float qa = q0;
  float qb = q1;
  float qc = q2;
  float qd = q3;

  qa += (-qb * gyroscopeX - qc * gyroscopeY - qd * gyroscopeZ) * 0.5f * dt;
  qb += (qa * gyroscopeX + qc * gyroscopeZ - qd * gyroscopeY) * 0.5f * dt;
  qc += (qa * gyroscopeY - qb * gyroscopeZ + qd * gyroscopeX) * 0.5f * dt;
  qd += (qa * gyroscopeZ + qb * gyroscopeY - qc * gyroscopeX) * 0.5f * dt;

  // renormalize quaternion
  float recipNorm = 1.0f / sqrt(qa * qa + qb * qb + qc * qc + qd * qd);
  q0 = qa * recipNorm;
  q1 = qb * recipNorm;
  q2 = qc * recipNorm;
  q3 = qd * recipNorm;

  // convert quaternion to Euler angles (deg)
  // standard ZYX (yaw-pitch-roll) sequence, each in [-180, 180]
  float roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
  float pitch = atan2(2.0f * (q0 * q2 - q1 * q3), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
  float yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q3 * q3 + q0 * q0)) * RAD_TO_DEG;

  // unwrap to keep continuity over full 360°
  angleX = unwrapNearest(roll, angleX);
  angleY = unwrapNearest(pitch, angleY);
  angleZ = unwrapNearest(yaw, angleZ);
}

void Gyroscope::printValues(Gyroscope* gyroscope) {
  float baseX = 0.0f;
  float baseY = 0.0f;
  float baseZ = 0.0f;

  if (gyroscope != nullptr) {
    baseX = gyroscope->getXAngle();
    baseY = gyroscope->getYAngle();
    baseZ = gyroscope->getZAngle();
  }

  Serial.print("Angle X: ");
  Serial.print(getXAngle() - baseX);
  Serial.print(", Angle Y: ");
  Serial.print(getYAngle() - baseY);
  Serial.print(", Ref Y: ");
  Serial.print(baseY);
  Serial.print(", Angle Z: ");
  Serial.println(getZAngle() - baseZ);
}

void Gyroscope::printValues(int length, Gyroscope* gyroscope) {
  Serial.println("Show gyroscope output for " + String(length) + " seconds");
  unsigned long startTime = millis();

  if (length == 0) {
    while (true) {
      // stop command
      if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 'c') {
          Serial.println("--- Stop ---");
          break;
        }
      }

      updateValues();
      if (gyroscope != nullptr) {
        gyroscope->updateValues();
      }
      printValues(gyroscope);
    }
    return;
  }

  while (millis() - startTime < length * 1000) {
    updateValues();
    if (gyroscope != nullptr) {
      gyroscope->updateValues();
    }
    printValues(gyroscope);
  }
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