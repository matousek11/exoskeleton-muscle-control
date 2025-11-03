#ifndef GYROSCOPE_H_
#define GYROSCOPE_H_

#include <stdint.h>

#include "I2Cdev.h"
#include "MPU6050.h"

/**
 * Wrapper class around MPU6050 that makes reading of angles easier.
 */
class Gyroscope {
 private:
  MPU6050* mpu;
  uint8_t addrOfMPU6050;

  /**
   * Raw X axis angle calculated from MPU6050 without any offset.
   */
  float angleX = 0;

  /**
   * Raw Y axis angle calculated from MPU6050 without any offset.
   */
  float angleY = 0;

  /**
   * Raw Z axis angle calculated from MPU6050 without any offset.
   */
  float angleZ = 0;

  /**
   * @brief Angle by which is angle for axis X subtracted.
   *
   * Used for setting of referential angle where angle is 0 degrees.
   */
  float referenceAngleX = 0;

  /**
   * @brief Angle by which is angle for axis Y subtracted.
   *
   * Used for setting of referential angle where angle is 0 degrees.
   */
  float referenceAngleY = 0;

  /**
   * @brief Angle by which is angle for axis Z subtracted.
   *
   * Used for setting of referential angle where angle is 0 degrees.
   */
  float referenceAngleZ = 0;

  /**
   * Used to log time of start of measurement.
   */
  unsigned long timer;

  /**
   * Prepares gyroscope for work.
   */
  void initialize();

 public:
  Gyroscope(uint8_t addrOfMPU6050);

  /**
   * @brief Update angles based on newest data from sensors.
   */
  void updateValues();

  /**
   * Print angles that are currently saved in attributes.
   */
  void printValues();

  /**
   * @param invert Whether to flip the sign
   * @return Current angle on X axis corrected by reference set up through calibration.
   */
  float getXAngle(bool invert = false);

  /**
   * @param invert Whether to flip the sign
   * @return Current angle on Y axis corrected by reference set up through calibration.
   */
  float getYAngle(bool invert = false);

  /**
   * @param invert Whether to flip the sign
   * @return Current angle on Z axis corrected by reference set up through calibration.
   */
  float getZAngle(bool invert = false);

  /**
   * @brief Calibrates the X-axis by setting the current orientation as the new zero reference.
   *
   * After calling this method, the current X-axis angle is considered 0°,
   * and all subsequent angle measurements obtained through getters will be relative to this reference.
   */
  void calibrateXAngle();

  /**
   * @brief Calibrates the Y-axis by setting the current orientation as the new zero reference.
   *
   * After calling this method, the current Y-axis angle is considered 0°,
   * and all subsequent angle measurements obtained through getters will be relative to this reference.
   */
  void calibrateYAngle();

  /**
   * @brief Calibrates the Z-axis by setting the current orientation as the new zero reference.
   *
   * After calling this method, the current Z-axis angle is considered 0°,
   * and all subsequent angle measurements obtained through getters will be relative to this reference.
   */
  void calibrateZAngle();
};

#endif