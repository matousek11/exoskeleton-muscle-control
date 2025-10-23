#ifndef SPECIFICCONTROLTARGET_H_
#define SPECIFICCONTROLTARGET_H_

#include "Arduino.h"

/**
 * Data class for setting up target angle of system that activates after absolute time point since start of algorithm.
 * Should be recalculated from ControlTarget data class.
 */
class SpecificControlTarget {
 private:
  /**
   * Time after which target becomes active in ms.
   */
  float activationPointTime;

  /**
   * Target angle the system tries to reach after activation.
   */
  float targetAngle;

 public:
  explicit SpecificControlTarget(float activationPointTime, float targetAngle)
      : activationPointTime(activationPointTime), targetAngle(targetAngle) {
    if (activationPointTime < 0.0f) {
      Serial.println("ERROR: activationPointTime must not be smaller than 0");
      while (true);
    }
  }

  float getActivationPointTime() const {
    return activationPointTime;
  }

  float getTargetAngle() const {
    return targetAngle;
  }
};

#endif