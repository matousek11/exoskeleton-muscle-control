#ifndef SPECIFICCONTROLTARGET_H_
#define SPECIFICCONTROLTARGET_H_

#include "Arduino.h"

/**
 * Data class for setting up target angles of system that activates after absolute time point since start of algorithm.
 * Should be recalculated from ControlTarget data class.
 */
class SpecificControlTarget {
 private:
  /**
   * Time after which target becomes active in ms.
   */
  float activationPointTime;

  /**
   * Lateral target angle that system tries to reach after activation point.
   */
  float lateralTargetAngle;

  /**
   * Longitudal target angle that system tries to reach after activation point.
   */
  float longitudinalTargetAngle;

 public:
  explicit SpecificControlTarget(float activationPointTime, float lateralTargetAngle, float longitudinalTargetAngle)
      : activationPointTime(activationPointTime)
      , lateralTargetAngle(lateralTargetAngle)
      , longitudinalTargetAngle(longitudinalTargetAngle) {
    if (activationPointTime < 0.0f) {
      Serial.println("ERROR: activationPointTime must not be smaller than 0");
      while (true);
    }
  }

  float getActivationPointTime() const {
    return activationPointTime;
  }

  float getLateralTargetAngle() const {
    return lateralTargetAngle;
  }

  float getLongitudinalTargetAngle() const {
    return longitudinalTargetAngle;
  }
};

#endif