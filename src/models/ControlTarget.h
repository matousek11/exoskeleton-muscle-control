#ifndef CONTROLTARGET_H_
#define CONTROLTARGET_H_

#include "Arduino.h"
#include "SpecificControlTarget.h"

/**
 * Data class for setting up target angle of system that activates after relative time point since start of algorithm.
 */
class ControlTarget {
 private:
  /**
   * Normalized time [0,1] after which the target angle becomes active.
   */
  float activationPoint;

  /**
   * Lateral target angle that system tries to reach after activation.
   */
  float lateralTargetAngle;

  /**
   * Longitudinal target angle that system tries to reach after activation.
   */
  float longitudinalTargetAngle;

 public:
  explicit ControlTarget(float activationPoint, float lateralTargetAngle, float longitudinalTargetAngle)
      : activationPoint(activationPoint)
      , lateralTargetAngle(lateralTargetAngle)
      , longitudinalTargetAngle(longitudinalTargetAngle) {
    if (activationPoint < 0.0f || activationPoint > 1.0f) {
      Serial.println("ERROR: activationPoint must be between 0 and 1");
      while (true);
    }
  }

  /**
   * Returns SpecificControlTarget with time activation point based on normalized time and control algorithm running
   * length.
   */
  SpecificControlTarget* calculateSpecificControlTarget(float startTime, float controlTime) {
    float absoluteActivationPoint = startTime + (controlTime * getActivationPoint());
    return new SpecificControlTarget(absoluteActivationPoint, getLateralTargetAngle(), getLongitudinalTargetAngle());
  }

  float getActivationPoint() const {
    return activationPoint;
  }

  float getLateralTargetAngle() const {
    return lateralTargetAngle;
  }

  float getLongitudinalTargetAngle() const {
    return longitudinalTargetAngle;
  }
};

#endif