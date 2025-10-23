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
   * Target angle the system tries to reach after activation.
   */
  float targetAngle;

 public:
  explicit ControlTarget(float activationPoint, float targetAngle)
      : activationPoint(activationPoint), targetAngle(targetAngle) {
    if (activationPoint < 0.0f || activationPoint > 1.0f) {
      Serial.println("ERROR: activationPoint must be between 0 and 1");
      while (true);
    }
  }

  SpecificControlTarget* calculateSpecificControlTarget(float startTime, float controlTime) {
    float absoluteActivationPoint = startTime + (controlTime * getActivationPoint());
    return new SpecificControlTarget(absoluteActivationPoint, getTargetAngle());
  }

  float getActivationPoint() const {
    return activationPoint;
  }

  float getTargetAngle() const {
    return targetAngle;
  }
};

#endif