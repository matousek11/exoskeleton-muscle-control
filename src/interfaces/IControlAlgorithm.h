#ifndef ICONTROLALGORITHM_H_
#define ICONTROLALGORITHM_H_

#include "./../models/ControlTarget.h"
#include "./../models/Gyroscope.h"
#include "./../models/Muscle.h"

/**
 * Interface used for implementation of control algorithms
 * used to achieve correct angle of joint through pneumatic muscle.
 */
class IControlAlgorithm {
 public:
  /**
   * Controls muscle through feedback loop control algorithm.
   * Feedback is provided by gyroscope angle.
   *
   * @param controlTime that control algorithm has before it gives control back to caller of the method.
   */
  virtual void controlMuscle(Muscle* muscle, Gyroscope* gyroscope, int controlTime, ControlTarget targets[],
                             size_t count) = 0;
};

#endif