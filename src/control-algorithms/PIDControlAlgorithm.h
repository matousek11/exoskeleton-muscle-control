#ifndef PIDCONTROLALGORITHM_H_
#define PIDCONTROLALGORITHM_H_

#include "./../interfaces/IControlAlgorithm.h"

/**
 * Implements default PID algorithm used to control system and get it into desired state
 */
class PIDControlAlgorithm : public IControlAlgorithm {
 public:
  PIDControlAlgorithm();

  /**
   * When called it will takeover for ms of control time and try to get system to desired state before end of control
   * time.
   *
   * @param muscle used as an actuator to get system to desired state
   * @param gyroscope used as sensor to create feedback loop
   * @param controlTime time in ms that algorithm has to get to desired state before it returns control to caller
   */
  void controlMuscle(Muscle* muscle, Gyroscope* gyroscope, int controlTime) override;
};

#endif