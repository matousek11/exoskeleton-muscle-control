#ifndef PIDCONTROLALGORITHM_H_
#define PIDCONTROLALGORITHM_H_

#include "./../interfaces/IControlAlgorithm.h"
#include "./../models/ControlTarget.h"
#include "./../models/SpecificControlTarget.h"

/**
 * Implements default PID algorithm used to control system and get it into desired state
 */
class PIDControlAlgorithm : public IControlAlgorithm {
 private:
  SpecificControlTarget** specificControlTargets = nullptr;
  size_t numberOfTargets = 0;

  /**
   * Prepares specific control targets from relative control targets
   */
  void setSpecificControlTargets(ControlTarget targets[], size_t numberOfControlTargets, float startTime,
                                 float controlTime);

  /**
   * Looks into targets and return current active target angle based on time since start of control algorithm.
   */
  float getTargetAngle();

 public:
  PIDControlAlgorithm();
  virtual ~PIDControlAlgorithm();

  /**
   * When called it will takeover for ms of control time var and try to get system to desired state before end of
   * control time.
   *
   * @param muscle used as an actuator to get system to desired state
   * @param gyroscope used as sensor to create feedback loop
   * @param controlTime time in ms that algorithm has to get to desired state before it returns control to caller
   * @param targets consists of angle targets that should algorithm go to in order
   * @param number_of_targets number of angle targets in array
   */
  void controlMuscle(Muscle* muscle, Gyroscope* gyroscope, int controlTime, ControlTarget targets[],
                     size_t number_of_targets) override;

  void deleteTargets();
};

#endif