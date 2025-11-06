#ifndef ANTAGONISTICPIDCONTROLALGORITHM_H_
#define ANTAGONISTICPIDCONTROLALGORITHM_H_

#include "./../models/Actuator.h"
#include "./../models/ControlTarget.h"
#include "./../models/Gyroscope.h"
#include "./../models/SpecificControlTarget.h"

/**
 * Implements antagonistic PID algorithm used to control system and get it into desired state, actuators opposite to
 * each other have to cooperate in order to stay in tension and at target angle.
 */
class AntagonisticPIDControlAlgorithm {
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
  AntagonisticPIDControlAlgorithm();
  virtual ~AntagonisticPIDControlAlgorithm();

  /**
   * When called it will takeover for ms of control time var and try to get system to desired state before end of
   * control time.
   *
   * @param forwardActuator used to push system more forward into desired state or keep tension
   * @param backwardActuator used to push system more backward into desired state or keep tension
   * @param gyroscope used as sensor to create feedback loop
   * @param controlTime time in ms that algorithm has to get to desired state before it returns control to caller
   * @param targets consists of angle targets that should algorithm go to in order
   * @param number_of_targets number of angle targets in array
   */
  void controlMuscle(Actuator* forwardActuator, Actuator* backwardActuator, Gyroscope* gyroscope, int controlTime,
                     ControlTarget targets[], size_t number_of_targets);

  void deleteTargets();
};

#endif