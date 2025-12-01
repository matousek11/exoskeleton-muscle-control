#ifndef TWODOFANTAGONISTICPARTICULARMUSCLECONTROLALGORITHM_H_
#define TWODOFANTAGONISTICPARTICULARMUSCLECONTROLALGORITHM_H_

#include "./../models/Actuator.h"
#include "./../models/ControlTarget.h"
#include "./../models/Gyroscope.h"
#include "./../models/SpecificControlTarget.h"

/**
 * Implements 2 DOF antagonistic PID algorithm used to control system and get it into desired state, actuators opposite
 * to each other have to cooperate in order to stay in tension and at target angle.
 */
class TwoDOFAntagonisticParticularMuscleControlAlgorithm {
 private:
  SpecificControlTarget** specificControlTargets = nullptr;
  size_t numberOfTargets = 0;

  /**
   * Prepares specific control targets from relative control targets
   */
  void setSpecificControlTargets(ControlTarget targets[], size_t numberOfControlTargets, float startTime,
                                 float controlTime);

  /**
   * Looks into targets and return current active lateral target angle based on time since start of control algorithm.
   */
  float getLateralTargetAngle();

  /**
   * Looks into targets and return current active longitudinal target angle based on time since start of control
   * algorithm.
   */
  float getLongitudinalTargetAngle();

 public:
  TwoDOFAntagonisticParticularMuscleControlAlgorithm();
  virtual ~TwoDOFAntagonisticParticularMuscleControlAlgorithm();

  /**
   * When called it will takeover for ms of control time var and try to get system to desired state before end of
   * control time.
   *
   * @param leftFrontActuator used to push system more forward and left into desired state or keep tension
   * @param rightFrontActuator used to push system more forward and right into desired state or keep tension
   * @param leftBackActuator used to push system more backward and left into desired state or keep tension
   * @param rightBackActuator used to push system more backward and right into desired state or keep tension
   * @param gyroscope used as sensor to create feedback loop
   * @param controlTime time in ms that algorithm has to get to desired state before it returns control to caller
   * @param targets consists of angle targets that should algorithm go to in order
   * @param number_of_targets number of angle targets in array
   */
  void controlMuscle(Actuator* leftFrontActuator, Actuator* rightFrontActuator, Actuator* leftBackActuator,
                     Actuator* rightBackActuator, Gyroscope* gyroscope, int controlTime, ControlTarget targets[],
                     size_t number_of_targets);

  void deleteTargets();
};

#endif