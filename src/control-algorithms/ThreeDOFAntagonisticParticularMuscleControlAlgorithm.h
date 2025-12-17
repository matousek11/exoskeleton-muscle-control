#ifndef THREEDOFANTAGONISTICPARTICULARMUSCLECONTROLALGORITHM_H_
#define THREEDOFANTAGONISTICPARTICULARMUSCLECONTROLALGORITHM_H_

#include "./../models/Actuator.h"
#include "./../models/StateMachineActuator.h"
#include "./../models/ControlTarget.h"
#include "./../models/Gyroscope.h"
#include "./../models/SpecificControlTarget.h"

/**
 * Implements 3 DOF antagonistic PID algorithm used to control system and get it into desired state, actuators opposite
 * to each other have to cooperate in order to stay in tension and at target angle.
 */
class ThreeDOFAntagonisticParticularMuscleControlAlgorithm {
 private:
  SpecificControlTarget** specificLowerLegControlTargets = nullptr;
  size_t numberOfLowerLegTargets = 0;
  SpecificControlTarget** specificUpperLegControlTargets = nullptr;
  size_t numberOfUpperLegTargets = 0;

  /**
   * Prepares specific control targets from relative control targets
   */
  void setSpecificControlTargets(ControlTarget targets[], size_t numberOfControlTargets, float startTime,
                                 unsigned long controlTime, bool isLowerLegTargets = true);

  /**
   * Looks into targets and return current active lateral target angle for lower leg based on time since start of
   * control algorithm.
   */
  float getLowerLegLateralTargetAngle();

  /**
   * Looks into targets and return current active longitudinal target angle for lower leg based on time since start of
   * control algorithm.
   */
  float getLowerLegLongitudinalTargetAngle();

  /**
   * Looks into targets and return current active lateral target angle for upper leg based on time since start of
   * control algorithm.
   */
  float getUpperLegLateralTargetAngle();

 public:
  ThreeDOFAntagonisticParticularMuscleControlAlgorithm();
  virtual ~ThreeDOFAntagonisticParticularMuscleControlAlgorithm();

  /**
   * When called it will takeover for ms of control time var and try to get system to desired state before end of
   * control time.
   *
   * @param leftBottomFrontActuator used to push system more forward and left into desired state or keep tension in
   * lower leg
   * @param rightBottomFrontActuator used to push system more forward and right into desired state or keep tension in
   * lower leg
   * @param leftBottomBackActuator used to push system more backward and left into desired state or keep tension in
   * lower leg
   * @param rightBottomBackActuator used to push system more backward and right into desired state or keep tension in
   * lower leg
   * @param topFrontActuator used to push system more forward in upper part of leg
   * @param topBackActuator used to push system more backward in upper part of leg
   * @param lowerGyroscope used as sensor to create feedback loop for lower part of leg (ankle)
   * @param upperGyroscope used as sensor to create feedback loop for upper part of leg (knee)
   * @param controlTime time in ms that algorithm has to get to desired state before it returns control to caller
   * @param lowerLegTargets consists of lower leg angle targets that should algorithm go to in order
   * @param numberOfLowerLegTargets number of lower leg angle targets in array
   * @param upperLegTargets consists of upper leg angle targets that should algorithm go to in order
   * @param numberOfUpperLegTargets number of upper leg angle targets in array
   */
  void controlMuscle(StateMachineActuator* leftBottomFrontActuator, StateMachineActuator* rightBottomFrontActuator,
                     StateMachineActuator* leftBottomBackActuator, StateMachineActuator* rightBottomBackActuator,
                     StateMachineActuator* topBottomFrontActuator, StateMachineActuator* topBackActuator, Gyroscope* lowerGyroscope,
                     Gyroscope* upperGyroscope, unsigned long controlTime, ControlTarget lowerLegTargets[],
                     size_t numberOfLowerLegTargets, ControlTarget upperLegTargets[], size_t numberOfUpperLegTargets);

  void deleteTargets();
};

#endif