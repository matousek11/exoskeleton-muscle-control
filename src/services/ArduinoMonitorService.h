#ifndef ARDUINOMONITORSERVICE_H_
#define ARDUINOMONITORSERVICE_H_

#include "./../control-algorithms/AntagonisticPIDControlAlgorithm.h"
#include "./../control-algorithms/ThreeDOFAntagonisticParticularMuscleControlAlgorithm.h"
#include "./../control-algorithms/TwoDOFAntagonisticPIDControlAlgorithm.h"
#include "./../control-algorithms/TwoDOFAntagonisticParticularMuscleControlAlgorithm.h"
#include "./../interfaces/IControlAlgorithm.h"
#include "./../models/Actuator.h"
#include "./../models/Gyroscope.h"
#include "./../models/Muscle.h"
#include "Arduino.h"

/**
 * @brief Structure holding all system components for monitor control
 */
struct SystemComponents {
  Muscle* muscle;
  Gyroscope* upperGyroscope;
  Gyroscope* gyroscope;
  IControlAlgorithm* controlAlgorithm;
  AntagonisticPIDControlAlgorithm* antagonisticControlAlgorithm;
  TwoDOFAntagonisticPIDControlAlgorithm* twoDOFAntagonisticControlAlgorithm;
  TwoDOFAntagonisticParticularMuscleControlAlgorithm* twoDOFAntagonisticParticularMuscleControlAlgorithm;
  ThreeDOFAntagonisticParticularMuscleControlAlgorithm* threeDOFAntagonisticParticularMuscleControlAlgorithm;

  Actuator* frontActuator;
  Actuator* backActuator;
  Actuator* leftActuator;
  Actuator* rightActuator;
  Actuator* topFrontActuator;
  Actuator* topBackActuator;
  Actuator* leftFrontActuator;
  Actuator* rightFrontActuator;
  Actuator* leftBackActuator;
  Actuator* rightBackActuator;
};

/**
 * @brief Houses everything that's used for control of app through monitor.
 *
 * Used for start up of app when controlled through monitor.
 */
class ArduinoMonitorService {
 private:
  /**
   * @brief Clears the serial monitor
   *
   * Clears the serial monitor by printing new lines as monitor clear commands does not exists
   */
  void clearSerialMonitor();

 public:
  /**
   * @brief Used for control of software through monitor terminal.
   *
   * Serves as entry point for app when controlled through monitor terminal.
   *
   * @param systemComponents System components structure containing all actuators, algorithms, and sensors
   */
  void controlThroughMonitor(const SystemComponents& systemComponents);

  /**
   * @brief Prints help with commands for user into monitor.
   *
   * @param inputCommand command that was used
   * @param unknownCommand whether unknown command was used
   */
  void printPossibleCommands(String* inputCommand = nullptr, bool unknownCommand = false);

  /**
   * @brief Close all valves in the system.
   */
  void closeAllValves(const SystemComponents& systemComponents);
};

#endif