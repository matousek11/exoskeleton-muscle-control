#ifndef ARDUINOMONITORSERVICE_H_
#define ARDUINOMONITORSERVICE_H_

#include "./../models/Gyroscope.h"
#include "./../models/Muscle.h"
#include "Arduino.h"

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
   * @param muscle muscle which would be controlled through monitor
   * @param gyroscope gyroscope which would be controlled through monitor
   */
  void controlThroughMonitor(Muscle* muscle, Gyroscope* gyroscope);

  /**
   * @brief Prints help with commands for user into monitor.
   *
   * @param inputCommand command that was used
   * @param unknownCommand whether unknown command was used
   */
  void printPossibleCommands(String* inputCommand = nullptr, bool unknownCommand = false);
};

#endif