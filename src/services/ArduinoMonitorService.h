#ifndef ARDUINOMONITORSERVICE_H_
#define ARDUINOMONITORSERVICE_H_

#include "./../models/Muscle.h"
#include "Arduino.h"

class ArduinoMonitorService {
 private:
  void clearSerialMonitor();

 public:
  void controlThroughMonitor(Muscle* muscle);
  void printPossibleCommands(String* inputCommand = nullptr,
                             bool unknownCommand = false);
};

#endif