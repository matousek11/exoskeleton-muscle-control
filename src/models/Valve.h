#ifndef VALVE_H_
#define VALVE_H_

#include "../enums/ValveType.h"
#include "Arduino.h"

class Valve {
 private:
  int controlPin;
  ValveType type;

 public:
  Valve(int controlPin, ValveType type);

  void open();
  void close();
  bool isOpen();
  String getStatus();
  String getType();
};

#endif
