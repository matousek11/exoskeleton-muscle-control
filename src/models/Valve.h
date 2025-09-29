#ifndef VALVE_H
#define VALVE_H

#include "Arduino.h"
#include "../enums/ValveType.h"

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
