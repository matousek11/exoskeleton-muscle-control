#ifndef VALVE_H_
#define VALVE_H_

#include "../enums/ValveType.h"
#include "./../interfaces/IValve.h"
#include "Arduino.h"

class Valve : public IValve {
 public:
  Valve(int controlPin, ValveType type);

  void open() override;
  void close() override;
  bool isOpen() const override;
  String getStatus() const override;
  String getType() const override;
  int getValvePin() const override;
};

#endif