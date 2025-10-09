#ifndef IVALVE_H_
#define IVALVE_H_

#include "./../enums/ValveType.h"
#include "Arduino.h"

class IValve {
 protected:
  int controlPin;
  ValveType type;

  IValve(int controlPin, ValveType type) : controlPin(controlPin), type(type) {};

 public:
  virtual ~IValve() {};

  virtual void open() = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;
  virtual String getStatus() const = 0;
  virtual String getType() const = 0;
  virtual int getValvePin() const = 0;
};

#endif
