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

  /**
   * Physically opens valve
   */
  virtual void open() = 0;

  /**
   * Physically closes valve
   */
  virtual void close() = 0;
  virtual bool isOpen() const = 0;

  /**
   * Returns status of valve ready for printing.
   */
  virtual String getStatus() const = 0;

  /**
   * Returns whether valve is inlet or outlet and is ready for printing.
   */
  virtual String getType() const = 0;
  virtual int getValvePin() const = 0;
};

#endif
