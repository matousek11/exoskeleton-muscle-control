#ifndef MOTORSHIELDVALVE_H_
#define MOTORSHIELDVALVE_H_

#include "./../interfaces/IValve.h"
#include "Adafruit_MotorShield.h"

class MotorShieldValve : public IValve {
 private:
  Adafruit_DCMotor* valve;
  bool valveIsOpen = false;
  uint8_t addrOfMotorShield;

 public:
  MotorShieldValve(int controlPin, ValveType type, Adafruit_MotorShield* motorShield, uint8_t addrOfMotorShield);

  void open() override;
  void close() override;
  bool isOpen() const override;
  String getStatus() const override;
  String getType() const override;
  int getValvePin() const override;
};

#endif