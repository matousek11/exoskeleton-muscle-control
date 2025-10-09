#ifndef VALVEFACTORY_H_
#define VALVEFACTORY_H_

#include "./../enums/ValveType.h"
#include "./../interfaces/IValve.h"
#include "./../models/MotorShieldValve.h"
#include "./../models/Valve.h"
#include "Adafruit_MotorShield.h"

class ValveFactory {
 private:
  static constexpr uint8_t NUM_SHIELDS = 3;
  int smallestEmptyShieldIndex = 0;
  uint8_t addresses[NUM_SHIELDS];
  Adafruit_MotorShield* shields[NUM_SHIELDS];

  void initShieldMemory();
  int findAddressIndex(uint8_t addr);
  Adafruit_MotorShield* getShield(uint8_t addr);

 public:
  ValveFactory();
  IValve* createValve(int controlPin, ValveType valveType);
  IValve* createValve(int controlPin, ValveType valveType, uint8_t addr);
};

#endif