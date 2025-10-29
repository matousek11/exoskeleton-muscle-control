#ifndef VALVEFACTORY_H_
#define VALVEFACTORY_H_

#include "./../enums/ValveType.h"
#include "./../interfaces/IValve.h"
#include "./../models/MotorShieldValve.h"
#include "./../models/Valve.h"
#include "Adafruit_MotorShield.h"

/**
 * Used as an abstraction for creation of valves as they can be connected directly or through motor board. Different
 * valve with same object is then returned based on connection type.
 */
class ValveFactory {
 private:
  /**
   * Express maximum number of Adafruit motor shields that can be connected.
   */
  static constexpr uint8_t NUM_SHIELDS = 3;
  int smallestEmptyShieldIndex = 0;

  /**
   * Place of addresses of already initialized Adafruit motor shields.
   */
  uint8_t addresses[NUM_SHIELDS];

  /**
   * Initialized objects of already prepared Adafruit motor shields.
   */
  Adafruit_MotorShield* shields[NUM_SHIELDS];

  /**
   * Prepares memory of Adafruit motor shield valve connections.
   */
  void initShieldMemory();

  /**
   * Checks whether Adafruit motor shield on selected address is already initialized and loaded.
   * @param addr Address of Adafruit motor shield (0x60)
   */
  int findAddressIndex(uint8_t addr);

  /**
   * @param addr Address of Adafruit motor shield (0x60)
   */
  Adafruit_MotorShield* getShield(uint8_t addr);

 public:
  ValveFactory();

  /**
   * Returns valve that is connected directly through black shield.
   * @param controlPin Number of the pin that is connected to valve
   * @param valveType Whether valve is inlet or outlet
   */
  IValve* createValve(int controlPin, ValveType valveType);

  /**
   * Returns valve that is connected through Adafruit motor shields.
   * @param controlPin Number of the motor pins that are connected to valve
   * @param valveType Whether valve is inlet or outlet
   * @param addr Address of Adafruit motor shield (0x60)
   */
  IValve* createValve(int controlPin, ValveType valveType, uint8_t addr);
};

#endif