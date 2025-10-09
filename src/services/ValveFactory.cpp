#include "ValveFactory.h"

#include "./../enums/ValveType.h"
#include "./../models/MotorShieldValve.h"
#include "./../models/Valve.h"

ValveFactory::ValveFactory() {
  initShieldMemory();
}

IValve* ValveFactory::createValve(int controlPin, ValveType valveType) {
  return new Valve(controlPin, valveType);
}

IValve* ValveFactory::createValve(int controlPin, ValveType valveType, uint8_t addr) {
  return new MotorShieldValve(controlPin, valveType, getShield(addr), addr);
}

Adafruit_MotorShield* ValveFactory::getShield(uint8_t addr) {
  int index = findAddressIndex(addr);
  if (index != -1) {
    return shields[index];
  }

  shields[smallestEmptyShieldIndex] = new Adafruit_MotorShield(addr);
  addresses[smallestEmptyShieldIndex] = addr;
  if (!shields[smallestEmptyShieldIndex]->begin()) {
    Serial.print("Could not find Motor Shield on address 0x");
    Serial.print(addr, HEX);
    Serial.println(". Check wiring.");
    while (1);
  }

  Serial.print("Motor Shield on address 0x");
  Serial.print(addr, HEX);
  Serial.println(" initialized.");

  smallestEmptyShieldIndex += 1;

  return shields[smallestEmptyShieldIndex - 1];
}

int ValveFactory::findAddressIndex(uint8_t addr) {
  for (uint8_t i = 0; i < NUM_SHIELDS; i++) {
    if (addresses[i] == addr) {
      return i;
    }
  }
  return -1;
}

void ValveFactory::initShieldMemory() {
  for (uint8_t i = 0; i < NUM_SHIELDS; i++) {
    shields[i] = nullptr;
    addresses[i] = 0;
  }
}