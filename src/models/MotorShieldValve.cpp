#include "MotorShieldValve.h"

#include "Adafruit_MotorShield.h"

MotorShieldValve::MotorShieldValve(int motorNumber, ValveType type, Adafruit_MotorShield* motorShield,
                                   uint8_t addrOfMotorShield)
    : IValve(motorNumber, type) {
  this->addrOfMotorShield = addrOfMotorShield;

  valve = motorShield->getMotor(motorNumber);
  if (!valve) {
    Serial.print(F("Failed to get motor."));
    while (1);
  }
  valve->setSpeed(255);

  // Avoid String concatenation to prevent heap fragmentation
  Serial.print(getTypeChar());
  Serial.print(F(" initialized, motor number: "));
  Serial.print(this->controlPin);
  Serial.print(F(" on board with address: "));
  Serial.println(addrOfMotorShield, HEX);
}

void MotorShieldValve::open() {
  valve->run(FORWARD);
  valveIsOpen = true;

  // Avoid String concatenation to prevent heap fragmentation
  // Serial.print(F("PHYSICAL MOVEMENT: Opening "));
  // Serial.print(getTypeChar());
  // Serial.print(F(", motor: "));
  // Serial.print(controlPin);
  // Serial.print(F(" on board with address: "));
  // Serial.println(addrOfMotorShield, HEX);
}

void MotorShieldValve::close() {
  valve->run(RELEASE);
  valveIsOpen = false;

  // Avoid String concatenation to prevent heap fragmentation
  // Serial.print(F("PHYSICAL MOVEMENT: Closing "));
  // Serial.print(getTypeChar());
  // Serial.print(F(", motor: "));
  // Serial.print(controlPin);
  // Serial.print(F(" on board with address: "));
  // Serial.println(addrOfMotorShield, HEX);
}

bool MotorShieldValve::isOpen() const {
  return valveIsOpen;
}

String MotorShieldValve::getStatus() const {
  return String("status: ") + getType() + (isOpen() ? " open" : " closed");
}

String MotorShieldValve::getType() const {
  return this->type == ValveType::INLET ? "Inlet valve" : "Outlet valve";
}

const __FlashStringHelper* MotorShieldValve::getTypeChar() const {
  return this->type == ValveType::INLET ? F("Inlet valve") : F("Outlet valve");
}

int MotorShieldValve::getValvePin() const {
  return this->controlPin;
}