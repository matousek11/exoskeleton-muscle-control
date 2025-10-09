#include "MotorShieldValve.h"

#include "Adafruit_MotorShield.h"

MotorShieldValve::MotorShieldValve(int motorNumber, ValveType type, Adafruit_MotorShield* motorShield,
                                   uint8_t addrOfMotorShield)
    : IValve(motorNumber, type) {
  this->addrOfMotorShield = addrOfMotorShield;

  valve = motorShield->getMotor(motorNumber);
  valve->setSpeed(255);

  Serial.print(getType() + " initialiazed, motor number:" + this->controlPin + " on board with address: ");
  Serial.println(addrOfMotorShield, HEX);
}

void MotorShieldValve::open() {
  valve->run(FORWARD);
  valveIsOpen = true;

  Serial.print("PHYSICAL MOVEMENT: Opening " + getType() + ", motor: " + controlPin + " on board with address: ");
  Serial.println(addrOfMotorShield, HEX);
}

void MotorShieldValve::close() {
  valve->run(RELEASE);
  valveIsOpen = false;

  Serial.print("PHYSICAL MOVEMENT: Closing " + getType() + ", motor: " + controlPin + " on board with address: ");
  Serial.println(addrOfMotorShield, HEX);
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

int MotorShieldValve::getValvePin() const {
  return this->controlPin;
}