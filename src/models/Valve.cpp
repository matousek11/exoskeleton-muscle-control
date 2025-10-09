#include "Valve.h"

#include "../enums/ValveType.h"
#include "../interfaces/IValve.h"
#include "Arduino.h"

Valve::Valve(int controlPin, ValveType type) : IValve(controlPin, type) {
  pinMode(this->controlPin, OUTPUT);
  digitalWrite(this->controlPin, LOW);
  Serial.println(getType() + " initialiazed, pin:" + this->controlPin);
}

void Valve::open() {
  digitalWrite(controlPin, HIGH);
  Serial.print("PHYSICAL MOVEMENT: Opening " + getType() + ", pin: ");
  Serial.println(controlPin);
}

void Valve::close() {
  digitalWrite(controlPin, LOW);
  Serial.print("PHYSICAL MOVEMENT: Closing " + getType() + ", pin: ");
  Serial.println(controlPin);
}

bool Valve::isOpen() const {
  return digitalRead(controlPin) == HIGH;
}

String Valve::getStatus() const {
  return String("status: ") + getType() + (isOpen() ? " open" : " closed");
}

String Valve::getType() const {
  return this->type == ValveType::INLET ? "Inlet valve" : "Outlet valve";
}

int Valve::getValvePin() const {
  return this->controlPin;
}
