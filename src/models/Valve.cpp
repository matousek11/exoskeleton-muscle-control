#include "Valve.h"

#include "../enums/ValveType.h"
#include "../interfaces/IValve.h"
#include "Arduino.h"

Valve::Valve(int controlPin, ValveType type) : IValve(controlPin, type) {
  pinMode(this->controlPin, OUTPUT);
  digitalWrite(this->controlPin, LOW);
  Serial.print(F("Valve initialized: "));
  Serial.print(getTypeChar());
  Serial.print(F(", pin: "));
  Serial.println(this->controlPin);
}

void Valve::open() {
  digitalWrite(controlPin, HIGH);
  // Avoid String concatenation to prevent heap fragmentation
  // Serial.print(F("PHYSICAL MOVEMENT: Opening "));
  // Serial.print(getTypeChar());
  // Serial.print(F(", pin: "));
  // Serial.println(controlPin);
}

void Valve::close() {
  digitalWrite(controlPin, LOW);
  // Avoid String concatenation to prevent heap fragmentation
  // Serial.print(F("PHYSICAL MOVEMENT: Closing "));
  // Serial.print(getTypeChar());
  // Serial.print(F(", pin: "));
  // Serial.println(controlPin);
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

const __FlashStringHelper* Valve::getTypeChar() const {
  return this->type == ValveType::INLET ? F("Inlet valve") : F("Outlet valve");
}

int Valve::getValvePin() const {
  return this->controlPin;
}
