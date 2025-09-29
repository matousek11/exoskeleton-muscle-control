#include "Valve.h"
#include "Arduino.h"
#include "../enums/ValveType.h"

Valve::Valve(int pin, ValveType type) {
  this->controlPin = pin;
  this->type = type;

  pinMode(this->controlPin, OUTPUT);
  digitalWrite(this->controlPin, LOW);
  Serial.println(getType() + " initialiazed, pin:" + controlPin);
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

bool Valve::isOpen() {
  return digitalRead(controlPin) == HIGH;
}

String Valve::getStatus() {
  return String("status: ") + getType() + (isOpen() ? " open" : " closed");
}

String Valve::getType() {
  return this->type == ValveType::INLET ? "Inlet valve" : "Outlet valve";
}
