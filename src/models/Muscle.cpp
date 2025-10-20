#include "Muscle.h"

#include "../enums/ValveType.h"
#include "./../interfaces/IValve.h"
#include "Valve.h"

Muscle::Muscle(IValve* inputValve, IValve* outputValve) {
  this->inputValve = inputValve;
  this->outputValve = outputValve;
  extend();
  Serial.println(String("Muscle initialized, inlet pin: ") + inputValve->getValvePin() + String(", outlet pin: ") +
                 outputValve->getValvePin());
}

void Muscle::extend() {
  inputValve->close();
  outputValve->open();
}

void Muscle::retract() {
  outputValve->close();
  inputValve->open();
}

void Muscle::addPressure(int pressureTime) {
  if (pressureTime < 20) {
    pressureTime = 20;
  }

  outputValve->close();
  delay(200);
  inputValve->open();
  delay(pressureTime);
  inputValve->close();
}

void Muscle::releasePressure(int pressureTime) {
  if (pressureTime < 20) {
    pressureTime = 20;
  }

  inputValve->close();
  delay(200);
  outputValve->open();
  delay(pressureTime);
  outputValve->close();
}

void Muscle::openInput() {
  inputValve->open();
}

void Muscle::closeInput() {
  inputValve->close();
}

void Muscle::openOutput() {
  outputValve->open();
}

void Muscle::closeOutput() {
  outputValve->close();
}

bool Muscle::isExtended() {
  return !inputValve->isOpen() && outputValve->isOpen();
}

String Muscle::getStatus() {
  return String("status: ") + (isExtended() ? "Muscle extended" : "Muscle retracted");
}

void Muscle::test() {
  Serial.println("Muscle test");

  Serial.println("Retracting Muscle");
  delay(1000);
  retract();

  delay(3000);
  Serial.println("Extending Muscle");
  delay(1000);
  extend();

  delay(3000);
  Serial.println("Retracting Muscle");
  delay(1000);
  retract();

  delay(3000);
  Serial.println("Extending Muscle");
  delay(1000);
  extend();
}
