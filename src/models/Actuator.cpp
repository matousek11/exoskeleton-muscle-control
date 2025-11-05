#include "Actuator.h"

#include "../enums/ValveType.h"
#include "./../interfaces/IValve.h"

Actuator::Actuator(IValve* inletValves[], size_t inletValvesCount, IValve* outletValves[], size_t outletValvesCount) {
  this->inletValves = inletValves;
  this->inletValvesCount = inletValvesCount;
  this->outletValves = outletValves;
  this->outletValvesCount = outletValvesCount;
  extend();
  Serial.println("Initializing actuator...");
  Serial.println("Actuator contains inlet valves on pins:");

  for (size_t i = 0; i < inletValvesCount; ++i) {
    Serial.println(inletValves[i]->getValvePin());
  }

  Serial.println("Actuator contains outlet valves on pins:");
  for (size_t i = 0; i < outletValvesCount; ++i) {
    Serial.println(outletValves[i]->getValvePin());
  }
}

void Actuator::extend() {
  extended = true;
  for (size_t i = 0; i < inletValvesCount; ++i) {
    inletValves[i]->close();
  }

  for (size_t i = 0; i < outletValvesCount; ++i) {
    outletValves[i]->open();
  }
}

void Actuator::retract() {
  extended = false;
  for (size_t i = 0; i < outletValvesCount; ++i) {
    outletValves[i]->close();
  }

  for (size_t i = 0; i < inletValvesCount; ++i) {
    inletValves[i]->open();
  }
}

void Actuator::addPressure(int pressureTime) {
  int muscleSealingTime = 10;
  if (pressureTime < 25) {
    pressureTime = 25;
  }

  for (size_t i = 0; i < outletValvesCount; ++i) {
    outletValves[i]->close();
  }
  delay(muscleSealingTime);
  for (size_t i = 0; i < inletValvesCount; ++i) {
    inletValves[i]->open();
  }
  delay(pressureTime);
  for (size_t i = 0; i < inletValvesCount; ++i) {
    inletValves[i]->close();
  }
}

void Actuator::releasePressure(int pressureTime) {
  int muscleSealingTime = 10;
  if (pressureTime < 25) {
    pressureTime = 25;
  }

  for (size_t i = 0; i < inletValvesCount; ++i) {
    inletValves[i]->close();
  }
  delay(muscleSealingTime);
  for (size_t i = 0; i < outletValvesCount; ++i) {
    outletValves[i]->open();
  }
  delay(pressureTime);
  for (size_t i = 0; i < outletValvesCount; ++i) {
    outletValves[i]->close();
  }
}

void Actuator::addPressureFluidly(int pressureTime) {
  int muscleSealingTime = 10;
  int openTime = 25;
  if (pressureTime < openTime) {
    pressureTime = openTime;
  }

  int numberOfOpenings = pressureTime / openTime;

  for (size_t i = 0; i < outletValvesCount; ++i) {
    outletValves[i]->close();
  }
  delay(muscleSealingTime);
  for (int i = 0; i < numberOfOpenings; i++) {
    for (size_t i = 0; i < inletValvesCount; ++i) {
      inletValves[i]->open();
    }
    delay(openTime);
    for (size_t i = 0; i < inletValvesCount; ++i) {
      inletValves[i]->close();
    }
    delay(muscleSealingTime);
  }
}

void Actuator::releasePressureFluidly(int pressureTime) {
  int muscleSealingTime = 10;
  int openTime = 25;
  if (pressureTime < openTime) {
    pressureTime = openTime;
  }

  int numberOfOpenings = pressureTime / openTime;

  for (size_t i = 0; i < inletValvesCount; ++i) {
    inletValves[i]->close();
  }
  delay(muscleSealingTime);
  for (int i = 0; i < numberOfOpenings; i++) {
    for (size_t i = 0; i < outletValvesCount; ++i) {
      outletValves[i]->open();
    }
    delay(openTime);
    for (size_t i = 0; i < outletValvesCount; ++i) {
      outletValves[i]->close();
    }
    delay(muscleSealingTime);
  }
}

void Actuator::openInput() {
  for (size_t i = 0; i < inletValvesCount; ++i) {
    inletValves[i]->open();
  }
}

void Actuator::closeInput() {
  for (size_t i = 0; i < inletValvesCount; ++i) {
    inletValves[i]->close();
  }
}

void Actuator::openOutput() {
  for (size_t i = 0; i < outletValvesCount; ++i) {
    outletValves[i]->open();
  }
}

void Actuator::closeOutput() {
  for (size_t i = 0; i < outletValvesCount; ++i) {
    outletValves[i]->close();
  }
}

bool Actuator::isExtended() {
  return extended;
}

String Actuator::getStatus() {
  return String("status: ") + (isExtended() ? "Muscle extended" : "Muscle retracted");
}

void Actuator::test() {
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
