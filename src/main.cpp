#include "Adafruit_MotorShield.h"
#include "Arduino.h"
#include "Wire.h"
#include "enums/ValveType.h"
#include "models/Muscle.h"
#include "models/Valve.h"
#include "services/ArduinoMonitorService.h"
#include "services/ValveFactory.h"

const int availableValvePins[] = {4, 5, 6, 7, 10, 11, 12, 13};

Muscle* leftMuscle;
ValveFactory* valveFactory;
ArduinoMonitorService* arduinoMonitorService;

void setup() {
  Serial.begin(9600);

  arduinoMonitorService = new ArduinoMonitorService();
  valveFactory = new ValveFactory();

  leftMuscle = new Muscle(valveFactory->createValve(availableValvePins[0], ValveType::INLET),
                          valveFactory->createValve(1, ValveType::OUTLET, 0X60));
  arduinoMonitorService->printPossibleCommands(nullptr);
}

void loop() {
  arduinoMonitorService->controlThroughMonitor(leftMuscle);
}