#include "Arduino.h"
#include "enums/ValveType.h"
#include "models/Muscle.h"
#include "models/Valve.h"
#include "services/ArduinoMonitorService.h"

const int availableValvePins[] = {4, 5, 6, 7, 10, 11, 12, 13};

Muscle* leftMuscle;
ArduinoMonitorService* arduinoMonitorService;

void setup() {
  Serial.begin(9600);

  arduinoMonitorService = new ArduinoMonitorService();
  leftMuscle = new Muscle(availableValvePins[0], availableValvePins[1]);
  arduinoMonitorService->printPossibleCommands(nullptr);
}

void loop() {
  arduinoMonitorService->controlThroughMonitor(leftMuscle);
}