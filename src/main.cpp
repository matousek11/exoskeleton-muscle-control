#include "Adafruit_MotorShield.h"
#include "Arduino.h"
#include "Wire.h"
#include "control-algorithms/PIDControlAlgorithm.h"
#include "enums/ValveType.h"
#include "models/Gyroscope.h"
#include "models/Muscle.h"
#include "models/Valve.h"
#include "services/ArduinoMonitorService.h"
#include "services/ValveFactory.h"

const int availableValvePins[] = {4, 5, 6, 7, 10, 11, 12, 13};

Muscle* leftMuscle;
ValveFactory* valveFactory;
Gyroscope* gyroscope;
IControlAlgorithm* controlAlgorithm;
ArduinoMonitorService* arduinoMonitorService;

void setup() {
  Serial.begin(115200);
  // needed for MPU6050 readings and I2C scanner
  Wire.begin();

  arduinoMonitorService = new ArduinoMonitorService();
  valveFactory = new ValveFactory();

  gyroscope = new Gyroscope(0x68);
  controlAlgorithm = new PIDControlAlgorithm();
  leftMuscle = new Muscle(valveFactory->createValve(4, ValveType::OUTLET, 0X60),
                          valveFactory->createValve(3, ValveType::OUTLET, 0X60));
  arduinoMonitorService->printPossibleCommands(nullptr);
}

void loop() {
  arduinoMonitorService->controlThroughMonitor(leftMuscle, gyroscope, controlAlgorithm);
}