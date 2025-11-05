#include "Adafruit_MotorShield.h"
#include "Arduino.h"
#include "Wire.h"
#include "control-algorithms/PIDControlAlgorithm.h"
#include "enums/ValveType.h"
#include "models/Actuator.h"
#include "models/Gyroscope.h"
#include "models/Muscle.h"
#include "models/Valve.h"
#include "services/ArduinoMonitorService.h"
#include "services/ValveFactory.h"

const int availableValvePins[] = {4, 5, 6, 7, 10, 11, 12, 13};

Muscle* leftMuscle;
Actuator* frontActuator;
Actuator* backActuator;
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
  leftMuscle = new Muscle(valveFactory->createValve(4, ValveType::INLET, 0X60),
                          valveFactory->createValve(3, ValveType::OUTLET, 0X60));

  IValve* frontInletValves[] = {
    valveFactory->createValve(4, ValveType::INLET),
    valveFactory->createValve(10, ValveType::INLET)
  };
  IValve* frontOutletValves[] = {
    valveFactory->createValve(5, ValveType::OUTLET),
    valveFactory->createValve(11, ValveType::OUTLET)
  };
  frontActuator = new Actuator(frontInletValves, 2, frontOutletValves, 2);

  IValve* backInletValves[] = {
    valveFactory->createValve(4, ValveType::INLET, 0x60),
    valveFactory->createValve(13, ValveType::INLET)
  };
  IValve* backOutletValves[] = {
    valveFactory->createValve(1, ValveType::OUTLET, 0x60),
    valveFactory->createValve(3, ValveType::OUTLET, 0x60)
  };

  backActuator = new Actuator(backInletValves, 2, backOutletValves, 2);
  arduinoMonitorService->printPossibleCommands(nullptr);
}

void loop() {
  arduinoMonitorService->controlThroughMonitor(leftMuscle, gyroscope, controlAlgorithm, frontActuator, backActuator);
}