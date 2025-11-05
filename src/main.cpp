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

static IValve* backInletValves[2];
static IValve* backOutletValves[2];
static IValve* frontInletValves[2];
static IValve* frontOutletValves[2];

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

  frontInletValves[0] = valveFactory->createValve(4, ValveType::INLET);
  frontInletValves[1] = valveFactory->createValve(10, ValveType::INLET);
  frontOutletValves[0] = valveFactory->createValve(5, ValveType::OUTLET);
  frontOutletValves[1] = valveFactory->createValve(11, ValveType::OUTLET);
  frontActuator = new Actuator(frontInletValves, 2, frontOutletValves, 2);

  backInletValves[0] = valveFactory->createValve(4, ValveType::INLET, 0x60);
  backInletValves[1] = valveFactory->createValve(13, ValveType::INLET);
  backOutletValves[0] = valveFactory->createValve(1, ValveType::OUTLET, 0x60);
  backOutletValves[1] = valveFactory->createValve(3, ValveType::OUTLET, 0x60);
  backActuator = new Actuator(backInletValves, 2, backOutletValves, 2);
  arduinoMonitorService->printPossibleCommands(nullptr);
}

void loop() {
  arduinoMonitorService->controlThroughMonitor(leftMuscle, gyroscope, controlAlgorithm, frontActuator, backActuator);
}