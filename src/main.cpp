#include "Adafruit_MotorShield.h"
#include "Arduino.h"
#include "Wire.h"
#include "control-algorithms/AntagonisticPIDControlAlgorithm.h"
#include "control-algorithms/PIDControlAlgorithm.h"
#include "control-algorithms/TwoDOFAntagonisticParticularMuscleControlAlgorithm.h"
#include "enums/ValveType.h"
#include "models/Actuator.h"
#include "models/Gyroscope.h"
#include "models/Muscle.h"
#include "models/Valve.h"
#include "services/ArduinoMonitorService.h"
#include "services/ValveFactory.h"

SystemComponents systemComponents;
ValveFactory* valveFactory;
ArduinoMonitorService* arduinoMonitorService;

static IValve* topFrontInletValves[1];
static IValve* topFrontOutletValves[1];
static IValve* topBackInletValves[1];
static IValve* topBackOutletValves[1];

static IValve* frontInletValves[2];
static IValve* frontOutletValves[2];
static IValve* leftInletValves[2];
static IValve* leftOutletValves[2];
static IValve* rightInletValves[2];
static IValve* rightOutletValves[2];
static IValve* backInletValves[2];
static IValve* backOutletValves[2];

static IValve* leftFrontInletValves[1];
static IValve* leftFrontOutletValves[1];
static IValve* rightFrontInletValves[1];
static IValve* rightFrontOutletValves[1];
static IValve* leftBackInletValves[1];
static IValve* leftBackOutletValves[1];
static IValve* rightBackInletValves[1];
static IValve* rightBackOutletValves[1];

void setup() {
  // setup communication speed with terminal
  Serial.begin(115200);
  // needed for MPU6050 readings and I2C scanner
  Wire.begin();

  arduinoMonitorService = new ArduinoMonitorService();
  valveFactory = new ValveFactory();

  systemComponents.upperGyroscope = new Gyroscope(0x69);
  systemComponents.gyroscope = new Gyroscope(0x68);

  // Init of control algorithms
  systemComponents.controlAlgorithm = new PIDControlAlgorithm();
  systemComponents.antagonisticControlAlgorithm = new AntagonisticPIDControlAlgorithm();
  systemComponents.twoDOFAntagonisticControlAlgorithm = new TwoDOFAntagonisticPIDControlAlgorithm();
  systemComponents.twoDOFAntagonisticParticularMuscleControlAlgorithm =
      new TwoDOFAntagonisticParticularMuscleControlAlgorithm();

  // One muscle for non-antagonistic gravity pull test with test stand V1
  systemComponents.muscle = new Muscle(valveFactory->createValve(13, ValveType::INLET),
                                       valveFactory->createValve(1, ValveType::OUTLET, 0X60));

  // Upper leg actuators initialization
  topFrontInletValves[0] = valveFactory->createValve(12, ValveType::INLET);
  topFrontOutletValves[0] = valveFactory->createValve(3, ValveType::OUTLET, 0x61);
  systemComponents.topFrontActuator = new Actuator(topFrontInletValves, 1, topFrontOutletValves, 1);

  topBackInletValves[0] = valveFactory->createValve(11, ValveType::INLET);
  topBackOutletValves[0] = valveFactory->createValve(4, ValveType::OUTLET, 0x61);
  systemComponents.topBackActuator = new Actuator(topBackInletValves, 1, topBackOutletValves, 1);

  // Antagonistic 1 and 2 DOF actuator algorithm parts initialization
  frontInletValves[0] = valveFactory->createValve(4, ValveType::INLET, 0x60);
  frontInletValves[1] = valveFactory->createValve(13, ValveType::INLET);
  frontOutletValves[0] = valveFactory->createValve(1, ValveType::OUTLET, 0x60);
  frontOutletValves[1] = valveFactory->createValve(3, ValveType::OUTLET, 0x60);
  systemComponents.frontActuator = new Actuator(frontInletValves, 2, frontOutletValves, 2);

  leftInletValves[0] = valveFactory->createValve(13, ValveType::INLET);
  leftInletValves[1] = valveFactory->createValve(4, ValveType::INLET);
  leftOutletValves[0] = valveFactory->createValve(6, ValveType::OUTLET);
  leftOutletValves[1] = valveFactory->createValve(1, ValveType::OUTLET, 0X60);
  systemComponents.leftActuator = new Actuator(leftInletValves, 2, leftOutletValves, 2);

  rightInletValves[0] = valveFactory->createValve(10, ValveType::INLET);
  rightInletValves[1] = valveFactory->createValve(4, ValveType::INLET, 0x60);
  rightOutletValves[0] = valveFactory->createValve(7, ValveType::OUTLET);
  rightOutletValves[1] = valveFactory->createValve(3, ValveType::OUTLET, 0X60);
  systemComponents.rightActuator = new Actuator(rightInletValves, 2, rightOutletValves, 2);

  backInletValves[0] = valveFactory->createValve(4, ValveType::INLET);
  backInletValves[1] = valveFactory->createValve(10, ValveType::INLET);
  backOutletValves[0] = valveFactory->createValve(7, ValveType::OUTLET);
  backOutletValves[1] = valveFactory->createValve(6, ValveType::OUTLET);
  systemComponents.backActuator = new Actuator(backInletValves, 2, backOutletValves, 2);

  // Antagonistic 2 DOF particular muscle algorithm parts initialization
  leftFrontInletValves[0] = valveFactory->createValve(13, ValveType::INLET);
  leftFrontOutletValves[0] = valveFactory->createValve(1, ValveType::OUTLET, 0x60);
  systemComponents.leftFrontActuator = new Actuator(leftFrontInletValves, 1, leftFrontOutletValves, 1);

  rightFrontInletValves[0] = valveFactory->createValve(4, ValveType::INLET, 0x60);
  rightFrontOutletValves[0] = valveFactory->createValve(3, ValveType::OUTLET, 0x60);
  systemComponents.rightFrontActuator = new Actuator(rightFrontInletValves, 1, rightFrontOutletValves, 1);

  leftBackInletValves[0] = valveFactory->createValve(4, ValveType::INLET);
  leftBackOutletValves[0] = valveFactory->createValve(6, ValveType::OUTLET);
  systemComponents.leftBackActuator = new Actuator(leftBackInletValves, 1, leftBackOutletValves, 1);

  rightBackInletValves[0] = valveFactory->createValve(10, ValveType::INLET);
  rightBackOutletValves[0] = valveFactory->createValve(7, ValveType::OUTLET);
  systemComponents.rightBackActuator = new Actuator(rightBackInletValves, 1, rightBackOutletValves, 1);

  arduinoMonitorService->printPossibleCommands(nullptr);
}

void loop() {
  arduinoMonitorService->controlThroughMonitor(systemComponents);
}