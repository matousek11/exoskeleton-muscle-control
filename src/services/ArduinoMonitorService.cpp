#include "ArduinoMonitorService.h"

#include "./../control-algorithms/PIDControlAlgorithm.h"
#include "./../helpers/Debugger.h"
#include "./../interfaces/IControlAlgorithm.h"
#include "./../models/Actuator.h"
#include "./../models/ControlTarget.h"
#include "./../models/Muscle.h"
#include "Arduino.h"

void ArduinoMonitorService::controlThroughMonitor(const SystemComponents& systemComponents) {
  if (!Serial.available()) {
    return;
  }

  bool unknownCommand = false;

  String command = Serial.readStringUntil('\n');  // read until Enter
  command.trim();
  clearSerialMonitor();
  Serial.println("---- Output ----");

  int pressureTimeIncrement = 15;

  if (command.equalsIgnoreCase("e")) {
    systemComponents.muscle->extend();
    Serial.println(systemComponents.muscle->getStatus());
  } else if (command.equalsIgnoreCase("r")) {
    systemComponents.muscle->retract();
    Serial.println(systemComponents.muscle->getStatus());
  } else if (command.equalsIgnoreCase("+")) {
    systemComponents.muscle->addPressure(25);
    Serial.println("Pressure added");
  } else if (command.equalsIgnoreCase("-")) {
    systemComponents.muscle->releasePressure(25);
    Serial.println("Pressure released");
  } else if (command.equalsIgnoreCase("io")) {
    systemComponents.muscle->openInput();
    Serial.println("Input opened");
  } else if (command.equalsIgnoreCase("ic")) {
    systemComponents.muscle->closeInput();
    Serial.println("Input closed");
  } else if (command.equalsIgnoreCase("oo")) {
    systemComponents.muscle->openOutput();
    Serial.println("Output open");
  } else if (command.equalsIgnoreCase("oc")) {
    systemComponents.muscle->closeOutput();
    Serial.println("Output closed");
  } else if (command.equalsIgnoreCase("status")) {
    Serial.println(systemComponents.muscle->getStatus());
  } else if (command.equalsIgnoreCase("test")) {
    systemComponents.muscle->test();
  } else if (command.equalsIgnoreCase("bdg")) {
    systemComponents.gyroscope->printValues(0, nullptr);
  } else if (command.equalsIgnoreCase("tdg")) {
    systemComponents.upperGyroscope->printValues(0, nullptr);
  } else if (command.equalsIgnoreCase("ia")) {
    Serial.println("Init axis");
    systemComponents.upperGyroscope->calibrateXAngle();
    systemComponents.upperGyroscope->calibrateYAngle();
    systemComponents.upperGyroscope->calibrateZAngle();
    systemComponents.gyroscope->calibrateXAngle();
    systemComponents.gyroscope->calibrateYAngle();
    systemComponents.gyroscope->calibrateZAngle();
  } else if (command.equalsIgnoreCase("t70")) {
    Serial.println("target 70 degrees");
    ControlTarget targets[1] = {ControlTarget(0.0f, 70.0f, 0)};
    systemComponents.controlAlgorithm->controlMuscle(systemComponents.muscle, systemComponents.gyroscope, 20000,
                                                     targets, 1);
  } else if (command.equalsIgnoreCase("t-dyn")) {
    Serial.println("target 70, 30, 70 and then 60 degrees");
    ControlTarget targets[4] = {ControlTarget(0.0f, 70.0f, 90.0f), ControlTarget(0.3f, 30.0f, 90.0f),
                                ControlTarget(0.6f, 70.0f, 90.0f), ControlTarget(0.8f, 60.0f, 90.0f)};
    systemComponents.controlAlgorithm->controlMuscle(systemComponents.muscle, systemComponents.gyroscope, 25000,
                                                     targets, 4);
  } else if (command.equalsIgnoreCase("t-ant-dyn")) {
    Serial.println(F("target -20, 0 degrees, 10 degrees and -10 degrees"));
    ControlTarget targets[4] = {ControlTarget(0.0f, 70.0f, 0.0f), ControlTarget(0.25f, 90.0f, 0.0f),
                                ControlTarget(0.5f, 100.0f, 0.0f), ControlTarget(0.75f, 80.0f, 0.0f)};
    systemComponents.antagonisticControlAlgorithm->controlMuscle(
        systemComponents.stateMachineFrontActuator, systemComponents.stateMachineBackActuator,
        systemComponents.gyroscope, 23000, targets, 4, nullptr);

    Serial.println(F("DEBUG: Returned from controlMuscle"));
    Serial.println(F("DEBUG: About to close all valves"));
    closeAllValves(systemComponents);
    Serial.println(F("DEBUG: All valves closed"));
  } else if (command.equalsIgnoreCase("t-ant-dyn-2-dof")) {
    Serial.println("target -20;0, 0;0 degrees, 10;0 degrees and -10;0 degrees");
    ControlTarget targets[4] = {ControlTarget(0.0f, 70.0f, 0.0f), ControlTarget(0.25f, 90.0f, 0.0f),
                                ControlTarget(0.5f, 100.0f, 0.0f), ControlTarget(0.75f, 80.0f, 0.0f)};
    systemComponents.twoDOFAntagonisticControlAlgorithm->controlMuscle(
        systemComponents.frontActuator, systemComponents.backActuator, systemComponents.leftActuator,
        systemComponents.rightActuator, systemComponents.gyroscope, 23000, targets, 4);

    closeAllValves(systemComponents);
  } else if (command.equalsIgnoreCase("t-ant-dyn-2-dof-v2")) {
    Serial.println("target -20;-2, 0;10 degrees, 10;0 degrees and -10;0 degrees");
    ControlTarget targets[4] = {ControlTarget(0.0f, 70.0f, -2.0f), ControlTarget(0.25f, 90.0f, 10.0f),
                                ControlTarget(0.5f, 100.0f, 0.0f), ControlTarget(0.75f, 80.0f, 0.0f)};
    systemComponents.twoDOFAntagonisticParticularMuscleControlAlgorithm->controlMuscle(
        systemComponents.leftFrontActuator, systemComponents.rightFrontActuator, systemComponents.leftBackActuator,
        systemComponents.rightBackActuator, systemComponents.gyroscope, 40000, targets, 4);

    closeAllValves(systemComponents);
  } else if (command.equalsIgnoreCase("t-ant-dyn-3-dof")) {
    Serial.println("target -20;0;-90 deg, 0;10;-10 deg, 10;0;-100 deg and -10;0;-10 deg");
    ControlTarget lowerLegTargets[4] = {ControlTarget(0.0f, 70.0f, 0.0f), ControlTarget(0.25f, 90.0f, 10.0f),
                                        ControlTarget(0.5f, 100.0f, 0.0f), ControlTarget(0.75f, 80.0f, 0.0f)};
    ControlTarget upperLegTargets[4] = {ControlTarget(0.0f, -90.0f, 0.0f), ControlTarget(0.25f, -10.0f, 0.0f),
                                        ControlTarget(0.5f, -100.0f, 0.0f), ControlTarget(0.75f, -10.0f, 0.0f)};
    systemComponents.threeDOFAntagonisticParticularMuscleControlAlgorithm->controlMuscle(
        systemComponents.leftFrontActuator, systemComponents.rightFrontActuator, systemComponents.leftBackActuator,
        systemComponents.rightBackActuator, systemComponents.topFrontActuator, systemComponents.topBackActuator,
        systemComponents.gyroscope, systemComponents.upperGyroscope, 60000, lowerLegTargets, 4, upperLegTargets, 4);

    closeAllValves(systemComponents);
  } else if (command.equalsIgnoreCase("t-up-ant-dyn")) {
    Serial.println("target -30;0, -70;0 degrees, -100;0 and -10;0 degrees for upper leg");
    ControlTarget targets[3] = {ControlTarget(0.0f, -30.0f, 0.0f), ControlTarget(0.25f, -70.0f, 0.0f),
                                ControlTarget(0.75f, -10.0f, 0.0f)};
    systemComponents.antagonisticControlAlgorithm->controlMuscle(
        systemComponents.stateMachineTopFrontActuator, systemComponents.stateMachineTopBackActuator,
        systemComponents.gyroscope, 30000, targets, 3, systemComponents.upperGyroscope);

    closeAllValves(systemComponents);
  } else if (command.equalsIgnoreCase("stabilize")) {
    Serial.println("Centering test stand");
    systemComponents.topFrontActuator->addPressureFluidlyWithOutflowValve();
    systemComponents.topBackActuator->addPressureFluidlyWithOutflowValve();
    systemComponents.topFrontActuator->addPressureFluidlyWithOutflowValve();

    ControlTarget targets[1] = {ControlTarget(0.0f, 90.0f, 0.0f)};
    systemComponents.twoDOFAntagonisticControlAlgorithm->controlMuscle(
        systemComponents.frontActuator, systemComponents.backActuator, systemComponents.leftActuator,
        systemComponents.rightActuator, systemComponents.gyroscope, 5000, targets, 1);

    closeAllValves(systemComponents);
  } else if (command.equalsIgnoreCase("i2c")) {
    Debugger::scanI2C();
  } else if (command.equalsIgnoreCase("fe")) {
    systemComponents.frontActuator->extend();
  } else if (command.equalsIgnoreCase("fr")) {
    systemComponents.frontActuator->retract();
  } else if (command.equalsIgnoreCase("f+")) {
    systemComponents.frontActuator->addPressureFluidlyWithOutflowValve(pressureTimeIncrement);
  } else if (command.equalsIgnoreCase("f-")) {
    systemComponents.frontActuator->releasePressureFluidlyWithInputValve(pressureTimeIncrement);
  } else if (command.equalsIgnoreCase("f-test")) {
    systemComponents.frontActuator->test();
  } else if (command.equalsIgnoreCase("be")) {
    systemComponents.backActuator->extend();
  } else if (command.equalsIgnoreCase("br")) {
    systemComponents.backActuator->retract();
  } else if (command.equalsIgnoreCase("b+")) {
    systemComponents.backActuator->addPressureFluidlyWithOutflowValve(pressureTimeIncrement);
  } else if (command.equalsIgnoreCase("b-")) {
    systemComponents.backActuator->releasePressureFluidlyWithInputValve(pressureTimeIncrement);
  } else if (command.equalsIgnoreCase("b-test")) {
    systemComponents.backActuator->test();
  } else if (command.equalsIgnoreCase("le")) {
    systemComponents.leftActuator->extend();
  } else if (command.equalsIgnoreCase("lr")) {
    systemComponents.leftActuator->retract();
  } else if (command.equalsIgnoreCase("l+")) {
    systemComponents.leftActuator->addPressureFluidlyWithOutflowValve(pressureTimeIncrement);
  } else if (command.equalsIgnoreCase("l-")) {
    systemComponents.leftActuator->releasePressureFluidlyWithInputValve(pressureTimeIncrement);
  } else if (command.equalsIgnoreCase("l-test")) {
    systemComponents.leftActuator->test();
  } else if (command.equalsIgnoreCase("re")) {
    systemComponents.rightActuator->extend();
  } else if (command.equalsIgnoreCase("rr")) {
    systemComponents.rightActuator->retract();
  } else if (command.equalsIgnoreCase("r+")) {
    systemComponents.rightActuator->addPressureFluidlyWithOutflowValve(pressureTimeIncrement);
  } else if (command.equalsIgnoreCase("r-")) {
    systemComponents.rightActuator->releasePressureFluidlyWithInputValve(pressureTimeIncrement);
  } else if (command.equalsIgnoreCase("r-test")) {
    systemComponents.rightActuator->test();
  } else if (command.equalsIgnoreCase("tfe")) {
    systemComponents.topFrontActuator->extend();
  } else if (command.equalsIgnoreCase("tfr")) {
    systemComponents.topFrontActuator->retract();
  } else if (command.equalsIgnoreCase("tf+")) {
    systemComponents.topFrontActuator->addPressureFluidlyWithOutflowValve(pressureTimeIncrement);
  } else if (command.equalsIgnoreCase("tf-")) {
    systemComponents.topFrontActuator->releasePressureFluidlyWithInputValve(pressureTimeIncrement);
  } else if (command.equalsIgnoreCase("tf-test")) {
    systemComponents.topFrontActuator->test();
  } else if (command.equalsIgnoreCase("tbe")) {
    systemComponents.topBackActuator->extend();
  } else if (command.equalsIgnoreCase("tbr")) {
    systemComponents.topBackActuator->retract();
  } else if (command.equalsIgnoreCase("tb+")) {
    systemComponents.topBackActuator->addPressureFluidlyWithOutflowValve(pressureTimeIncrement);
  } else if (command.equalsIgnoreCase("tb-")) {
    systemComponents.topBackActuator->releasePressureFluidlyWithInputValve(pressureTimeIncrement);
  } else if (command.equalsIgnoreCase("tb-test")) {
    systemComponents.topBackActuator->test();
  } else if (command.equalsIgnoreCase("close-all") || command.equalsIgnoreCase("c")) {
    closeAllValves(systemComponents);
  } else if (command.equalsIgnoreCase("cycle-test")) {
    systemComponents.muscle->cycleTest();
  } else {
    unknownCommand = true;
  }

  printPossibleCommands(&command, unknownCommand);
  unknownCommand = false;
}

void ArduinoMonitorService::printPossibleCommands(String* inputCommand, bool unknownCommand) {
  Serial.println();
  if (inputCommand != nullptr) {
    Serial.println(F("---- Input ----"));
    Serial.println(*inputCommand);
    if (unknownCommand == true) {
      Serial.println(F("Unknown command"));
    }
  }

  Serial.println(F("--- Commands ---"));
  Serial.println(F("Muscle commands: 'e' - extend, 'r' - retract, '+' - pressurize valve for 25ms,"));
  Serial.println(F("  '-' - depressurize valve for 25ms, 'test' - tests muscles valves, 'status' - status of muscle"));
  Serial.println(F("Commands for valves on muscle: 'io' - open input valve, 'ic' - close input valve,"));
  Serial.println(F("  'oo' - open output valve, 'oc' - close output valve"));
  Serial.println(F("Commands for gyroscope (MPU6050): 'bdg' - display bottom gyroscope output - stop by 'c',"));
  Serial.println(F("  'tdg' - display top gyroscope output - stop by 'c'"));
  Serial.println(F("Commands for feedback loop algorithms: 'stabilize' - put test stand into start position,"));
  Serial.println(F("  't70' - target 70 degrees, 't-dyn' - target 70 and then 30 degrees,"));
  Serial.println(F("  't-ant-dyn' - target -20, 0, 10 and -20 degrees,"));
  Serial.println(F("  't-ant-dyn-2-dof' - target -20;0, 0;0, 10;0 and -20;0,"));
  Serial.println(F("  't-ant-dyn-2-dof-v2' - target -20;80, 0;100, 10;80 and -20;90 with particular muscles,"));
  Serial.println(F("  't-ant-dyn-3-dof' - target -20;-10;-90 deg, 0;10;-10 deg, 10;-10;-100 deg and -10;0;-10 deg"));
  Serial.println(F("Commands for upper leg: 't-up-ant-dyn' - target -30, -70, -100 and -10 degrees - stop by c"));
  Serial.println(F("Top front actuator: 'tfe' - extend, 'tfr' - retract, 'tf+' - add pressure,"));
  Serial.println(F("  'tf-' - remove pressure, 'tf-test' - test run"));
  Serial.println(F("Top back actuator: 'tbe' - extend, 'tbr' - retract, 'tb+' - add pressure,"));
  Serial.println(F("  'tb-' - remove pressure, 'tb-test' - test run"));
  Serial.println(F("Front actuator: 'fe' - extend, 'fr' - retract, 'f+' - add pressure,"));
  Serial.println(F("  'f-' - remove pressure, 'f-test' - test run"));
  Serial.println(F("Back actuator: 'be' - extend, 'br' - retract, 'b+' - add pressure,"));
  Serial.println(F("  'b-' - remove pressure, 'b-test' - test run"));
  Serial.println(F("Left actuator: 'le' - extend, 'lr' - retract, 'l+' - add pressure,"));
  Serial.println(F("  'l-' - remove pressure, 'l-test' - test run"));
  Serial.println(F("Right actuator: 're' - extend, 'rr' - retract, 'r+' - add pressure,"));
  Serial.println(F("  'r-' - remove pressure, 'r-test' - test run"));
  Serial.println(F("All valves commands: 'close-all/c' - close all valves"));
  Serial.println(F("Tests: 'cycle-test' - runs cycle test on muscle, cancelled by c"));
  Serial.println(F("Debug tools: 'i2c' - I2C device scanner"));
}

void ArduinoMonitorService::clearSerialMonitor() {
  for (int i = 0; i < 60; i++) {
    Serial.println();
  }
}

void ArduinoMonitorService::closeAllValves(const SystemComponents& systemComponents) {
  systemComponents.muscle->closeInput();
  systemComponents.muscle->closeOutput();

  systemComponents.topFrontActuator->closeInput();
  systemComponents.topFrontActuator->closeOutput();
  systemComponents.topBackActuator->closeInput();
  systemComponents.topBackActuator->closeOutput();

  systemComponents.frontActuator->closeInput();
  systemComponents.frontActuator->closeOutput();
  systemComponents.leftActuator->closeInput();
  systemComponents.leftActuator->closeOutput();
  systemComponents.rightActuator->closeInput();
  systemComponents.rightActuator->closeOutput();
  systemComponents.backActuator->closeInput();
  systemComponents.backActuator->closeOutput();
}