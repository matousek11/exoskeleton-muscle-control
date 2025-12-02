#include "ArduinoMonitorService.h"

#include "./../control-algorithms/PIDControlAlgorithm.h"
#include "./../helpers/Debugger.h"
#include "./../interfaces/IControlAlgorithm.h"
#include "./../models/Actuator.h"
#include "./../models/Muscle.h"
#include "Arduino.h"

void ArduinoMonitorService::controlThroughMonitor(
    Muscle* muscle, Gyroscope* gyroscope, IControlAlgorithm* controlAlgorithm,
    AntagonisticPIDControlAlgorithm* antagonisticControlAlgorithm,
    TwoDOFAntagonisticPIDControlAlgorithm* twoDOFAntagonisticControlAlgorithm, TwoDOFAntagonisticParticularMuscleControlAlgorithm* twoDOFAntagonisticParticularMuscleControlAlgorithm,
    Actuator* frontActuator,
    Actuator* backActuator, Actuator* leftActuator, Actuator* rightActuator, Actuator* topFrontActuator,
    Actuator* topBackActuator,
    Actuator* leftFrontActuator, Actuator* rightFrontActuator, Actuator* leftBackActuator, Actuator* rightBackActuator) {
  bool unknownCommand = false;

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // read until Enter
    command.trim();
    clearSerialMonitor();
    Serial.println("---- Output ----");

    int pressureTimeIncrement = 15;

    if (command.equalsIgnoreCase("e")) {
      muscle->extend();
      Serial.println(muscle->getStatus());
    } else if (command.equalsIgnoreCase("r")) {
      muscle->retract();
      Serial.println(muscle->getStatus());
    } else if (command.equalsIgnoreCase("+")) {
      muscle->addPressure(25);
      Serial.println("Pressure added");
    } else if (command.equalsIgnoreCase("-")) {
      muscle->releasePressure(25);
      Serial.println("Pressure released");
    } else if (command.equalsIgnoreCase("io")) {
      muscle->openInput();
      Serial.println("Input opened");
    } else if (command.equalsIgnoreCase("ic")) {
      muscle->closeInput();
      Serial.println("Input closed");
    } else if (command.equalsIgnoreCase("oo")) {
      muscle->openOutput();
      Serial.println("Output open");
    } else if (command.equalsIgnoreCase("oc")) {
      muscle->closeOutput();
      Serial.println("Output closed");
    } else if (command.equalsIgnoreCase("status")) {
      Serial.println(muscle->getStatus());
    } else if (command.equalsIgnoreCase("test")) {
      muscle->test();
    } else if (command.equalsIgnoreCase("dg2")) {
      Serial.println("Show gyroscope output for 2 seconds");
      unsigned long startTime = millis();

      while (millis() - startTime < 2000) {
        gyroscope->updateValues();
        gyroscope->printValues();
      }
    } else if (command.equalsIgnoreCase("dg10")) {
      Serial.println("Show gyroscope output for 10 seconds");
      unsigned long startTime = millis();

      while (millis() - startTime < 10000) {
        gyroscope->updateValues();
        gyroscope->printValues();
      }
    } else if (command.equalsIgnoreCase("dg60")) {
      Serial.println("Show gyroscope output for 60 seconds");
      unsigned long startTime = millis();

      while (millis() - startTime < 60000) {
        gyroscope->updateValues();
        gyroscope->printValues();
      }
    } else if (command.equalsIgnoreCase("ia")) {
      Serial.println("Init axis");
      gyroscope->calibrateXAngle();
      gyroscope->calibrateYAngle();
      gyroscope->calibrateZAngle();
    } else if (command.equalsIgnoreCase("t70")) {
      Serial.println("target 70 degrees");
      ControlTarget targets[1] = {ControlTarget(0.0f, 70.0f, 0)};
      controlAlgorithm->controlMuscle(muscle, gyroscope, 20000, targets, 1);
    } else if (command.equalsIgnoreCase("t-dyn")) {
      Serial.println("target 70, 30, 70 and then 60 degrees");
      ControlTarget targets[4] = {ControlTarget(0.0f, 70.0f, 90.0f), ControlTarget(0.3f, 30.0f, 90.0f),
                                  ControlTarget(0.6f, 70.0f, 90.0f), ControlTarget(0.8f, 60.0f, 90.0f)};
      controlAlgorithm->controlMuscle(muscle, gyroscope, 25000, targets, 4);
    } else if (command.equalsIgnoreCase("t-ant-dyn")) {
      Serial.println("target -20, 0 degrees, 10 degrees and -10 degrees");
      ControlTarget targets[4] = {ControlTarget(0.0f, -20.0f, 90.0f), ControlTarget(0.25f, 0.0f, 90.0f),
                                  ControlTarget(0.5f, 10.0f, 90.0f), ControlTarget(0.75f, -20.0f, 90.0f)};
      antagonisticControlAlgorithm->controlMuscle(frontActuator, backActuator, gyroscope, 23000, targets, 4);

      frontActuator->closeInput();
      frontActuator->closeOutput();
      leftActuator->closeInput();
      leftActuator->closeOutput();
      rightActuator->closeInput();
      rightActuator->closeOutput();
      backActuator->closeInput();
      backActuator->closeOutput();

      topFrontActuator->closeInput();
      topFrontActuator->closeOutput();
      topBackActuator->closeInput();
      topBackActuator->closeOutput();

      muscle->closeInput();
      muscle->closeOutput();
    } else if (command.equalsIgnoreCase("t-ant-dyn-2-dof")) {
      Serial.println("target -20;0, 0;0 degrees, 10;0 degrees and -10;0 degrees");
      ControlTarget targets[4] = {ControlTarget(0.0f, -20.0f, 90.0f), ControlTarget(0.25f, 0.0f, 90.0f),
                                  ControlTarget(0.5f, 10.0f, 90.0f), ControlTarget(0.75f, -20.0f, 90.0f)};
      twoDOFAntagonisticControlAlgorithm->controlMuscle(frontActuator, backActuator, leftActuator, rightActuator,
                                                        gyroscope, 23000, targets, 4);

      frontActuator->closeInput();
      frontActuator->closeOutput();
      leftActuator->closeInput();
      leftActuator->closeOutput();
      rightActuator->closeInput();
      rightActuator->closeOutput();
      backActuator->closeInput();
      backActuator->closeOutput();

      topFrontActuator->closeInput();
      topFrontActuator->closeOutput();
      topBackActuator->closeInput();
      topBackActuator->closeOutput();

      muscle->closeInput();
      muscle->closeOutput();
    } else if (command.equalsIgnoreCase("t-ant-dyn-2-dof-v2")) {
      Serial.println("target -20;-10, 0;10 degrees, 10;-10 degrees and -10;0 degrees");
      ControlTarget targets[4] = {ControlTarget(0.0f, -20.0f, 80.0f), ControlTarget(0.25f, 0.0f, 100.0f),
                                  ControlTarget(0.5f, 10.0f, 80.0f), ControlTarget(0.75f, -20.0f, 90.0f)};
      twoDOFAntagonisticParticularMuscleControlAlgorithm->controlMuscle(leftFrontActuator, rightFrontActuator, leftBackActuator, rightBackActuator,
                                                        gyroscope, 50000, targets, 4);

      frontActuator->closeInput();
      frontActuator->closeOutput();
      leftActuator->closeInput();
      leftActuator->closeOutput();
      rightActuator->closeInput();
      rightActuator->closeOutput();
      backActuator->closeInput();
      backActuator->closeOutput();

      topFrontActuator->closeInput();
      topFrontActuator->closeOutput();
      topBackActuator->closeInput();
      topBackActuator->closeOutput();

      muscle->closeInput();
      muscle->closeOutput();
    } else if (command.equalsIgnoreCase("stabilize")) {
      Serial.println("Centering test stand");
      topFrontActuator->addPressureFluidlyWithOutflowValve();
      topBackActuator->addPressureFluidlyWithOutflowValve();
      topFrontActuator->addPressureFluidlyWithOutflowValve();
      
      ControlTarget targets[1] = {ControlTarget(0.0f, 0.0f, 90.0f)};
      twoDOFAntagonisticControlAlgorithm->controlMuscle(frontActuator, backActuator, leftActuator, rightActuator,
                                                        gyroscope, 5000, targets, 1);

      frontActuator->closeInput();
      frontActuator->closeOutput();
      leftActuator->closeInput();
      leftActuator->closeOutput();
      rightActuator->closeInput();
      rightActuator->closeOutput();
      backActuator->closeInput();
      backActuator->closeOutput();

      topFrontActuator->closeInput();
      topFrontActuator->closeOutput();
      topBackActuator->closeInput();
      topBackActuator->closeOutput();

      muscle->closeInput();
      muscle->closeOutput();
    } else if (command.equalsIgnoreCase("i2c")) {
      Debugger::scanI2C();
    } else if (command.equalsIgnoreCase("fe")) {
      frontActuator->extend();
    } else if (command.equalsIgnoreCase("fr")) {
      frontActuator->retract();
    } else if (command.equalsIgnoreCase("f+")) {
      frontActuator->addPressureFluidlyWithOutflowValve(pressureTimeIncrement);
    } else if (command.equalsIgnoreCase("f-")) {
      frontActuator->releasePressureFluidlyWithInputValve(pressureTimeIncrement);
    } else if (command.equalsIgnoreCase("f-test")) {
      frontActuator->test();
    } else if (command.equalsIgnoreCase("be")) {
      backActuator->extend();
    } else if (command.equalsIgnoreCase("br")) {
      backActuator->retract();
    } else if (command.equalsIgnoreCase("b+")) {
      backActuator->addPressureFluidlyWithOutflowValve(pressureTimeIncrement);
    } else if (command.equalsIgnoreCase("b-")) {
      backActuator->releasePressureFluidlyWithInputValve(pressureTimeIncrement);
    } else if (command.equalsIgnoreCase("b-test")) {
      backActuator->test();
    } else if (command.equalsIgnoreCase("le")) {
      leftActuator->extend();
    } else if (command.equalsIgnoreCase("lr")) {
      leftActuator->retract();
    } else if (command.equalsIgnoreCase("l+")) {
      leftActuator->addPressureFluidlyWithOutflowValve(pressureTimeIncrement);
    } else if (command.equalsIgnoreCase("l-")) {
      leftActuator->releasePressureFluidlyWithInputValve(pressureTimeIncrement);
    } else if (command.equalsIgnoreCase("l-test")) {
      leftActuator->test();
    } else if (command.equalsIgnoreCase("re")) {
      rightActuator->extend();
    } else if (command.equalsIgnoreCase("rr")) {
      rightActuator->retract();
    } else if (command.equalsIgnoreCase("r+")) {
      rightActuator->addPressureFluidlyWithOutflowValve(pressureTimeIncrement);
    } else if (command.equalsIgnoreCase("r-")) {
      rightActuator->releasePressureFluidlyWithInputValve(pressureTimeIncrement);
    } else if (command.equalsIgnoreCase("r-test")) {
      rightActuator->test();
    } else if (command.equalsIgnoreCase("tfe")) {
      topFrontActuator->extend();
    } else if (command.equalsIgnoreCase("tfr")) {
      topFrontActuator->retract();
    } else if (command.equalsIgnoreCase("tf+")) {
      topFrontActuator->addPressureFluidlyWithOutflowValve(pressureTimeIncrement);
    } else if (command.equalsIgnoreCase("tf-")) {
      topFrontActuator->releasePressureFluidlyWithInputValve(pressureTimeIncrement);
    } else if (command.equalsIgnoreCase("tf-test")) {
      topFrontActuator->test();
    } else if (command.equalsIgnoreCase("tbe")) {
      topBackActuator->extend();
    } else if (command.equalsIgnoreCase("tbr")) {
      topBackActuator->retract();
    } else if (command.equalsIgnoreCase("tb+")) {
      topBackActuator->addPressureFluidlyWithOutflowValve(pressureTimeIncrement);
    } else if (command.equalsIgnoreCase("tb-")) {
      topBackActuator->releasePressureFluidlyWithInputValve(pressureTimeIncrement);
    } else if (command.equalsIgnoreCase("tb-test")) {
      topBackActuator->test();
    } else if (command.equalsIgnoreCase("close-all") || command.equalsIgnoreCase("c")) {
      frontActuator->closeInput();
      frontActuator->closeOutput();
      leftActuator->closeInput();
      leftActuator->closeOutput();
      rightActuator->closeInput();
      rightActuator->closeOutput();
      backActuator->closeInput();
      backActuator->closeOutput();

      topFrontActuator->closeInput();
      topFrontActuator->closeOutput();
      topBackActuator->closeInput();
      topBackActuator->closeOutput();

      muscle->closeInput();
      muscle->closeOutput();
    } else {
      unknownCommand = true;
    }

    printPossibleCommands(&command, unknownCommand);
    unknownCommand = false;
  }
}

void ArduinoMonitorService::printPossibleCommands(String* inputCommand, bool unknownCommand) {
  Serial.println();
  if (inputCommand != nullptr) {
    Serial.println("---- Input ----");
    Serial.println(*inputCommand);
    if (unknownCommand == true) {
      Serial.println("Unknown command");
    }
  }

  Serial.println("--- Commands ---");
  Serial.println(
      "Muscle commands: 'e' - extend, 'r' - retract, '+' - pressurize valve for 50ms, '-' - depressurize valve for "
      "50ms, 'test' - tests muscles valves, 'status' - status of muscle");
  Serial.println(
      "Commands for valves on muscle: 'io' - open input valve, 'ic' - close "
      "input valve, 'oo' - "
      "open output valve, 'oc' - close output valve");
  Serial.println(
      "Commands for gyroscope (MPU6050): 'dg2/dg10/dg60' - show gyroscope output for 2s/10s/60s, 'ia' - init axis "
      "(first run "
      "dg10)");
  Serial.println(
      "Commands for feedback loop algorithms: 'stabilize' - put test stand into start position, 't70' - target 70 degrees, 't-dyn' - target 70 and then 30 degrees, "
      "'t-ant-dyn' - target -20, 0, 10 and -20 degrees, 't-ant-dyn-2-dof' - target -20;0, 0;0, 10;0 and -20;0, 't-ant-dyn-2-dof-v2' - target -20;80, 0;100, 10;80 and -20;90 with particular muscles"
      "degrees");
  Serial.println(
      "Top front actuator commands: 'tfe' - top front extend, 'tfr' - top front retract, 'tf+' - add pressure to top "
      "front, 'tf-' - "
      "remove pressure from top front, 'tf-test' - test run of top front actuators");
  Serial.println(
      "Top back actuator commands: 'tbe' - top back extend, 'tbr' - top back retract, 'tb+' - add pressure to top "
      "back, 'tb-' - "
      "remove pressure from top back, 'tb-test' - test run of top back actuators");
  Serial.println(
      "Front actuator commands: 'fe' - front extend, 'fr' - front retract, 'f+' - add pressure to front, 'f-' - "
      "remove pressure from front, 'f-test' - test run of front actuators");
  Serial.println(
      "Back actuator commands: 'be' - back extend, 'br' - back retract, 'b+' - add pressure to back, 'b-' - "
      "remove pressure from back, 'b-test' - test run of back actuators");
  Serial.println(
      "Back actuator commands: 'le' - left extend, 'lr' - left retract, 'l+' - add pressure to left, 'l-' - "
      "remove pressure from left, 'l-test' - test run of left actuators");
  Serial.println(
      "Right actuator commands: 're' - right extend, 'rr' - right retract, 'r+' - add pressure to right, 'r-' - "
      "remove pressure from right, 'r-test' - test run of right actuators");
  Serial.println("All valves commands: 'close-all/c' - close all valves");
  Serial.println("Debug tools: 'i2c' - I2C device scanner");
}

void ArduinoMonitorService::clearSerialMonitor() {
  for (int i = 0; i < 60; i++) {
    Serial.println();
  }
}