#include "ArduinoMonitorService.h"

#include "./../control-algorithms/PIDControlAlgorithm.h"
#include "./../helpers/Debugger.h"
#include "./../interfaces/IControlAlgorithm.h"
#include "./../models/Muscle.h"
#include "Arduino.h"

void ArduinoMonitorService::controlThroughMonitor(Muscle* muscle, Gyroscope* gyroscope,
                                                  IControlAlgorithm* controlAlgorithm) {
  bool unknownCommand = false;

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // read until Enter
    command.trim();
    clearSerialMonitor();
    Serial.println("---- Output ----");

    if (command.equalsIgnoreCase("e")) {
      muscle->extend();
      Serial.println(muscle->getStatus());
    } else if (command.equalsIgnoreCase("r")) {
      muscle->retract();
      Serial.println(muscle->getStatus());
    } else if (command.equalsIgnoreCase("+")) {
      muscle->addPressure();
      Serial.println("Pressure added");
    } else if (command.equalsIgnoreCase("-")) {
      muscle->releasePressure();
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
      controlAlgorithm->controlMuscle(muscle, gyroscope, 20000);
      muscle->extend();
    } else if (command.equalsIgnoreCase("i2c")) {
      Debugger::scanI2C();
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
      "Commands for gyroscope (MPU6050): 'dg10/dg60' - show gyroscope output for 10s/60s, 'ia' - init axis (first run "
      "dg10)");
  Serial.println("Commands for feedback loop algorithms: 't70' - target 70 degrees");
  Serial.println("Debug tools: 'i2c' - I2C device scanner");
}

void ArduinoMonitorService::clearSerialMonitor() {
  for (int i = 0; i < 60; i++) {
    Serial.println();
  }
}