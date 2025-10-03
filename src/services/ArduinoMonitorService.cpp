#include "Arduino.h"
#include "ArduinoMonitorService.h"
#include "./../models/Muscle.h"

void ArduinoMonitorService::controlThroughMonitor(Muscle* muscle) {
    bool unknownCommand = false;

    if (Serial.available()) {
        String command = Serial.readStringUntil('\n'); // read until Enter
        command.trim();
        clearSerialMonitor();
        Serial.println("---- Output ----");

        if (command.equalsIgnoreCase("e")) {
            muscle->extend();
            Serial.println(muscle->getStatus());
        } 
        else if (command.equalsIgnoreCase("r")) {
            muscle->retract();
            Serial.println(muscle->getStatus());
        }
        else if (command.equalsIgnoreCase("+")) {
            muscle->addPressure();
            Serial.println("Pressure added");
        }
        else if (command.equalsIgnoreCase("-")) {
            muscle->releasePressure();
            Serial.println("Pressure released");
        }
        else if (command.equalsIgnoreCase("io")) {
            muscle->openInput();
            Serial.println("Input opened");
        }
        else if (command.equalsIgnoreCase("ic")) {
            muscle->closeInput();
            Serial.println("Input closed");
        }
        else if (command.equalsIgnoreCase("oo")) {
            muscle->openOutput();
            Serial.println("Output open");
        }
        else if (command.equalsIgnoreCase("oc")) {
            muscle->closeOutput();
            Serial.println("Output closed");
        }
        else if (command.equalsIgnoreCase("status")) {
            Serial.println(muscle->getStatus());
        }
        else if (command.equalsIgnoreCase("test")) {
            muscle->test();
        }
        else {
            unknownCommand = true;
        }

        printPossibleCommands(&command, unknownCommand);
        unknownCommand = false;
    }
}

void ArduinoMonitorService::printPossibleCommands(
    String* inputCommand = nullptr,
    bool unknownCommand = false
) {
    Serial.println();
    if (inputCommand != nullptr) {
        Serial.println("---- Input ----");
        Serial.println(*inputCommand);
        if (unknownCommand == true) {
            Serial.println("Unknown command");
        }
    }

    Serial.println("--- Commands ---");
    Serial.println("Muscle commands: 'e' - extend, 'r' - retract, 'test', 'status'");
    Serial.println("Commands for valves on muscle: 'io' - open input valve, 'ic' - close input valve, 'oo' - open output valve, 'oc' - close output valve");
}

void ArduinoMonitorService::clearSerialMonitor() {
    for (int i = 0; i < 60; i++) {
        Serial.println();
    }
}