#include "Arduino.h"
#include "models/Valve.h"
#include "models/Muscle.h"
#include "enums/ValveType.h"

void valveControl();
void muscleControl();

const int availableValvePins[] = {4, 5, 6, 7, 10, 11, 12, 13};

Valve* valve;
Muscle* leftMuscle;
Muscle* rightMuscle;

void setup() {
  Serial.begin(9600);

  //valve = new Valve(availableValvePins[0], ValveType::INLET);
  leftMuscle = new Muscle(availableValvePins[0], availableValvePins[1]);
  rightMuscle = new Muscle(availableValvePins[2], availableValvePins[3]);
  Serial.println();
  Serial.println("Unknown command. Type 'extend', 'retract', 'test', 'status', 'io', 'ic', 'oo', 'oc'.");
}

void loop() {
  muscleControl();
}

void muscleControl() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n'); // read until Enter
    command.trim();

    if (command.equalsIgnoreCase("extend")) {
      leftMuscle->extend();
      Serial.println(leftMuscle->getStatus());
    } 
    else if (command.equalsIgnoreCase("retract")) {
      leftMuscle->retract();
      Serial.println(leftMuscle->getStatus());
    }
    else if (command.equalsIgnoreCase("+")) {
      leftMuscle->addPressure();
      Serial.println("Pressure added");
    }
    else if (command.equalsIgnoreCase("-")) {
      leftMuscle->releasePressure();
      Serial.println("Pressure released");
    }
    else if (command.equalsIgnoreCase("io")) { // input open
      leftMuscle->openInput();
      Serial.println("Input opened");
    }
    else if (command.equalsIgnoreCase("ic")) { // input close
      leftMuscle->closeInput();
      Serial.println("Input closed");
    }
    else if (command.equalsIgnoreCase("oo")) { // output open
      leftMuscle->openOutput();
      Serial.println("Output open");
    }
    else if (command.equalsIgnoreCase("oc")) { // output close
      leftMuscle->closeOutput();
      Serial.println("Output closed");
    }
    else if (command.equalsIgnoreCase("status")) {
      Serial.println("Status:");
      Serial.println(leftMuscle->getStatus());
    }
    else if (command.equalsIgnoreCase("test")) {
      leftMuscle->test();
    }
    else {
      Serial.println("Unknown command. Type 'extend', 'retract', 'test', 'status', 'io', 'ic', 'oo', 'oc'.");
    }
    Serial.println("Type 'extend', 'retract', 'test' or 'status'.");
  }
}

// void valveControl() {
//   if (Serial.available()) {
//     String command = Serial.readStringUntil('\n'); // read until Enter
//     command.trim();

//     if (command.equalsIgnoreCase("open")) {
//       valve->open();
//       Serial.println("Valve opened");
//       Serial.println(valve->getStatus());
//     } 
//     else if (command.equalsIgnoreCase("close")) {
//       valve->close();
//       Serial.println("Valve closed");
//       Serial.println(valve->getStatus());
//     }
//     else if (command.equalsIgnoreCase("status")) {
//       Serial.println("Status:");
//       Serial.println(valve->getStatus());
//     }
//     else {
//       Serial.println("Unknown command. Type 'open', 'close' or 'status'.");
//     }
//     Serial.println("Type 'open' or 'close' to control the valve.");
//   }
// }