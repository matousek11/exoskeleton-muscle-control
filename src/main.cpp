#include "Adafruit_MotorShield.h"
#include "Arduino.h"
#include "Wire.h"
#include "enums/ValveType.h"
#include "models/Muscle.h"
#include "models/Valve.h"
#include "services/ArduinoMonitorService.h"

const int availableValvePins[] = {4, 5, 6, 7, 10, 11, 12, 13};

Muscle* leftMuscle;
ArduinoMonitorService* arduinoMonitorService;
Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x60);
Adafruit_DCMotor* myMotor = AFMS.getMotor(1);

void setup() {
  Serial.begin(9600);

  if (!AFMS.begin()) {  // create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }

  // myMotor->setSpeed(150);
  // myMotor->run(FORWARD);
  // // turn on motor
  // myMotor->run(RELEASE);

  // arduinoMonitorService = new ArduinoMonitorService();
  // leftMuscle = new Muscle(availableValvePins[0], availableValvePins[1]);
  // arduinoMonitorService->printPossibleCommands(nullptr);
}

void loop() {
  // arduinoMonitorService->controlThroughMonitor(leftMuscle);
  myMotor->run(FORWARD);  // zapne solenoid
  delay(1000);            // drží 1 sekundu
  myMotor->run(RELEASE);  // vypne solenoid
  delay(1000);
}