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
Adafruit_MotorShield AFMS1 = Adafruit_MotorShield(0x60);
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield(0x61);
Adafruit_MotorShield AFMS3 = Adafruit_MotorShield(0x62);
Adafruit_DCMotor* myMotor1 = AFMS1.getMotor(1);
Adafruit_DCMotor* myMotor2 = AFMS1.getMotor(2);
Adafruit_DCMotor* myMotor3 = AFMS1.getMotor(3);
Adafruit_DCMotor* myMotor4 = AFMS1.getMotor(4);
Adafruit_DCMotor* myMotor5 = AFMS2.getMotor(1);
Adafruit_DCMotor* myMotor6 = AFMS3.getMotor(4);

void setup() {
  Serial.begin(9600);

  if (!AFMS1.begin()) {  // create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield on address 0x60. Check wiring.");
    while (1);
  }

  if (!AFMS2.begin()) {  // create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield on address 0x62. Check wiring.");
    while (1);
  }

  if (!AFMS3.begin()) {  // create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield on address 0x62. Check wiring.");
    while (1);
  }

  myMotor1->setSpeed(255);
  myMotor2->setSpeed(255);
  myMotor3->setSpeed(255);
  myMotor4->setSpeed(255);
  myMotor5->setSpeed(255);
  myMotor6->setSpeed(255);
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
  myMotor1->run(FORWARD);  // open
  Serial.println("opening valve 1");
  delay(3000);            
  // myMotor1->run(RELEASE);  // close
  // Serial.println("closing valve 1");
  delay(3000);

  myMotor2->run(FORWARD);  // open
  Serial.println("opening valve 2");
  delay(3000);            
  // myMotor2->run(RELEASE);  // close
  // Serial.println("closing valve 2");
  delay(3000);

  myMotor3->run(FORWARD);  // open
  Serial.println("opening valve 3");
  delay(3000);            
  // myMotor3->run(RELEASE);  // close
  // Serial.println("closing valve 3");
  delay(3000);

  myMotor4->run(FORWARD);  // open
  Serial.println("opening valve 4");
  delay(3000);            
  // myMotor4->run(RELEASE);  // close
  // Serial.println("closing valve 4");
  delay(3000);

  myMotor5->run(FORWARD);  // open
  Serial.println("opening valve 5");
  delay(3000);            
  // myMotor5->run(RELEASE);  // close
  // Serial.println("closing valve 5");
  delay(3000);

  myMotor6->run(FORWARD);  // open
  Serial.println("opening valve 6");
  delay(3000);            
  myMotor6->run(RELEASE);  // close
  Serial.println("closing valve 6");
  delay(3000);
}