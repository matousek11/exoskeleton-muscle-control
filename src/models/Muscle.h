#ifndef MUSCLE_H_
#define MUSCLE_H_

#include "Arduino.h"
#include "Valve.h"

class Muscle {
 private:
  Valve inputValve;
  Valve outputValve;

 public:
  Muscle(int inputValvePin, int outputValvePin);

  void extend();
  void retract();
  void addPressure();
  void releasePressure();
  void openInput();
  void closeInput();
  void openOutput();
  void closeOutput();
  bool isExtended();
  String getStatus();
  void test();
};

#endif