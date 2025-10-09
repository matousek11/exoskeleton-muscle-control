#ifndef MUSCLE_H_
#define MUSCLE_H_

#include "./../interfaces/IValve.h"
#include "Arduino.h"
#include "Valve.h"

class Muscle {
 private:
  IValve* inputValve;
  IValve* outputValve;

 public:
  Muscle(IValve* inputValve, IValve* outputValve);

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