#ifndef MUSCLE_H_
#define MUSCLE_H_

#include "./../interfaces/IValve.h"
#include "Arduino.h"
#include "Valve.h"

/**
 * Class that makes abstraction above input and output valves through which muscle is formed.
 * Valves are linked together as one muscle.
 */
class Muscle {
 private:
  /**
   * Valve used for inflow of pressure.
   */
  IValve* inputValve;

  /**
   * Valve used for outflow of pressure.
   */
  IValve* outputValve;

 public:
  Muscle(IValve* inputValve, IValve* outputValve);

  /**
   * Fully release pressure from muscle.
   */
  void extend();

  /**
   * Pressurize muscle.
   */
  void retract();

  /**
   * @param pressureTime For how long pressure should be added in miliseconds.
   */
  void addPressure(int pressureTime = 50);

  /**
   * @param pressureTime For how long pressure should be released in miliseconds.
   */
  void releasePressure(int pressureTime = 50);
  void openInput();
  void closeInput();
  void openOutput();
  void closeOutput();

  /**
   * @return true when muscle is depresurized.
   */
  bool isExtended();

  /**
   * Prints current settings of muscle.
   */
  String getStatus();

  /**
   * Test valves by opening and closing them for several times.
   */
  void test();
};

#endif