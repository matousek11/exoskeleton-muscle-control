#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include "./../interfaces/IValve.h"

class Actuator {
 private:
  /**
   * Inlet valves
   */
  IValve** inletValves;

  /**
   * Keeps track of how many actuators are in array
   */
  size_t inletValvesCount = 0;

  /**
   * Inlet valves
   */
  IValve** outletValves;

  /**
   * Keeps track of how many actuators are in array
   */
  size_t outletValvesCount = 0;

  bool extended;

 public:
  Actuator(IValve* inletValves[], size_t inletValvesCount, IValve* outletValves[], size_t outletValvesCount);

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

  /**
   * Add pressure from muscle for specified time but in more than one openings so transition is more fluid.
   * @param pressureTime For how long pressure should be added in miliseconds.
   */
  void addPressureFluidly(int pressureTime = 50);

  /**
   * Release pressure from muscle for specified time but in more than one openings so transition is more fluid.
   * @param pressureTime For how long pressure should be released in miliseconds.
   */
  void releasePressureFluidly(int pressureTime = 50);

  /**
   * Add pressure from muscle for specified time but in more than one openings so transition is more fluid.
   * Utilize also outflow valve to make transition more smooth.
   * @param pressureTime For how long pressure should be added in miliseconds.
   */
  void addPressureFluidlyWithOutflowValve(int pressureTime = 50);

  /**
   * Release pressure from muscle for specified time but in more than one openings so transition is more fluid.
   * Utilize also input valve to make transition more smooth.
   * @param pressureTime For how long pressure should be released in miliseconds.
   */
  void releasePressureFluidlyWithInputValve(int pressureTime = 50);

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