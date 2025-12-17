#ifndef STATEMACHINEACTUATOR_H_
#define STATEMACHINEACTUATOR_H_

#include "./../interfaces/IValve.h"

class StateMachineActuator {
 public:
  // Timing constants - designed for 20ms loop resolution
  // All times should be multiples of expected loop time for accurate timing
  static const int MUSCLE_SEALING_TIME = 5;
  static const int OPEN_TIME = 25;
  static const int SHORT_THRESHOLD = 5;

  // Operation type - what the actuator is doing
  enum OperationType { OP_IDLE, OP_PRESSURIZE, OP_DEPRESSURIZE };

  // State within the operation
  enum ActionState {
    STATE_IDLE,
    STATE_SEALING,          // Closing opposing valve, waiting seal time
    STATE_SHORT_MAIN_OPEN,  // Short mode: main valve open (and drain if very short)
    STATE_SHORT_WAIT,       // Short mode: waiting remaining pressure time after drain closed
    STATE_LOOP_MAIN_OPEN,   // Loop mode: main valve open for OPEN_TIME
    STATE_LOOP_MAIN_CLOSED  // Loop mode: main valve closed, waiting seal time
  };

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
   * Outlet valves
   */
  IValve** outletValves;

  /**
   * Keeps track of how many actuators are in array
   */
  size_t outletValvesCount = 0;

  bool extended;

  // State machine variables
  OperationType operationType = OP_IDLE;
  ActionState actionState = STATE_IDLE;
  unsigned long stateStartTimestamp = 0;
  int requestedPressureTime = 0;
  int calculatedPressureTime = 0;
  int drainOpenTime = 0;
  int remainingOpenings = 0;
  bool useShortDrain = false;

  /**
   * Helper to transition to a new state and record timestamp
   */
  void transitionTo(ActionState newState);

 public:
  StateMachineActuator(IValve* inletValves[], size_t inletValvesCount, IValve* outletValves[],
                       size_t outletValvesCount);

  /**
   * Fully release pressure from muscle.
   */
  void extend();

  /**
   * Pressurize muscle.
   */
  void retract();

  /**
   * Start non-blocking pressure addition.
   * Utilize also outflow valve to make transition more smooth.
   * Call updateStateMachine() in your loop to progress the operation.
   * @param pressureTime For how long pressure should be added in milliseconds.
   */
  void startAddPressure(int pressureTime = 50);

  /**
   * Start non-blocking pressure release.
   * Utilize also input valve to make transition more smooth.
   * Call updateStateMachine() in your loop to progress the operation.
   * @param pressureTime For how long pressure should be released in milliseconds.
   */
  void startReleasePressure(int pressureTime = 50);

  /**
   * Updates state machine of actuator. Call this in your main loop.
   * @return true if operation is still in progress, false if idle/complete
   */
  bool updateStateMachine();

  /**
   * Check if actuator is currently performing an operation
   * @return true if busy, false if idle
   */
  bool isBusy();

  /**
   * Cancel current operation and return to idle state.
   * Closes all valves.
   */
  void cancel();

  void openInput();
  void closeInput();
  void openOutput();
  void closeOutput();

  /**
   * @return true when muscle is depressurized.
   */
  bool isExtended();

  /**
   * Prints current settings of muscle.
   */
  String getStatus();

  /**
   * Get current operation type
   */
  OperationType getOperationType();

  /**
   * Get current action state
   */
  ActionState getActionState();

  /**
   * Test valves by opening and closing them for several times.
   */
  void test();
};

#endif