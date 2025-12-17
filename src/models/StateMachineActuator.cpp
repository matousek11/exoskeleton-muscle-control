#include "StateMachineActuator.h"

#include "../enums/ValveType.h"
#include "./../interfaces/IValve.h"

StateMachineActuator::StateMachineActuator(IValve* inletValves[], size_t inletValvesCount, IValve* outletValves[],
                                           size_t outletValvesCount) {
  this->inletValves = inletValves;
  this->inletValvesCount = inletValvesCount;
  this->outletValves = outletValves;
  this->outletValvesCount = outletValvesCount;
  extend();
  Serial.println("Initializing state machine actuator...");
  Serial.println("Actuator contains inlet valves on pins:");

  for (size_t i = 0; i < inletValvesCount; ++i) {
    Serial.println(inletValves[i]->getValvePin());
  }

  Serial.println("Actuator contains outlet valves on pins:");
  for (size_t i = 0; i < outletValvesCount; ++i) {
    Serial.println(outletValves[i]->getValvePin());
  }
}

void StateMachineActuator::transitionTo(ActionState newState) {
  actionState = newState;
  stateStartTimestamp = millis();
}

void StateMachineActuator::extend() {
  extended = true;
  for (size_t i = 0; i < inletValvesCount; ++i) {
    inletValves[i]->close();
  }

  for (size_t i = 0; i < outletValvesCount; ++i) {
    outletValves[i]->open();
  }
}

void StateMachineActuator::retract() {
  extended = false;
  for (size_t i = 0; i < outletValvesCount; ++i) {
    outletValves[i]->close();
  }

  for (size_t i = 0; i < inletValvesCount; ++i) {
    inletValves[i]->open();
  }
}

void StateMachineActuator::startAddPressure(int pressureTime) {
  // Store requested pressure time
  requestedPressureTime = pressureTime;

  // Determine mode and calculate parameters
  if (pressureTime < OPEN_TIME) {
    // Short mode
    if (pressureTime < SHORT_THRESHOLD) {
      // Very short: use drain valve
      useShortDrain = true;
      drainOpenTime = OPEN_TIME + (pressureTime / 5);  // Integer math
      calculatedPressureTime = pressureTime;
    } else {
      // Short but not very short: extend pressure time
      useShortDrain = false;
      calculatedPressureTime = pressureTime + OPEN_TIME;
    }
    remainingOpenings = 0;  // Not using loop mode
  } else {
    // Loop mode
    useShortDrain = false;
    remainingOpenings = pressureTime / OPEN_TIME;
    calculatedPressureTime = pressureTime;
  }

  // Start operation: close output valve first (sealing phase)
  operationType = OP_PRESSURIZE;
  closeOutput();
  transitionTo(STATE_SEALING);
}

void StateMachineActuator::startReleasePressure(int pressureTime) {
  // Store requested pressure time
  requestedPressureTime = pressureTime;

  // Determine mode and calculate parameters
  if (pressureTime < OPEN_TIME) {
    // Short mode
    if (pressureTime < SHORT_THRESHOLD) {
      // Very short: use drain valve (input in this case)
      useShortDrain = true;
      drainOpenTime = OPEN_TIME + (pressureTime / 5);  // Integer math
      calculatedPressureTime = pressureTime;
    } else {
      // Short but not very short: extend pressure time
      useShortDrain = false;
      calculatedPressureTime = pressureTime + OPEN_TIME;
    }
    remainingOpenings = 0;  // Not using loop mode
  } else {
    // Loop mode
    useShortDrain = false;
    remainingOpenings = pressureTime / OPEN_TIME;
    calculatedPressureTime = pressureTime;
  }

  // Start operation: close input valve first (sealing phase)
  operationType = OP_DEPRESSURIZE;
  closeInput();
  transitionTo(STATE_SEALING);
}

bool StateMachineActuator::updateStateMachine() {
  if (operationType == OP_IDLE) {
    return false;  // Nothing to do
  }

  unsigned long now = millis();
  unsigned long elapsed = now - stateStartTimestamp;
  
  // Use >= for timing comparisons to handle loop timing variations
  // This ensures we don't miss timing windows even with variable loop times

  if (operationType == OP_PRESSURIZE) {
    switch (actionState) {
      case STATE_SEALING:
        // Wait for sealing time, then decide short vs loop mode
        if (elapsed >= MUSCLE_SEALING_TIME) {
          if (requestedPressureTime < OPEN_TIME) {
            // Short mode: open input, optionally open output for drain
            openInput();
            if (useShortDrain) {
              openOutput();  // Will close after drainOpenTime
            }
            transitionTo(STATE_SHORT_MAIN_OPEN);
          } else {
            // Loop mode: open input
            openInput();
            transitionTo(STATE_LOOP_MAIN_OPEN);
          }
        }
        break;

      case STATE_SHORT_MAIN_OPEN:
        // In short mode with drain: wait for drain time, then close output
        if (useShortDrain) {
          if (elapsed >= (unsigned long)drainOpenTime) {
            closeOutput();
            transitionTo(STATE_SHORT_WAIT);
          }
        } else {
          // No drain, go directly to wait for calculatedPressureTime
          if (elapsed >= (unsigned long)calculatedPressureTime) {
            closeInput();
            operationType = OP_IDLE;
            transitionTo(STATE_IDLE);
          }
        }
        break;

      case STATE_SHORT_WAIT:
        // Wait for remaining pressure time after drain closed
        if (elapsed >= (unsigned long)calculatedPressureTime) {
          closeInput();
          operationType = OP_IDLE;
          transitionTo(STATE_IDLE);
        }
        break;

      case STATE_LOOP_MAIN_OPEN:
        // Input is open, wait for OPEN_TIME
        if (elapsed >= OPEN_TIME) {
          closeInput();
          remainingOpenings--;
          if (remainingOpenings <= 0) {
            // Done with all openings
            operationType = OP_IDLE;
            transitionTo(STATE_IDLE);
          } else {
            // More openings to do, wait sealing time
            transitionTo(STATE_LOOP_MAIN_CLOSED);
          }
        }
        break;

      case STATE_LOOP_MAIN_CLOSED:
        // Input is closed, wait for sealing time before next opening
        if (elapsed >= MUSCLE_SEALING_TIME) {
          openInput();
          transitionTo(STATE_LOOP_MAIN_OPEN);
        }
        break;

      default:
        break;
    }
  } else if (operationType == OP_DEPRESSURIZE) {
    switch (actionState) {
      case STATE_SEALING:
        // Wait for sealing time, then decide short vs loop mode
        if (elapsed >= MUSCLE_SEALING_TIME) {
          if (requestedPressureTime < OPEN_TIME) {
            // Short mode: open output, optionally open input for drain
            openOutput();
            if (useShortDrain) {
              openInput();  // Will close after drainOpenTime
            }
            transitionTo(STATE_SHORT_MAIN_OPEN);
          } else {
            // Loop mode: open output
            openOutput();
            transitionTo(STATE_LOOP_MAIN_OPEN);
          }
        }
        break;

      case STATE_SHORT_MAIN_OPEN:
        // In short mode with drain: wait for drain time, then close input
        if (useShortDrain) {
          if (elapsed >= (unsigned long)drainOpenTime) {
            closeInput();
            transitionTo(STATE_SHORT_WAIT);
          }
        } else {
          // No drain, go directly to wait for calculatedPressureTime
          if (elapsed >= (unsigned long)calculatedPressureTime) {
            closeOutput();
            operationType = OP_IDLE;
            transitionTo(STATE_IDLE);
          }
        }
        break;

      case STATE_SHORT_WAIT:
        // Wait for remaining pressure time after drain closed
        if (elapsed >= (unsigned long)calculatedPressureTime) {
          closeOutput();
          operationType = OP_IDLE;
          transitionTo(STATE_IDLE);
        }
        break;

      case STATE_LOOP_MAIN_OPEN:
        // Output is open, wait for OPEN_TIME
        if (elapsed >= OPEN_TIME) {
          closeOutput();
          remainingOpenings--;
          if (remainingOpenings <= 0) {
            // Done with all openings
            operationType = OP_IDLE;
            transitionTo(STATE_IDLE);
          } else {
            // More openings to do, wait sealing time
            transitionTo(STATE_LOOP_MAIN_CLOSED);
          }
        }
        break;

      case STATE_LOOP_MAIN_CLOSED:
        // Output is closed, wait for sealing time before next opening
        if (elapsed >= MUSCLE_SEALING_TIME) {
          openOutput();
          transitionTo(STATE_LOOP_MAIN_OPEN);
        }
        break;

      default:
        break;
    }
  }

  return operationType != OP_IDLE;
}

bool StateMachineActuator::isBusy() {
  return operationType != OP_IDLE;
}

void StateMachineActuator::cancel() {
  closeInput();
  closeOutput();
  operationType = OP_IDLE;
  actionState = STATE_IDLE;
}

void StateMachineActuator::openInput() {
  for (size_t i = 0; i < inletValvesCount; ++i) {
    inletValves[i]->open();
  }
}

void StateMachineActuator::closeInput() {
  for (size_t i = 0; i < inletValvesCount; ++i) {
    inletValves[i]->close();
  }
}

void StateMachineActuator::openOutput() {
  for (size_t i = 0; i < outletValvesCount; ++i) {
    outletValves[i]->open();
  }
}

void StateMachineActuator::closeOutput() {
  for (size_t i = 0; i < outletValvesCount; ++i) {
    outletValves[i]->close();
  }
}

bool StateMachineActuator::isExtended() {
  return extended;
}

String StateMachineActuator::getStatus() {
  return String("status: ") + (isExtended() ? "Muscle extended" : "Muscle retracted");
}

StateMachineActuator::OperationType StateMachineActuator::getOperationType() {
  return operationType;
}

StateMachineActuator::ActionState StateMachineActuator::getActionState() {
  return actionState;
}

void StateMachineActuator::test() {
  Serial.println("Muscle test");

  Serial.println("Retracting Muscle");
  delay(1000);
  retract();

  delay(3000);
  Serial.println("Extending Muscle");
  delay(1000);
  extend();

  delay(3000);
  Serial.println("Retracting Muscle");
  delay(1000);
  retract();

  delay(3000);
  Serial.println("Extending Muscle");
  delay(1000);
  extend();
}
