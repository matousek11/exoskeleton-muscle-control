#include "AntagonisticPIDControlAlgorithm.h"

#include "./../models/ControlTarget.h"
#include "./../models/SpecificControlTarget.h"

AntagonisticPIDControlAlgorithm::AntagonisticPIDControlAlgorithm() {
  Serial.println("Creating PID 1 DOF antagonistic control algorithm class");
}

void AntagonisticPIDControlAlgorithm::controlMuscle(StateMachineActuator* forwardActuator,
                                                    StateMachineActuator* backwardActuator, Gyroscope* gyroscope,
                                                    int controlTime, ControlTarget targets[], size_t number_of_targets,
                                                    Gyroscope* upperGyroscope) {
  // --- PID tuning parameters ---
  const float Kp = 0.1f;    // Proportional gain
  const float Ki = 0.24f;   // Integral gain
  const float Kd = 0.005f;  // Derivative gain

  const float targetTolerance = 4;
  const float valveOpenTimeClamp = 300;

  // --- Control setup ---
  const int loopDelay = 0;  // PID update every 50ms
  unsigned long previousTime = millis();
  unsigned long lastDataPrint = millis();

  float integral = 0.0f;
  float previousError = 0.0f;

  bool isFirstCycle = true;

  unsigned long startTime = millis();
  setSpecificControlTargets(targets, number_of_targets, startTime, controlTime);

  float targetXAngle = getTargetAngle();

  float proportionalPart = 0;
  float integralPart = 0;
  float derivativePart = 0;
  float output = 0;

  Serial.println("Starting PID control for 20s...");
  while (millis() - startTime < (unsigned long)controlTime) {
    unsigned long loopStartTime = millis();
    // stop command
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'c') {
        Serial.println("--- Emergency stop ---");
        forwardActuator->cancel();
        backwardActuator->cancel();
        forwardActuator->extend();
        backwardActuator->extend();
        delay(1500);
        break;
      }
    }

    unsigned long now = millis();
    float deltaTime = (now - previousTime) / 1000.0f;  // seconds

    // --- Read current angle ---
    if (upperGyroscope != nullptr) {
      upperGyroscope->updateValues(gyroscope);
      gyroscope->updateValues();
    } else {
      gyroscope->updateValues();
    }

    float currentAngle = 0;
    if (upperGyroscope == nullptr) {
      currentAngle = gyroscope->getXAngle();
    } else {
      currentAngle = upperGyroscope->getXAngle();
      float lowerGyroscopeAngle = gyroscope->getXAngle();
      currentAngle -= lowerGyroscopeAngle;
    }

    // --- PID calculations ---
    float error = targetXAngle - currentAngle;
    // do not calculate when in target tolerance or actuators not idle
    if (abs(error) > targetTolerance && !forwardActuator->isBusy() && !backwardActuator->isBusy()) {
      integral += error * deltaTime;
      int clamp = 300;
      if (integral > clamp) {
        integral = clamp;
      } else if (integral < -clamp) {
        integral = -clamp;
      }

      float deltaAddition = 300;  // fix extremely big derivative caused by short time in first cycle
      if (!isFirstCycle) {
        deltaAddition = 0;
      }

      float derivative = (error - previousError) / (deltaTime + deltaAddition);
      if (derivative > clamp) {
        derivative = clamp;
      } else if (derivative < -clamp) {
        derivative = -clamp;
      }

      proportionalPart = Kp * error;
      integralPart = Ki * integral;
      derivativePart = Kd * derivative;
      output = proportionalPart + integralPart + derivativePart;

      if (!isFirstCycle) {
        // --- Apply control ---
        if (abs(error) > targetTolerance && output > 0) {
          // move forward
          if (output > valveOpenTimeClamp) {  // upper clamp
            output = valveOpenTimeClamp;
          }

          // Increase angle (move forward)
          forwardActuator->startAddPressure(abs(output));
          backwardActuator->startReleasePressure((abs(output) * 1.2));
        } else if (abs(error) > targetTolerance && output < 0) {
          // move backward
          if (abs(output) > valveOpenTimeClamp) {  // upper clamp
            output = -valveOpenTimeClamp;
          }

          // Decrease angle (move backward)
          forwardActuator->startReleasePressure((abs(output) * 1.2));
          backwardActuator->startAddPressure(abs(output));
        } else {
          // Small correction area — hold position
          forwardActuator->closeInput();
          forwardActuator->closeOutput();
          backwardActuator->closeInput();
          backwardActuator->closeOutput();
        }
      }
    }

    forwardActuator->updateStateMachine();
    backwardActuator->updateStateMachine();

    if (millis() - lastDataPrint > 200) {
      // --- Debug info ---
      Serial.print("Target: ");
      Serial.print(targetXAngle);
      Serial.print(" | Angle: ");
      Serial.print(currentAngle);
      Serial.print(" | Error: ");
      Serial.print(error);
      Serial.print(" | Output: ");
      Serial.print(output);
      Serial.print(" | Time (ms): ");
      Serial.print(now);
      Serial.print(" | Prop: ");
      Serial.print(proportionalPart);
      Serial.print(" | Der: ");
      Serial.print(derivativePart);
      Serial.print(" | Int: ");
      Serial.print(integralPart);
      Serial.print(" | Loop time (ms): ");
      Serial.println(millis() - loopStartTime);

      lastDataPrint = millis();
    }

    // delay(loopDelay);

    // Prepare for next iteration
    previousError = error;
    previousTime = now;

    targetXAngle = getTargetAngle();
    isFirstCycle = false;
  }

  Serial.println(F("End of control"));
  Serial.print(F("Deleting targets, count: "));
  Serial.println(numberOfTargets);
  Serial.print(F("specificControlTargets ptr: "));
  Serial.println((unsigned long)specificControlTargets, HEX);

  deleteTargets();

  Serial.println(F("Targets deleted successfully"));
}

float AntagonisticPIDControlAlgorithm::getTargetAngle() {
  for (size_t i = numberOfTargets; i-- > 0;) {
    if (specificControlTargets[i]->getActivationPointTime() <= millis()) {
      return specificControlTargets[i]->getLateralTargetAngle();
    }
  }

  Serial.println("Error: No target angle found, setting target to 0deg");
  return 0;
}

void AntagonisticPIDControlAlgorithm::setSpecificControlTargets(ControlTarget targets[], size_t numberOfControlTargets,
                                                                float startTime, float controlTime) {
  // Check integrity of array
  float previousValue = 0;
  for (size_t i = 0; i < numberOfControlTargets; ++i) {
    float activationPoint = targets[i].getActivationPoint();
    if (previousValue > activationPoint) {
      Serial.println("Error: control targets array doesnt have ascending activation point values");
      while (true);
    }

    previousValue = activationPoint;
  }

  // Allocate array of pointers
  specificControlTargets = new SpecificControlTarget*[numberOfControlTargets];
  numberOfTargets = numberOfControlTargets;

  // Fill array with specific control targets
  for (size_t i = 0; i < numberOfControlTargets; ++i) {
    specificControlTargets[i] = targets[i].calculateSpecificControlTarget(startTime, controlTime);
  }
}

void AntagonisticPIDControlAlgorithm::deleteTargets() {
  Serial.println(F("DEBUG: deleteTargets() called"));
  if (specificControlTargets != nullptr) {
    Serial.print(F("DEBUG: Deleting "));
    Serial.print(numberOfTargets);
    Serial.println(F(" targets"));
    for (size_t i = 0; i < numberOfTargets; ++i) {
      Serial.print(F("DEBUG: Deleting target "));
      Serial.println(i);
      delete specificControlTargets[i];
    }
    Serial.println(F("DEBUG: Deleting array"));
    delete[] specificControlTargets;
    specificControlTargets = nullptr;
    Serial.println(F("DEBUG: Array deleted"));
  }
  Serial.println(F("DEBUG: deleteTargets() complete"));
}

AntagonisticPIDControlAlgorithm::~AntagonisticPIDControlAlgorithm() {
  // specific control targets array cleanup
  deleteTargets();
}