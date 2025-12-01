#include "TwoDOFAntagonisticPIDControlAlgorithm.h"

#include "./../models/ControlTarget.h"
#include "./../models/SpecificControlTarget.h"

TwoDOFAntagonisticPIDControlAlgorithm::TwoDOFAntagonisticPIDControlAlgorithm() {
  Serial.println("Creating PID 2 DOF antagonistic control algorithm class");
}

void TwoDOFAntagonisticPIDControlAlgorithm::controlMuscle(Actuator* forwardActuator, Actuator* backwardActuator,
                                                          Actuator* leftActuator, Actuator* rightActuator,
                                                          Gyroscope* gyroscope, int controlTime,
                                                          ControlTarget targets[], size_t number_of_targets) {
  // Common init vars
  const float targetTolerance = 4;
  const float valveOpenTimeClamp = 300;
  const int loopDelay = 50;  // PID update every 50ms
  unsigned long startTime = millis();

  setSpecificControlTargets(targets, number_of_targets, startTime, controlTime);

  // ----- Lateral PID init -----
  // --- PID tuning parameters ---
  const float lateralKp = 0.1f;    // Proportional gain
  const float lateralKi = 0.24f;   // Integral gain
  const float lateralKd = 0.005f;  // Derivative gain

  // --- Control setup ---
  unsigned long previousLateralTime = millis();

  float lateralIntegral = 0.0f;
  float previousLateralError = 0.0f;

  bool isFirstLateralCycle = true;

  float lateralTargetAngle = getLateralTargetAngle();

  float lateralProportionalPart = 0;
  float lateralIntegralPart = 0;
  float lateralDerivativePart = 0;
  float lateralOutput = 0;

  // ----- Longitudinal PID init -----
  const float longitudinalKp = 0.1f;    // Proportional gain
  const float longitudinalKi = 0.24f;   // Integral gain
  const float longitudinalKd = 0.005f;  // Derivative gain

  // --- Control setup ---
  unsigned long previousLongitudinalTime = millis();

  float longitudinalIntegral = 0.0f;
  float previousLongitudinalError = 0.0f;

  bool isFirstLongitudinalCycle = true;

  float longitudinalTargetAngle = getLongitudinalTargetAngle();

  float longitudinalProportionalPart = 0;
  float longitudinalIntegralPart = 0;
  float longitudinalDerivativePart = 0;
  float longitudinalOutput = 0;

  Serial.println("Starting 2 DOF PID control for 20s...");
  while (millis() - startTime < (unsigned long)controlTime) {
    unsigned long loopStartTime = millis();
    // stop command
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'c') {
        Serial.println("--- Emergency stop ---");
        forwardActuator->extend();
        backwardActuator->extend();
        delay(1500);
        break;
      }
    }

    unsigned long currentTime = millis();
    float lateralDeltaTime = (currentTime - previousLateralTime) / 1000.0f;  // seconds

    // --- Read current angle ---
    for (int i = 0; i < 30; i++) {
      gyroscope->updateValues();
    }

    // ---- Lateral control calculation part
    float currentLateralAngle = gyroscope->getYAngle();

    // --- PID calculations ---
    float lateralAngleError = lateralTargetAngle - currentLateralAngle;
    if (abs(lateralAngleError) > targetTolerance) {  // do not calculate when in target tolerance
      lateralIntegral += lateralAngleError * lateralDeltaTime;
      int clamp = 300;
      if (lateralIntegral > clamp) {
        lateralIntegral = clamp;
      } else if (lateralIntegral < -clamp) {
        lateralIntegral = -clamp;
      }

      float lateralDeltaAddition = 300;  // fix extremely big derivative caused by short time in first cycle
      if (!isFirstLateralCycle) {
        lateralDeltaAddition = 0;
      }

      float lateralDerivative = (lateralAngleError - previousLateralError) / (lateralDeltaTime + lateralDeltaAddition);
      if (lateralDerivative > clamp) {
        lateralDerivative = clamp;
      } else if (lateralDerivative < -clamp) {
        lateralDerivative = -clamp;
      }

      lateralProportionalPart = lateralKp * lateralAngleError;
      lateralIntegralPart = lateralKi * lateralIntegral;
      lateralDerivativePart = lateralKd * lateralDerivative;
      lateralOutput = lateralProportionalPart + lateralIntegralPart + lateralDerivativePart;

      if (!isFirstLateralCycle) {
        // --- Apply control ---
        if (abs(lateralAngleError) > targetTolerance && lateralOutput > 0) {
          // move forward
          if (lateralOutput > valveOpenTimeClamp) {  // upper clamp
            lateralOutput = valveOpenTimeClamp;
          }

          // Increase angle (move forward)
          forwardActuator->addPressureFluidlyWithOutflowValve(abs(lateralOutput));
          backwardActuator->releasePressureFluidlyWithInputValve((abs(lateralOutput) * 1.2));
        } else if (abs(lateralAngleError) > targetTolerance && lateralOutput < 0) {
          // move backward
          if (abs(lateralOutput) > valveOpenTimeClamp) {  // upper clamp
            lateralOutput = -valveOpenTimeClamp;
          }

          // Decrease angle (move backward)
          forwardActuator->releasePressureFluidlyWithInputValve((abs(lateralOutput) * 1.2));
          backwardActuator->addPressureFluidlyWithOutflowValve(abs(lateralOutput));
        } else {
          // Small correction area — hold position
          forwardActuator->closeInput();
          forwardActuator->closeOutput();
          backwardActuator->closeInput();
          backwardActuator->closeOutput();
        }
      }
    }

    // ---- Longitudinal control calculation part
    currentTime = millis();
    float longitudinalDeltaTime = (currentTime - previousLongitudinalTime) / 1000.0f;  // seconds
    float currentLongitudinalAngle = gyroscope->getXAngle();

    // --- PID calculations ---
    float longitudinalAngleError = longitudinalTargetAngle - currentLongitudinalAngle;
    if (abs(longitudinalAngleError) > targetTolerance) {  // do not calculate when in target tolerance
      longitudinalIntegral += longitudinalAngleError * longitudinalDeltaTime;
      int clamp = 300;
      if (longitudinalIntegral > clamp) {
        longitudinalIntegral = clamp;
      } else if (longitudinalIntegral < -clamp) {
        longitudinalIntegral = -clamp;
      }

      float longitudinalDeltaAddition = 300;  // fix extremely big derivative caused by short time in first cycle
      if (!isFirstLongitudinalCycle) {
        longitudinalDeltaAddition = 0;
      }

      float longitudinalDerivative =
          (longitudinalAngleError - previousLongitudinalError) / (longitudinalDeltaTime + longitudinalDeltaAddition);
      if (longitudinalDerivative > clamp) {
        longitudinalDerivative = clamp;
      } else if (longitudinalDerivative < -clamp) {
        longitudinalDerivative = -clamp;
      }

      longitudinalProportionalPart = longitudinalKp * longitudinalAngleError;
      longitudinalIntegralPart = longitudinalKi * longitudinalIntegral;
      longitudinalDerivativePart = longitudinalKd * longitudinalDerivative;
      longitudinalOutput = longitudinalProportionalPart + longitudinalIntegralPart + longitudinalDerivativePart;

      if (!isFirstLongitudinalCycle) {
        // --- Apply control ---
        if (abs(longitudinalAngleError) > targetTolerance && longitudinalOutput > 0) {
          // move forward
          if (longitudinalOutput > valveOpenTimeClamp) {  // upper clamp
            longitudinalOutput = valveOpenTimeClamp;
          }

          // Increase angle (move forward)
          rightActuator->addPressureFluidlyWithOutflowValve(abs(longitudinalOutput));
          leftActuator->releasePressureFluidlyWithInputValve((abs(longitudinalOutput) * 1.2));
        } else if (abs(longitudinalAngleError) > targetTolerance && longitudinalOutput < 0) {
          // move backward
          if (abs(longitudinalOutput) > valveOpenTimeClamp) {  // upper clamp
            longitudinalOutput = -valveOpenTimeClamp;
          }

          // Decrease angle (move backward)
          rightActuator->releasePressureFluidlyWithInputValve((abs(longitudinalOutput) * 1.2));
          leftActuator->addPressureFluidlyWithOutflowValve(abs(longitudinalOutput));
        } else {
          // Small correction area — hold position
          leftActuator->closeInput();
          leftActuator->closeOutput();
          rightActuator->closeInput();
          rightActuator->closeOutput();
        }
      }
    }

    // --- Debug info ---
    Serial.print("Target: ");
    Serial.print(lateralTargetAngle);
    Serial.print(" | Angle: ");
    Serial.print(currentLateralAngle);
    Serial.print(" | Error: ");
    Serial.print(lateralAngleError);
    Serial.print(" | Output: ");
    Serial.print(lateralOutput);
    Serial.print(" | Time (ms): ");
    Serial.print(currentTime);
    Serial.print(" | Prop: ");
    Serial.print(lateralProportionalPart);
    Serial.print(" | Der: ");
    Serial.print(lateralDerivativePart);
    Serial.print(" | Int: ");
    Serial.print(lateralIntegralPart);
    Serial.print(" | Long target: ");
    Serial.print(longitudinalTargetAngle);
    Serial.print(" | Long angle: ");
    Serial.print(currentLongitudinalAngle);
    Serial.print(" | Long error: ");
    Serial.print(longitudinalAngleError);
    Serial.print(" | Long output: ");
    Serial.print(longitudinalOutput);
    Serial.print(" | Long prop: ");
    Serial.print(longitudinalProportionalPart);
    Serial.print(" | Long der: ");
    Serial.print(longitudinalDerivativePart);
    Serial.print(" | Long int: ");
    Serial.print(longitudinalIntegralPart);

    // Prepare for next iteration
    previousLateralError = lateralAngleError;
    previousLateralTime = currentTime;
    previousLongitudinalError = longitudinalAngleError;
    previousLongitudinalTime = currentTime;

    delay(loopDelay);

    Serial.print(" | Loop time (ms): ");
    Serial.println(millis() - loopStartTime);

    lateralTargetAngle = getLateralTargetAngle();
    longitudinalTargetAngle = getLongitudinalTargetAngle();
    isFirstLateralCycle = false;
    isFirstLongitudinalCycle = false;
  }

  deleteTargets();
}

float TwoDOFAntagonisticPIDControlAlgorithm::getLateralTargetAngle() {
  for (size_t i = numberOfTargets; i-- > 0;) {
    if (specificControlTargets[i]->getActivationPointTime() <= millis()) {
      return specificControlTargets[i]->getLateralTargetAngle();
    }
  }

  Serial.println("Error: No target angle found, setting target to 0deg");
  return 0;
}

float TwoDOFAntagonisticPIDControlAlgorithm::getLongitudinalTargetAngle() {
  for (size_t i = numberOfTargets; i-- > 0;) {
    if (specificControlTargets[i]->getActivationPointTime() <= millis()) {
      return specificControlTargets[i]->getLongitudinalTargetAngle();
    }
  }

  Serial.println("Error: No target angle found, setting target to 0deg");
  return 0;
}

void TwoDOFAntagonisticPIDControlAlgorithm::setSpecificControlTargets(ControlTarget targets[],
                                                                      size_t numberOfControlTargets, float startTime,
                                                                      float controlTime) {
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

void TwoDOFAntagonisticPIDControlAlgorithm::deleteTargets() {
  if (specificControlTargets != nullptr) {
    for (size_t i = 0; i < numberOfTargets; ++i) {
      delete specificControlTargets[i];
    }
    delete[] specificControlTargets;
    specificControlTargets = nullptr;
  }
}

TwoDOFAntagonisticPIDControlAlgorithm::~TwoDOFAntagonisticPIDControlAlgorithm() {
  // specific control targets array cleanup
  deleteTargets();
}