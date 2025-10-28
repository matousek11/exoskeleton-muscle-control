#include "PIDControlAlgorithm.h"

#include "./../models/ControlTarget.h"
#include "./../models/SpecificControlTarget.h"

PIDControlAlgorithm::PIDControlAlgorithm() {
  Serial.println("Creatin PID control algorithm class");
}

void PIDControlAlgorithm::controlMuscle(Muscle* muscle, Gyroscope* gyroscope, int controlTime, ControlTarget targets[],
                                        size_t number_of_targets) {
  // --- PID tuning parameters ---
  const float Kp = 0.4f;   // Proportional gain
  const float Ki = 0.02f;  // Integral gain
  const float Kd = 0.04f;   // Derivative gain

  const float targetTolerance = 4;
  const float valveOpenTimeClamp = 300;

  // --- Control setup ---
  const int loopDelay = 50;  // PID update every 50ms
  unsigned long previousTime = millis();

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
        muscle->extend();
        break;
      }
    }

    unsigned long now = millis();
    float deltaTime = (now - previousTime) / 1000.0f;  // seconds

    // --- Read current angle ---
    for (int i = 0; i < 30; i++) {
      gyroscope->updateValues();
    }
    float currentAngle = gyroscope->getXAngle();

    // --- PID calculations ---
    float error = targetXAngle - currentAngle;
    if (abs(error) > targetTolerance) {  // do not calculate when in target tolerance
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
        if (abs(error) > targetTolerance && output > targetTolerance) {
          if (output > valveOpenTimeClamp) {  // upper clamp
            output = valveOpenTimeClamp;
          }

          muscle->addPressureFluidly(abs(output));  // Increase angle
        } else if (abs(error) > targetTolerance && output < -targetTolerance) {
          if (abs(output) > valveOpenTimeClamp) {  // upper clamp
            output = -valveOpenTimeClamp;
          }

          muscle->releasePressureFluidly(abs(output));  // Decrease angle
        } else {
          // Small correction area — hold position
          muscle->closeInput();
          muscle->closeOutput();
        }
      }
    }

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

    // Prepare for next iteration
    previousError = error;
    previousTime = now;

    delay(loopDelay);

    Serial.print(" | Loop time (ms): ");
    Serial.println(millis() - loopStartTime);

    targetXAngle = getTargetAngle();
    isFirstCycle = false;
  }

  muscle->extend();
  deleteTargets();
}

float PIDControlAlgorithm::getTargetAngle() {
  for (size_t i = numberOfTargets; i-- > 0;) {
    if (specificControlTargets[i]->getActivationPointTime() <= millis()) {
      return specificControlTargets[i]->getTargetAngle();
    }
  }

  Serial.println("Error: No target angle found, setting target to 0deg");
  return 0;
}

void PIDControlAlgorithm::setSpecificControlTargets(ControlTarget targets[], size_t numberOfControlTargets,
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

void PIDControlAlgorithm::deleteTargets() {
  if (specificControlTargets != nullptr) {
    for (size_t i = 0; i < numberOfTargets; ++i) {
      delete specificControlTargets[i];
    }
    delete[] specificControlTargets;
    specificControlTargets = nullptr;
  }
}

PIDControlAlgorithm::~PIDControlAlgorithm() {
  // specific control targets array cleanup
  deleteTargets();
}