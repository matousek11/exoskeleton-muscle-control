#include "ThreeDOFAntagonisticParticularMuscleControlAlgorithm.h"

#include "./../models/ControlTarget.h"
#include "./../models/SpecificControlTarget.h"

ThreeDOFAntagonisticParticularMuscleControlAlgorithm::ThreeDOFAntagonisticParticularMuscleControlAlgorithm() {
  Serial.println("Creating PID 3 DOF antagonistic particular muscle control algorithm class");
}

void ThreeDOFAntagonisticParticularMuscleControlAlgorithm::controlMuscle(
    Actuator* leftBottomFrontActuator, Actuator* rightBottomFrontActuator, Actuator* leftBottomBackActuator,
    Actuator* rightBottomBackActuator, Actuator* topFrontActuator, Actuator* topBackActuator, Gyroscope* lowerGyroscope,
    Gyroscope* upperGyroscope, unsigned long controlTime, ControlTarget lowerLegTargets[],
    size_t numberOfLowerLegTargets, ControlTarget upperLegTargets[], size_t numberOfUpperLegTargets) {
  // Common init vars
  const float lateralTargetTolerance = 4;
  const float longitudinalTargetTolerance = 6;
  const float valveOpenTimeClamp = 300;
  const int loopDelay = 50;  // PID update every 50ms
  unsigned long startTime = millis();

  setSpecificControlTargets(lowerLegTargets, numberOfLowerLegTargets, startTime, controlTime);
  setSpecificControlTargets(upperLegTargets, numberOfUpperLegTargets, startTime, controlTime, false);

  // ----- Lateral PID init -----
  // --- PID tuning parameters ---
  const float lateralKp = 0.1f;    // Proportional gain
  const float lateralKi = 0.28f;   // Integral gain
  const float lateralKd = 0.005f;  // Derivative gain

  // --- Control setup ---
  unsigned long previousLateralTime = millis();

  float lateralIntegral = 0.0f;
  float previousLateralError = 0.0f;

  bool isFirstLateralCycle = true;

  float lateralTargetAngle = getLowerLegLateralTargetAngle();

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

  float longitudinalTargetAngle = getLowerLegLongitudinalTargetAngle();

  float longitudinalProportionalPart = 0;
  float longitudinalIntegralPart = 0;
  float longitudinalDerivativePart = 0;
  float longitudinalOutput = 0;

  // ----- Upper leg lateral PID init -----
  // --- PID tuning parameters ---
  const float upperLegLateralKp = 0.1f;    // Proportional gain
  const float upperLegLateralKi = 0.28f;   // Integral gain
  const float upperLegLateralKd = 0.005f;  // Derivative gain

  // --- Control setup ---
  unsigned long previousUpperLegLateralTime = millis();

  float upperLegLateralIntegral = 0.0f;
  float previousUpperLegLateralError = 0.0f;

  bool isFirstUpperLegLateralCycle = true;

  float upperLegLateralTargetAngle = getUpperLegLateralTargetAngle();

  float upperLegLateralProportionalPart = 0;
  float upperLegLateralIntegralPart = 0;
  float upperLegLateralDerivativePart = 0;
  float upperLegLateralOutput = 0;

  Serial.println("Starting 3 DOF particular muscle PID control for 20s...");
  while (millis() - startTime < (unsigned long)controlTime) {
    unsigned long loopStartTime = millis();
    int leftFrontActuatorOutput = 0;
    int rightFrontActuatorOutput = 0;
    int leftBackActuatorOutput = 0;
    int rightBackActuatorOutput = 0;

    // stop command
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'c') {
        Serial.println("--- Emergency stop ---");
        leftBottomFrontActuator->extend();
        rightBottomFrontActuator->extend();
        leftBottomBackActuator->extend();
        rightBottomBackActuator->extend();
        topFrontActuator->extend();
        topBackActuator->extend();
        delay(1500);
        break;
      }
    }

    unsigned long currentTime = millis();
    float lateralDeltaTime = (currentTime - previousLateralTime) / 1000.0f;  // seconds

    // --- Read current angle ---
    for (int i = 0; i < 30; i++) {
      lowerGyroscope->updateValues();
      upperGyroscope->updateValues();
    }

    // ---- Lateral control calculation part
    float currentLowerLegLateralAngle = lowerGyroscope->getYAngle();

    // --- PID calculations ---
    float lateralAngleError = lateralTargetAngle - currentLowerLegLateralAngle;
    if (abs(lateralAngleError) > lateralTargetTolerance) {  // do not calculate when in target tolerance
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
        if (abs(lateralAngleError) > lateralTargetTolerance && lateralOutput > 0) {
          // move forward
          if (lateralOutput > valveOpenTimeClamp) {  // upper clamp
            lateralOutput = valveOpenTimeClamp;
          }

          // Increase angle (move forward)
          leftFrontActuatorOutput += abs(lateralOutput);
          rightFrontActuatorOutput += abs(lateralOutput);

          leftBackActuatorOutput -= abs(lateralOutput) * 1.2;
          rightBackActuatorOutput -= abs(lateralOutput) * 1.2;
        } else if (abs(lateralAngleError) > lateralTargetTolerance && lateralOutput < 0) {
          // move backward
          if (abs(lateralOutput) > valveOpenTimeClamp) {  // upper clamp
            lateralOutput = -valveOpenTimeClamp;
          }

          // Decrease angle (move backward)
          leftFrontActuatorOutput -= abs(lateralOutput) * 1.2;
          rightFrontActuatorOutput -= abs(lateralOutput) * 1.2;

          leftBackActuatorOutput += abs(lateralOutput);
          rightBackActuatorOutput += abs(lateralOutput);
        }
      }
    }

    // ---- Longitudinal control calculation part
    currentTime = millis();
    float longitudinalDeltaTime = (currentTime - previousLongitudinalTime) / 1000.0f;  // seconds
    float currentLowerLegLongitudinalAngle = lowerGyroscope->getXAngle();

    // --- PID calculations ---
    float longitudinalAngleError = longitudinalTargetAngle - currentLowerLegLongitudinalAngle;
    if (abs(longitudinalAngleError) > longitudinalTargetTolerance) {  // do not calculate when in target tolerance
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
        if (abs(longitudinalAngleError) > longitudinalTargetTolerance && longitudinalOutput > 0) {
          // move right
          if (longitudinalOutput > valveOpenTimeClamp) {  // upper clamp
            longitudinalOutput = valveOpenTimeClamp;
          }

          // Increase angle (move right)
          rightFrontActuatorOutput += abs(longitudinalOutput);
          rightBackActuatorOutput += abs(longitudinalOutput);
          // counter changes for another axis
          if (lateralAngleError > 0) {
            leftFrontActuatorOutput += abs(longitudinalOutput) * 0.4;
          } else if (lateralAngleError < -0) {
            leftBackActuatorOutput += abs(longitudinalOutput) * 0.4;
          }

          leftFrontActuatorOutput -= abs(longitudinalOutput) * 1.1;
          leftBackActuatorOutput -= abs(longitudinalOutput) * 1.1;
        } else if (abs(longitudinalAngleError) > longitudinalTargetTolerance && longitudinalOutput < 0) {
          // move left
          if (abs(longitudinalOutput) > valveOpenTimeClamp) {  // upper clamp
            longitudinalOutput = -valveOpenTimeClamp;
          }

          // Decrease angle (move left)
          rightFrontActuatorOutput -= abs(longitudinalOutput) * 1.2;
          rightBackActuatorOutput -= abs(longitudinalOutput) * 1.2;
          // counter changes for another axis
          if (lateralAngleError > 0) {
            rightFrontActuatorOutput += abs(longitudinalOutput) * 0.4;
          } else if (lateralAngleError < 0) {
            rightBackActuatorOutput += abs(longitudinalOutput) * 0.4;
          }

          leftFrontActuatorOutput += abs(longitudinalOutput);
          leftBackActuatorOutput += abs(longitudinalOutput);
        }
      }
    }

    if (leftFrontActuatorOutput != 0) {
      leftFrontActuatorOutput > 0
          ? leftBottomFrontActuator->addPressureFluidlyWithOutflowValve(leftFrontActuatorOutput)
          : leftBottomFrontActuator->releasePressureFluidlyWithInputValve(abs(leftFrontActuatorOutput));
    }

    if (rightFrontActuatorOutput != 0) {
      rightFrontActuatorOutput > 0
          ? rightBottomFrontActuator->addPressureFluidlyWithOutflowValve(rightFrontActuatorOutput)
          : rightBottomFrontActuator->releasePressureFluidlyWithInputValve(abs(rightFrontActuatorOutput));
    }

    if (leftBackActuatorOutput != 0) {
      leftBackActuatorOutput > 0
          ? leftBottomBackActuator->addPressureFluidlyWithOutflowValve(leftBackActuatorOutput)
          : leftBottomBackActuator->releasePressureFluidlyWithInputValve(abs(leftBackActuatorOutput));
    }

    if (rightBackActuatorOutput != 0) {
      rightBackActuatorOutput > 0
          ? rightBottomBackActuator->addPressureFluidlyWithOutflowValve(rightBackActuatorOutput)
          : rightBottomBackActuator->releasePressureFluidlyWithInputValve(abs(rightBackActuatorOutput));
    }

    // --- Read current angle ---
    for (int i = 0; i < 5; i++) {
      lowerGyroscope->updateValues();
      upperGyroscope->updateValues();
    }

    // ---- Upper leg lateral control calculation part
    currentTime = millis();
    float upperLegLateralDeltaTime = (currentTime - previousUpperLegLateralTime) / 1000.0f;  // seconds
    float currentUpperLegLateralAngle = upperGyroscope->getYAngle(lowerGyroscope);

    // --- PID calculations ---
    float upperLegLateralAngleError = upperLegLateralTargetAngle - currentUpperLegLateralAngle;
    if (abs(upperLegLateralAngleError) > lateralTargetTolerance) {  // do not calculate when in target tolerance
      upperLegLateralIntegral += upperLegLateralAngleError * upperLegLateralDeltaTime;
      int clamp = 300;
      if (upperLegLateralIntegral > clamp) {
        upperLegLateralIntegral = clamp;
      } else if (upperLegLateralIntegral < -clamp) {
        upperLegLateralIntegral = -clamp;
      }

      float upperLegLateralDeltaAddition = 300;  // fix extremely big derivative caused by short time in first cycle
      if (!isFirstUpperLegLateralCycle) {
        upperLegLateralDeltaAddition = 0;
      }

      float upperLegLateralDerivative = (upperLegLateralAngleError - previousUpperLegLateralError) /
                                        (lateralDeltaTime + upperLegLateralDeltaAddition);
      if (upperLegLateralDerivative > clamp) {
        upperLegLateralDerivative = clamp;
      } else if (upperLegLateralDerivative < -clamp) {
        upperLegLateralDerivative = -clamp;
      }

      upperLegLateralProportionalPart = upperLegLateralKp * upperLegLateralAngleError;
      upperLegLateralIntegralPart = upperLegLateralKi * upperLegLateralIntegral;
      upperLegLateralDerivativePart = upperLegLateralKd * upperLegLateralDerivative;
      upperLegLateralOutput =
          upperLegLateralProportionalPart + upperLegLateralIntegralPart + upperLegLateralDerivativePart;

      if (!isFirstUpperLegLateralCycle) {
        // --- Apply control ---
        if (abs(upperLegLateralAngleError) > lateralTargetTolerance && upperLegLateralOutput > 0) {
          // move forward
          if (upperLegLateralOutput > valveOpenTimeClamp) {  // upper clamp
            upperLegLateralOutput = valveOpenTimeClamp;
          }

          // Increase angle (move forward)
          topFrontActuator->addPressureFluidlyWithOutflowValve(abs(upperLegLateralOutput));
          topBackActuator->releasePressureFluidlyWithInputValve(abs(upperLegLateralOutput) * 1.2);
        } else if (abs(lateralAngleError) > lateralTargetTolerance && lateralOutput < 0) {
          // move backward
          if (abs(upperLegLateralOutput) > valveOpenTimeClamp) {  // upper clamp
            upperLegLateralOutput = -valveOpenTimeClamp;
          }

          // Decrease angle (move backward)
          topFrontActuator->releasePressureFluidlyWithInputValve(abs(upperLegLateralOutput) * 1.2);
          topBackActuator->addPressureFluidlyWithOutflowValve(abs(upperLegLateralOutput));
        }
      }
    }

    // --- Debug info ---
    Serial.print("Target: ");
    Serial.print(lateralTargetAngle);
    Serial.print(" | Angle: ");
    Serial.print(currentLowerLegLateralAngle);
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
    Serial.print(currentLowerLegLongitudinalAngle);
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
    Serial.print(" | left bottom front muscle open(ms): ");
    Serial.print(leftFrontActuatorOutput);
    Serial.print(" | right bottom front muscle open(ms): ");
    Serial.print(rightFrontActuatorOutput);
    Serial.print(" | left bottom back muscle open(ms): ");
    Serial.print(leftBackActuatorOutput);
    Serial.print(" | right bottom back muscle open(ms): ");
    Serial.print(rightBackActuatorOutput);
    Serial.print(" | Upper leg lateral target: ");
    Serial.print(upperLegLateralTargetAngle);
    Serial.print(" | Upper leg lateral angle: ");
    Serial.print(currentUpperLegLateralAngle);
    Serial.print(" | Upper leg lateral error: ");
    Serial.print(upperLegLateralAngleError);
    Serial.print(" | Upper leg lateral output: ");
    Serial.print(upperLegLateralOutput);
    Serial.print(" | Upper leg lateral prop: ");
    Serial.print(upperLegLateralProportionalPart);
    Serial.print(" | Upper leg lateral der: ");
    Serial.print(upperLegLateralDerivativePart);
    Serial.print(" | Upper leg lateral int: ");
    Serial.print(upperLegLateralIntegralPart);

    // Prepare for next iteration
    previousLateralError = lateralAngleError;
    previousLateralTime = currentTime;
    previousLongitudinalError = longitudinalAngleError;
    previousLongitudinalTime = currentTime;
    previousUpperLegLateralError = upperLegLateralAngleError;
    previousUpperLegLateralTime = currentTime;

    delay(loopDelay);

    Serial.print(" | Loop time (ms): ");
    Serial.println(millis() - loopStartTime);

    lateralTargetAngle = getLowerLegLateralTargetAngle();
    longitudinalTargetAngle = getLowerLegLongitudinalTargetAngle();
    upperLegLateralTargetAngle = getUpperLegLateralTargetAngle();
    isFirstLateralCycle = false;
    isFirstLongitudinalCycle = false;
    isFirstUpperLegLateralCycle = false;
  }

  deleteTargets();
}

float ThreeDOFAntagonisticParticularMuscleControlAlgorithm::getLowerLegLateralTargetAngle() {
  for (size_t i = numberOfLowerLegTargets; i-- > 0;) {
    if (specificLowerLegControlTargets[i]->getActivationPointTime() <= millis()) {
      return specificLowerLegControlTargets[i]->getLateralTargetAngle();
    }
  }

  Serial.println("Error: No target angle found, setting target to 0deg");
  return 0;
}

float ThreeDOFAntagonisticParticularMuscleControlAlgorithm::getLowerLegLongitudinalTargetAngle() {
  for (size_t i = numberOfLowerLegTargets; i-- > 0;) {
    if (specificLowerLegControlTargets[i]->getActivationPointTime() <= millis()) {
      return specificLowerLegControlTargets[i]->getLongitudinalTargetAngle();
    }
  }

  Serial.println("Error: No target angle found, setting target to 0deg");
  return 0;
}

float ThreeDOFAntagonisticParticularMuscleControlAlgorithm::getUpperLegLateralTargetAngle() {
  for (size_t i = numberOfUpperLegTargets; i-- > 0;) {
    if (specificUpperLegControlTargets[i]->getActivationPointTime() <= millis()) {
      return specificUpperLegControlTargets[i]->getLateralTargetAngle();
    }
  }

  Serial.println("Error: No target angle found, setting target to 0deg");
  return 0;
}

void ThreeDOFAntagonisticParticularMuscleControlAlgorithm::setSpecificControlTargets(ControlTarget targets[],
                                                                                     size_t numberOfControlTargets,
                                                                                     float startTime,
                                                                                     unsigned long controlTime,
                                                                                     bool isLowerLegTargets) {
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
  if (isLowerLegTargets) {
    specificLowerLegControlTargets = new SpecificControlTarget*[numberOfControlTargets];
    numberOfLowerLegTargets = numberOfControlTargets;
  } else {
    specificUpperLegControlTargets = new SpecificControlTarget*[numberOfControlTargets];
    numberOfUpperLegTargets = numberOfControlTargets;
  }

  // Fill array with specific control targets
  for (size_t i = 0; i < numberOfControlTargets; ++i) {
    if (isLowerLegTargets) {
      specificLowerLegControlTargets[i] = targets[i].calculateSpecificControlTarget(startTime, controlTime);
    } else {
      specificUpperLegControlTargets[i] = targets[i].calculateSpecificControlTarget(startTime, controlTime);
    }
  }
}

void ThreeDOFAntagonisticParticularMuscleControlAlgorithm::deleteTargets() {
  if (specificUpperLegControlTargets != nullptr) {
    for (size_t i = 0; i < numberOfUpperLegTargets; ++i) {
      delete specificUpperLegControlTargets[i];
    }
    delete[] specificUpperLegControlTargets;
    specificUpperLegControlTargets = nullptr;
  }

  if (specificLowerLegControlTargets != nullptr) {
    for (size_t i = 0; i < numberOfLowerLegTargets; ++i) {
      delete specificLowerLegControlTargets[i];
    }
    delete[] specificLowerLegControlTargets;
    specificLowerLegControlTargets = nullptr;
  }
}

ThreeDOFAntagonisticParticularMuscleControlAlgorithm::~ThreeDOFAntagonisticParticularMuscleControlAlgorithm() {
  // specific control targets array cleanup
  deleteTargets();
}