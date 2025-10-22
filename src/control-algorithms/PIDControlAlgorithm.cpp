#include "PIDControlAlgorithm.h"

PIDControlAlgorithm::PIDControlAlgorithm() {
  Serial.println("Creatin PID control algorithm class");
}

void PIDControlAlgorithm::controlMuscle(Muscle* muscle, Gyroscope* gyroscope, int controlTime) {
  // --- PID tuning parameters ---
  const float Kp = 0.9f;   // Proportional gain
  const float Ki = 0.08f;  // Integral gain
  const float Kd = 1.4f;   // Derivative gain

  const float targetTolerance = 5;
  const float valveOpenTimeClamp = 300;

  float targetXAngle = 70;

  // --- Control setup ---
  const int loopDelay = 50;  // PID update every 50ms
  unsigned long previousTime = millis();

  float integral = 0.0f;
  float previousError = 0.0f;

  bool isFirstCycle = true;

  Serial.println("Starting PID control for 20s...");

  unsigned long startTime = millis();
  float output = 0;
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
    for (int i = 0; i < 3; i++) {
      gyroscope->updateValues();
    }
    float currentAngle = gyroscope->getXAngle();

    // --- PID calculations ---
    float error = targetXAngle - currentAngle;
    if (abs(error) > targetTolerance) {  // do not calculate when in target tolerance
      integral += error * deltaTime;
      int clamp = 100;
      if (integral > clamp) {
        integral = clamp;
      } else if (integral < -clamp) {
        integral = -clamp;
      }

      float derivative = (error - previousError) / (deltaTime + 20);
      if (derivative > clamp) {
        derivative = clamp;
      } else if (derivative < -clamp) {
        derivative = -clamp;
      }

      output = Kp * error + Ki * integral + Kd * derivative;

      if (!isFirstCycle) {
        // --- Apply control ---
        if (abs(error) > targetTolerance && output > targetTolerance) {
          if (output > valveOpenTimeClamp) {  // upper clamp
            output = valveOpenTimeClamp;
          }

          muscle->addPressure(abs(output));  // Increase angle
        } else if (abs(error) > targetTolerance && output < -targetTolerance) {
          if (abs(output) > valveOpenTimeClamp) {  // upper clamp
            output = -valveOpenTimeClamp;
          }

          muscle->releasePressure(abs(output));  // Decrease angle
        } else {
          // Small correction area — hold position
          muscle->closeInput();
          muscle->closeOutput();
        }
      }
    }

    // --- Debug info ---
    Serial.print("Target: ");  // PID not printing when at target
    Serial.print(targetXAngle);
    Serial.print(" | Angle: ");
    Serial.print(currentAngle);
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | Output: ");
    Serial.print(output);
    Serial.print(" | Time (ms): ");
    Serial.print(now);

    // Prepare for next iteration
    previousError = error;
    previousTime = now;

    delay(loopDelay);

    if (!isFirstCycle) {
      Serial.print(" | Loop time (ms): ");
      Serial.println(millis() - loopStartTime);
    }

    isFirstCycle = false;
  }
}