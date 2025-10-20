#include "PIDControlAlgorithm.h"

PIDControlAlgorithm::PIDControlAlgorithm() {
  Serial.println("Creatin PID control algorithm class");
}

void PIDControlAlgorithm::controlMuscle(Muscle* muscle, Gyroscope* gyroscope, int controlTime) {
  // --- PID tuning parameters ---
  const float Kp = 1.2f;   // Proportional gain
  const float Ki = 0.05f;  // Integral gain
  const float Kd = 0.3f;   // Derivative gain

  const float targetTolerance = 5;
  const float valveOpenTimeClamp = 300;

  float targetXAngle = 45;

  // --- Control setup ---
  const int loopDelay = 50;  // PID update every 50ms
  unsigned long previousTime = millis();

  float integral = 0.0f;
  float previousError = 0.0f;

  bool isFirstCycle = true;

  Serial.println("Starting PID control for 10s...");

  unsigned long startTime = millis();
  while (millis() - startTime < (unsigned long)controlTime) {
    unsigned long now = millis();
    float deltaTime = (now - previousTime) / 1000.0f;  // seconds

    // --- Read current angle ---
    gyroscope->updateValues();
    float currentAngle = gyroscope->getXAngle();

    // --- PID calculations ---
    float error = targetXAngle - currentAngle;
    integral += error * deltaTime;
    float derivative = (error - previousError) / deltaTime;

    float output = Kp * error + Ki * integral + Kd * derivative;

    if (!isFirstCycle) {
      // --- Apply control ---
      if (output > targetTolerance && output < valveOpenTimeClamp) {
        muscle->addPressure(abs(output));  // Increase angle
      } else if (output < -targetTolerance && output < valveOpenTimeClamp) {
        muscle->releasePressure(abs(output));  // Decrease angle
      } else {
        // Small correction area — hold position
        muscle->closeInput();
        muscle->closeOutput();
      }

      // --- Debug info ---
      Serial.print("Target: ");
      Serial.print(targetXAngle);
      Serial.print(" | Angle: ");
      Serial.print(currentAngle);
      Serial.print(" | Output: ");
      Serial.println(output);
    }

    // Prepare for next iteration
    previousError = error;
    previousTime = now;
    isFirstCycle = false;

    delay(loopDelay);
  }
}