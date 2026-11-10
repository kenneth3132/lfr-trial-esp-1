/*
 * ======================================================================================
 * v1 Modular Line Follower: Closed-Loop PID & Motion Intelligence Controller
 * ======================================================================================
 * Features:
 *   - Pre-Turn Directional Memory: Analyzes 8-bit bitmask (bitSensor) for wide
 *     footprints before the corner ends.
 *   - Weighted Centroid Tracking: Computes exact line offset against CENTER_SETPOINT (45.0).
 *   - Proportional-Derivative (PD) Steering: Smooth trajectory without wobble.
 *   - 90° Turn Branching: Shifts to axle-aligned pivot when line terminates.
 *   - 4-Way Cross Junction Balance: Symmetrical centroid drives smoothly straight through (+).
 *   - Gap Traversal (350ms): Bridges dashed track lines without losing heading.
 *   - Off-Track Runaway Auto-Brake: Halts motors if line missing > 350ms on straight section.
 * ======================================================================================
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "config.h"
#include "motor.h"
#include "read_sensor.h"
#include "turns.h"

// Controller state variables
extern String direction;
extern float previous_error;
extern unsigned long lostLineStartTime;

/*
 * PID_Controller():
 * The central intelligence loop of the robot.
 * Called in every iteration of loop() while isRunning is true.
 */
inline void PID_Controller(int base_speed, float Kp, float Kd) {
  // 1. Sample sensor array & compute moment
  read_black_line();

  // ====================================================================
  // BRANCH 1: LINE DETECTED (At least 1 sensor is active)
  // ====================================================================
  if (sumOnSensor > 0) {
    // Reset gap / runaway timer
    lostLineStartTime = 0;

    // Calculate line position: sum(sensorDigital[i] * WeightValue[i]) / sum(sensorDigital[i])
    float line_position = (float)sensorWight / sumOnSensor;

    // Calculate tracking error (Center setpoint is 45.0)
    // Left deviation = positive error, Right deviation = negative error
    float error = CENTER_SETPOINT - line_position;

    // ------------------------------------------------------------------
    // Pre-Turn Bitmask Pattern Matching
    // ------------------------------------------------------------------
    // Case A: 4-Way Cross Intersection (+) - All 8 sensors fire
    if (bitSensor == 0b11111111) {
      direction = "straight"; // Cruise straight across intersection
    }
    // Case B: Wide Left Footprint (90° corner or Left-T branch)
    // Examples: 0b11111000, 0b11110000, 0b11111100, 0b11100000, 0b11000000
    else if ((bitSensor & 0b11100000) == 0b11100000 || bitSensor == 0b11000000 || bitSensor == 0b11111100) {
      direction = "left";
    }
    // Case C: Wide Right Footprint (90° corner or Right-T branch)
    // Examples: 0b00011111, 0b00001111, 0b00111111, 0b00000111, 0b00000011
    else if ((bitSensor & 0b00000111) == 0b00000111 || bitSensor == 0b00000011 || bitSensor == 0b00111111) {
      direction = "right";
    }
    // Case D: Center / Mild Curve Footprints
    // Examples: 0b00011000, 0b00110000, 0b00001100, 0b01100000, etc.
    else {
      direction = "straight";
    }

    // ------------------------------------------------------------------
    // Closed-Loop PD Steering Calculation
    // ------------------------------------------------------------------
    float derivative = error - previous_error;
    float correction = (Kp * error) + (Kd * derivative);
    previous_error = error;

    // Differential steering:
    // When error is positive (robot is to right of line), steer left:
    // Left motor slows down, Right motor speeds up.
    int leftMotorSpeed  = base_speed - (int)correction;
    int rightMotorSpeed = base_speed + (int)correction;

    // Actuate motors
    motor(leftMotorSpeed, rightMotorSpeed);
  }

  // ====================================================================
  // BRANCH 2: ALL SENSORS OFF THE LINE (sumOnSensor == 0 / bitSensor == 0)
  // ====================================================================
  else {
    // Check stored directional memory from pre-turn pattern matching:
    
    // --- Turn Left Trigger ---
    if (direction == "left") {
      Serial.println(F("[TURN] Executing 90° LEFT Turn..."));
      turnLeft(DEFAULT_TURN_SPEED, DEFAULT_TURN_SPEED);
      previous_error = 0;
      lostLineStartTime = 0;
    }
    // --- Turn Right Trigger ---
    else if (direction == "right") {
      Serial.println(F("[TURN] Executing 90° RIGHT Turn..."));
      turnRight(DEFAULT_TURN_SPEED, DEFAULT_TURN_SPEED);
      previous_error = 0;
      lostLineStartTime = 0;
    }
    // --- Straight Segment: Gap Traversal & Runaway Brake ---
    else {
      // Start tracking gap loss time
      if (lostLineStartTime == 0) {
        lostLineStartTime = millis();
      }

      unsigned long elapsedLost = millis() - lostLineStartTime;

      if (elapsedLost < GAP_TIMEOUT) {
        // Coast straight forward across dashed line or small gap
        motor(base_speed, base_speed);
      } else {
        // Safe Auto-Brake: Off-track > 350ms -> Stop robot to prevent crash
        stop();
        Serial.println(F("[OFF-TRACK] Lost line > 350ms! Auto-brake engaged."));
      }
    }
  }
}

#endif // PID_CONTROLLER_H
