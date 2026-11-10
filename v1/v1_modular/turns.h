/*
 * ======================================================================================
 * v1 Modular Line Follower: Sharp Turn & In-Place Pivot Module
 * ======================================================================================
 * Features:
 *   - Axle-Alignment Creep: Drives straight forward 80ms (DELAY_BEFORE_TURN) to align
 *     the wheel axis directly over the 90° corner vertex before spinning.
 *   - In-Place Differential Pivot: Rotates with one wheel forward and one reverse.
 *   - Center Sensor Line Re-Lock: Stops spinning the exact millisecond center sensors
 *     S3 or S4 detect the new line path.
 *   - Active Counter-Braking: Calls motor(0, 0) immediately upon re-lock to cancel
 *     rotational inertia and prevent overshoot.
 *   - 1500ms Watchdog Safety: Aborts spin if robot is lifted or runs off the track.
 * ======================================================================================
 */

#ifndef TURNS_H
#define TURNS_H

#include "config.h"
#include "motor.h"
#include "read_sensor.h"

// Direction state variable declaration (shared with pid_controller.h)
extern String direction;

/*
 * turnRight(int L, int R):
 * Executes an axle-aligned 90° right pivot.
 * Parameters:
 *   L: Left motor forward speed during spin (e.g. 150)
 *   R: Right motor reverse speed during spin (e.g. 150)
 */
inline void turnRight(int L, int R) {
  // 1. Axle Alignment Creep: Move straight forward to place drive axle over vertex
  motor(DEFAULT_CREEP_SPEED, DEFAULT_CREEP_SPEED);
  delay(DELAY_BEFORE_TURN);

  // 2. Begin in-place differential right pivot (Left forward, Right reverse)
  motor(L, -R);

  unsigned long startTime = millis();

  // 3. Spin until center sensor S3 or S4 recaptures the line
  while (true) {
    read_black_line();

    // Line Re-Lock condition: Center sensors S3 or S4 detect the line
    if (sensorDigital[3] == 1 || sensorDigital[4] == 1) {
      break;
    }

    // Safety Watchdog: Abort if spinning longer than 1500ms (e.g. lifted off track)
    if (millis() - startTime > WATCHDOG_TIMEOUT) {
      Serial.println(F("[WATCHDOG] Right turn timeout exceeded (>1500ms)! Aborting spin."));
      break;
    }

    delay(2);
  }

  // 4. Active Electronic Braking to eliminate rotational overshoot
  stop();
  delay(30);

  // 5. Reset directional memory state back to straight
  direction = "straight";
}

/*
 * turnLeft(int L, int R):
 * Executes an axle-aligned 90° left pivot.
 * Parameters:
 *   L: Left motor reverse speed during spin (e.g. 150)
 *   R: Right motor forward speed during spin (e.g. 150)
 */
inline void turnLeft(int L, int R) {
  // 1. Axle Alignment Creep: Move straight forward to place drive axle over vertex
  motor(DEFAULT_CREEP_SPEED, DEFAULT_CREEP_SPEED);
  delay(DELAY_BEFORE_TURN);

  // 2. Begin in-place differential left pivot (Left reverse, Right forward)
  motor(-L, R);

  unsigned long startTime = millis();

  // 3. Spin until center sensor S3 or S4 recaptures the line
  while (true) {
    read_black_line();

    // Line Re-Lock condition: Center sensors S3 or S4 detect the line
    if (sensorDigital[3] == 1 || sensorDigital[4] == 1) {
      break;
    }

    // Safety Watchdog: Abort if spinning longer than 1500ms
    if (millis() - startTime > WATCHDOG_TIMEOUT) {
      Serial.println(F("[WATCHDOG] Left turn timeout exceeded (>1500ms)! Aborting spin."));
      break;
    }

    delay(2);
  }

  // 4. Active Electronic Braking
  stop();
  delay(30);

  // 5. Reset directional memory state back to straight
  direction = "straight";
}

/*
 * distance(int cm):
 * Drives straight forward for a calibrated duration based on distance.
 * Useful for bridging long gaps or cross segments.
 */
inline void distance(int cm) {
  // Standard calibration estimate: ~25ms per cm at base_speed = 160
  const unsigned long ms_per_cm = 25;
  unsigned long runTime = (unsigned long)cm * ms_per_cm;

  motor(DEFAULT_BASE_SPEED, DEFAULT_BASE_SPEED);
  delay(runTime);
  stop();
}

#endif // TURNS_H
