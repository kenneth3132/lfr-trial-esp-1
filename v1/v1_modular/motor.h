/*
 * ======================================================================================
 * v1 Modular Line Follower: Motor Control & Actuation Module
 * ======================================================================================
 * Features:
 *   - Dual-channel bidirectional control for TB6612FNG H-bridge
 *   - Universal ESP32 PWM frequency & duty cycle generation
 *   - Active electronic braking on motor(0, 0)
 *   - Bench diagnostic testing (motor_test) and emergency halt (hard_stop)
 * ======================================================================================
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "config.h"

// Forward declaration
void flashLED(int times, int delayMs);

// Initialize motor direction pins, standby, and hardware PWM
inline void initMotors() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  // Enable the TB6612FNG driver
  digitalWrite(PIN_STBY, HIGH);

  // Configure PWM for both channels
  PWM_SETUP(PIN_PWMA, pwmA_ch, pwmFreq, pwmRes);
  PWM_SETUP(PIN_PWMB, pwmB_ch, pwmFreq, pwmRes);
}

/*
 * motor(int LPWM, int RPWM):
 * Actuates left and right motors with independent speed and direction.
 * Arguments:
 *   LPWM: -255 (full reverse) to +255 (full forward). 0 = active electric brake.
 *   RPWM: -255 (full reverse) to +255 (full forward). 0 = active electric brake.
 */
inline void motor(int LPWM, int RPWM) {
  // Clamp input values to valid range [-255, 255]
  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);

  // --- Left Motor Actuation ---
  if (LPWM > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    PWM_WRITE(PIN_PWMA, pwmA_ch, LPWM);
  } else if (LPWM < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    PWM_WRITE(PIN_PWMA, pwmA_ch, -LPWM);
  } else {
    // Active electronic brake (both low with 0 duty)
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    PWM_WRITE(PIN_PWMA, pwmA_ch, 0);
  }

  // --- Right Motor Actuation ---
  if (RPWM > 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    PWM_WRITE(PIN_PWMB, pwmB_ch, RPWM);
  } else if (RPWM < 0) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    PWM_WRITE(PIN_PWMB, pwmB_ch, -RPWM);
  } else {
    // Active electronic brake
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    PWM_WRITE(PIN_PWMB, pwmB_ch, 0);
  }
}

// Immediate electronic brake to eliminate forward/rotational momentum
inline void stop() {
  motor(0, 0);
}

// Emergency halt: enters an infinite halt loop while blinking LED
inline void hard_stop() {
  stop();
  Serial.println(F("[SAFETY] HARD STOP ACTIVATED. Reboot to reset."));
  while (true) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

// Bench diagnostic routine to verify H-bridge motor wiring
inline void motor_test() {
  Serial.println(F("[TEST] Starting motor diagnostic sequence..."));
  
  // Forward 1.2s
  Serial.println(F("  -> Forward (120, 120)"));
  motor(120, 120);
  delay(1200);

  // Backward 1.2s
  Serial.println(F("  -> Reverse (-120, -120)"));
  motor(-120, -120);
  delay(1200);

  // Right pivot 1.0s
  Serial.println(F("  -> Right Pivot (140, -140)"));
  motor(140, -140);
  delay(1000);

  // Left pivot 1.0s
  Serial.println(F("  -> Left Pivot (-140, 140)"));
  motor(-140, 140);
  delay(1000);

  // Stop
  stop();
  Serial.println(F("[TEST] Motor diagnostic complete."));
}

#endif // MOTOR_H
