/*
 * ======================================================================================
 * v1 Modular Line Follower: Sensor Auto-Calibration & EEPROM Persistence Module
 * ======================================================================================
 * Features:
 *   - 4-Sweep Rotational Scan: Oscillates Left -> Right -> Right -> Left at 40% speed
 *     to expose all 8 sensors to both track background and line.
 *   - Dynamic Threshold Calculation: Reference_ADC[i] = (Max_ADC[i] + Min_ADC[i]) / 2.
 *   - Individual Channel Validation: Verifies contrast (Max - Min >= 150).
 *   - Non-Volatile Flash Persistence via ESP32 EEPROM.
 *   - Graceful Fallback: Defaults invalid/disconnected channels to 2000.
 * ======================================================================================
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "config.h"
#include "motor.h"
#include "read_sensor.h"

// Calibration storage arrays
extern int Min_ADC[NUM_SENSORS];
extern int Max_ADC[NUM_SENSORS];

// EEPROM memory layout (ESP32):
// Offset 0: Magic byte (0x5A = Valid calibration present)
// Offset 1..16: Reference_ADC (8 * 2 bytes = 16 bytes)
// Offset 17..32: Min_ADC (16 bytes)
// Offset 33..48: Max_ADC (16 bytes)
const int EEPROM_SIZE = 64;
const uint8_t EEPROM_MAGIC = 0x5A;

/*
 * calibrateSensor():
 * Performs an automated 4-stage sweep or manual surface sample to record
 * peak minimums and maximums across all 8 sensors.
 */
inline void calibrateSensor() {
  Serial.println(F("\n========================================================"));
  Serial.println(F(">>> STARTING SENSOR AUTO-CALIBRATION <<<"));
  Serial.println(F("Place robot on track. Performing 4-sweep rotational scan..."));
  Serial.println(F("========================================================"));

  stop();

  // Reset calibration boundaries
  for (int i = 0; i < NUM_SENSORS; i++) {
    Min_ADC[i] = 4095;
    Max_ADC[i] = 0;
  }

  const int calibSpeed = 110; // ~40% motor speed for gentle sweeping

  // --- STAGE 1: Sweep Left (800ms) ---
  motor(-calibSpeed, calibSpeed);
  unsigned long start = millis();
  while (millis() - start < 800) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < Min_ADC[i]) Min_ADC[i] = val;
      if (val > Max_ADC[i]) Max_ADC[i] = val;
    }
    digitalWrite(LED_PIN, (millis() / 80) % 2);
    delay(4);
  }

  // --- STAGE 2: Sweep Right Across Center (1600ms) ---
  motor(calibSpeed, -calibSpeed);
  start = millis();
  while (millis() - start < 1600) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < Min_ADC[i]) Min_ADC[i] = val;
      if (val > Max_ADC[i]) Max_ADC[i] = val;
    }
    digitalWrite(LED_PIN, (millis() / 80) % 2);
    delay(4);
  }

  // --- STAGE 3: Sweep Back to Center (800ms) ---
  motor(-calibSpeed, calibSpeed);
  start = millis();
  while (millis() - start < 800) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < Min_ADC[i]) Min_ADC[i] = val;
      if (val > Max_ADC[i]) Max_ADC[i] = val;
    }
    digitalWrite(LED_PIN, (millis() / 80) % 2);
    delay(4);
  }

  stop();

  // --- Compute Thresholds & Validate Channels ---
  for (int i = 0; i < NUM_SENSORS; i++) {
    int contrast = Max_ADC[i] - Min_ADC[i];
    if (contrast >= 150) {
      // Valid channel: Midpoint threshold
      Reference_ADC[i] = (Min_ADC[i] + Max_ADC[i]) / 2;
    } else {
      // Weak contrast warning: Sensor didn't see both surfaces
      Serial.print(F("[WARN] Low contrast on S")); Serial.print(i + 1);
      Serial.print(F(" (Diff: ")); Serial.print(contrast);
      Serial.println(F("). Setting default threshold = 2000."));
      Reference_ADC[i] = 2000;
    }
  }

  // --- Save to Non-Volatile EEPROM ---
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(0, EEPROM_MAGIC);
  for (int i = 0; i < NUM_SENSORS; i++) {
    EEPROM.put(1 + (i * 2), Reference_ADC[i]);
    EEPROM.put(17 + (i * 2), Min_ADC[i]);
    EEPROM.put(33 + (i * 2), Max_ADC[i]);
  }
  EEPROM.commit();

  digitalWrite(LED_PIN, LOW);
  Serial.println(F("\n>>> CALIBRATION COMPLETE & SAVED TO EEPROM! <<<"));
  Serial.println(F("Channel | Min (White) | Max (Black) | Threshold (Midpoint)"));
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F("   S")); Serial.print(i + 1);
    Serial.print(F("    |    ")); Serial.print(Min_ADC[i]);
    Serial.print(F("     |    ")); Serial.print(Max_ADC[i]);
    Serial.print(F("     |    ")); Serial.println(Reference_ADC[i]);
  }
  Serial.println(F("========================================================\n"));
}

/*
 * LoadCalibration():
 * Loads stored thresholds from EEPROM on startup.
 * Defaults to 2000 if no valid calibration has been performed yet.
 */
inline void LoadCalibration() {
  EEPROM.begin(EEPROM_SIZE);
  uint8_t magic = EEPROM.read(0);

  if (magic == EEPROM_MAGIC) {
    Serial.println(F("[EEPROM] Valid calibration found. Loading thresholds..."));
    for (int i = 0; i < NUM_SENSORS; i++) {
      EEPROM.get(1 + (i * 2), Reference_ADC[i]);
      EEPROM.get(17 + (i * 2), Min_ADC[i]);
      EEPROM.get(33 + (i * 2), Max_ADC[i]);
    }
  } else {
    Serial.println(F("[EEPROM] No saved calibration found. Loading safe defaults (2000)..."));
    for (int i = 0; i < NUM_SENSORS; i++) {
      Min_ADC[i] = 400;
      Max_ADC[i] = 3600;
      Reference_ADC[i] = 2000;
    }
  }
}

#endif // CALIBRATION_H
