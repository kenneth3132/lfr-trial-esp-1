/*
 * ======================================================================================
 * v1 Modular Line Follower: Sensor Acquisition & Position Math Module
 * ======================================================================================
 * Features:
 *   - Direct 8-channel ADC acquisition for SmartElex RLS-08
 *   - Dynamic thresholding against calibrated Reference_ADC[i]
 *   - Track polarity inversion support (inverseON: 0 = Black line, 1 = White line)
 *   - Packed 8-bit bitmask generation (bitSensor) for pattern matching
 *   - Weighted moment (sensorWight) & active count (sumOnSensor)
 *   - Built-in diagnostic visualizers for Serial Monitor
 * ======================================================================================
 */

#ifndef READ_SENSOR_H
#define READ_SENSOR_H

#include "config.h"

// ================= GLOBAL SENSOR STATE =================
extern int raw_ADC[NUM_SENSORS];
extern int Reference_ADC[NUM_SENSORS];
extern uint8_t sensorDigital[NUM_SENSORS];

extern int sumOnSensor;        // Total number of sensors actively seeing the line
extern long sensorWight;       // Weighted sum: sum(sensorDigital[i] * WeightValue[i])
extern uint8_t bitSensor;      // 8-bit packed bitmask representing sensor state
extern uint8_t inverseON;      // 0 = Black on White, 1 = White on Black

/*
 * read_black_line():
 * Samples all 8 analog channels (S0 to S7), binarizes the reading against
 * Reference_ADC[i], applies polarity inversion if active, accumulates the
 * active sensor count and weighted moment, and packs bitSensor.
 */
inline void read_black_line() {
  sumOnSensor = 0;
  sensorWight = 0;
  bitSensor = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    // Read raw 12-bit ADC value from ESP32 pin (0 to 4095)
    raw_ADC[i] = analogRead(sensorPins[i]);

    // Binarize reading according to track polarity
    if (inverseON == 0) {
      // Normal Mode: Black line has LOW reflectance -> HIGH ADC voltage
      sensorDigital[i] = (raw_ADC[i] > Reference_ADC[i]) ? 1 : 0;
    } else {
      // Inverted Mode: White line has HIGH reflectance -> LOW ADC voltage
      sensorDigital[i] = (raw_ADC[i] <= Reference_ADC[i]) ? 1 : 0;
    }

    // If this sensor detects the line
    if (sensorDigital[i] == 1) {
      sumOnSensor++;
      sensorWight += (long)WeightValue[i];
      // Pack into bitmask (bit 7 = S0 far-left, bit 0 = S7 far-right)
      bitSensor |= (1 << (7 - i));
    }
  }
}

// Diagnostic tool: Streams raw ADC readings (0 to 4095) for all 8 sensors
inline void sensor_value_test() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    raw_ADC[i] = analogRead(sensorPins[i]);
    Serial.print(F("S")); Serial.print(i + 1);
    Serial.print(F(": ")); Serial.print(raw_ADC[i]);
    Serial.print(F("\t"));
  }
  Serial.println();
}

// Diagnostic tool: Displays binary bitmask and calculated centroid position
inline void Bit_Sensor_Show() {
  read_black_line();
  
  Serial.print(F("bitSensor: 0b"));
  for (int i = 7; i >= 0; i--) {
    Serial.print((bitSensor >> i) & 1);
  }
  
  Serial.print(F("  |  Active: "));
  Serial.print(sumOnSensor);
  
  if (sumOnSensor > 0) {
    float pos = (float)sensorWight / sumOnSensor;
    float err = CENTER_SETPOINT - pos;
    Serial.print(F("  |  Pos: ")); Serial.print(pos, 1);
    Serial.print(F("  |  Err: ")); Serial.print(err, 1);
  } else {
    Serial.print(F("  |  Pos: LOST"));
  }
  
  Serial.print(F("  |  Mode: "));
  Serial.println(inverseON == 0 ? F("BLACK") : F("WHITE"));
}

// Diagnostic tool: Prints raw values alongside digital outputs for threshold tuning
inline void sensor_value_check() {
  read_black_line();
  Serial.println(F("--- SENSOR DIAGNOSTIC TABLE ---"));
  Serial.println(F("Channel | Raw ADC | Threshold | Digital State"));
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F("   S")); Serial.print(i + 1);
    Serial.print(F("    |   ")); Serial.print(raw_ADC[i]);
    Serial.print(F("  |   ")); Serial.print(Reference_ADC[i]);
    Serial.print(F("   |      ")); Serial.println(sensorDigital[i]);
  }
  Serial.println();
}

// Diagnostic helper: Checks whether outer edge sensors are active
inline void side_calibration() {
  read_black_line();
  if (sensorDigital[0] == 1 || sensorDigital[1] == 1) {
    Serial.println(F("[SIDE] LEFT outer edge sensor active!"));
  }
  if (sensorDigital[6] == 1 || sensorDigital[7] == 1) {
    Serial.println(F("[SIDE] RIGHT outer edge sensor active!"));
  }
}

#endif // READ_SENSOR_H
