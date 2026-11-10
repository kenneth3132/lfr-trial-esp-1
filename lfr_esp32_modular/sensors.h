#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"

extern int rawValues[NUM_SENSORS];
extern int minValues[NUM_SENSORS];
extern int maxValues[NUM_SENSORS];
extern int thresholds[NUM_SENSORS];
extern int normValues[NUM_SENSORS];
extern bool isCalibrated;
extern int trackMode;

inline void initSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i]  = 400;
    maxValues[i]  = 3500;
    thresholds[i] = 1950;
  }
}

inline void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    rawValues[i] = analogRead(sensorPins[i]);
    int norm = map(rawValues[i], minValues[i], maxValues[i], 0, 1000);
    normValues[i] = constrain(norm, 0, 1000);
  }
}

inline int getPosition() {
  long numerator = 0;
  long denominator = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = (trackMode == 0) ? normValues[i] : (1000 - normValues[i]);
    if (val > 200) {
      numerator += (long)val * weights[i];
      denominator += val;
    }
  }

  if (denominator == 0) {
    return 9999; // Lost line
  }

  return (int)(numerator / denominator);
}

inline void calibrateSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i] = 4095;
    maxValues[i] = 0;
  }

  unsigned long start = millis();
  while (millis() - start < 4000) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < minValues[i]) minValues[i] = val;
      if (val > maxValues[i]) maxValues[i] = val;
    }
    digitalWrite(LED_PIN, (millis() / 80) % 2);
    delay(5);
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    thresholds[i] = (minValues[i] + maxValues[i]) / 2;
  }

  isCalibrated = true;
  digitalWrite(LED_PIN, LOW);
}

#endif // SENSORS_H
