/*
 * ======================================================================================
 * MODULAR ESP32 PID LINE FOLLOWER ROBOT
 * ======================================================================================
 * Modules:
 *   - config.h  : Pin mappings, PWM settings, sensor weights
 *   - sensors.h : ADC reading, auto-calibration, weighted position
 *   - motors.h  : TB6612FNG direction & universal PWM control
 *   - pid.h     : PID calculation, differential steering, line-loss recovery
 * ======================================================================================
 */

#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "pid.h"

// Global Variables
int rawValues[NUM_SENSORS];
int minValues[NUM_SENSORS];
int maxValues[NUM_SENSORS];
int thresholds[NUM_SENSORS];
int normValues[NUM_SENSORS];
bool isCalibrated = false;
int trackMode     = 0; // 0 = Black line, 1 = White line

float Kp = 0.045;
float Ki = 0.0000;
float Kd = 0.280;
int baseSpeed   = 160;
int maxSpeed    = 240;
int minSpeed    = -100;
int searchSpeed = 140;

int lastError   = 0;
float integral  = 0;
bool isRunning  = false;

unsigned long lastBtnCheck = 0;
const unsigned long debounceDelay = 250;

void flashLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    if (i < times - 1) delay(delayMs);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("\n--- ESP32 Modular PID Line Follower ---"));

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_CALIB, INPUT_PULLUP);
  pinMode(BTN_MODE,  INPUT_PULLUP);
  pinMode(LED_PIN,   OUTPUT);
  digitalWrite(LED_PIN, LOW);

  initMotors();
  initSensors();
  stopMotors();

  flashLED(2, 100);
  Serial.println(F("[READY] System initialized."));
}

void loop() {
  unsigned long now = millis();

  // Button 1: Start / Stop
  if (digitalRead(BTN_START) == LOW && (now - lastBtnCheck > debounceDelay)) {
    lastBtnCheck = now;
    isRunning = !isRunning;
    if (isRunning) {
      digitalWrite(LED_PIN, HIGH);
      lastError = 0;
      integral = 0;
      delay(400);
    } else {
      stopMotors();
      digitalWrite(LED_PIN, LOW);
    }
  }

  // Button 2: Calibrate
  if (!isRunning && digitalRead(BTN_CALIB) == LOW && (now - lastBtnCheck > debounceDelay)) {
    lastBtnCheck = now;
    Serial.println(F("[CALIBRATING] Sweep robot across line for 4s..."));
    calibrateSensors();
    Serial.println(F("[CALIBRATED] Done."));
    flashLED(3, 100);
  }

  // Button 3: Track Mode
  if (!isRunning && digitalRead(BTN_MODE) == LOW && (now - lastBtnCheck > debounceDelay)) {
    lastBtnCheck = now;
    trackMode = (trackMode == 0) ? 1 : 0;
    if (trackMode == 0) {
      Serial.println(F("[MODE] Black line on white"));
      flashLED(1, 300);
    } else {
      Serial.println(F("[MODE] White line on black"));
      flashLED(2, 150);
    }
  }

  // PID Execution
  if (isRunning) {
    runPID();
  }
}
