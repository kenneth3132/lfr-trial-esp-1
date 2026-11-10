/*
 * ======================================================================================
 * v1 Modular ESP32 High-Speed PID Line Follower Robot
 * ======================================================================================
 * Author: Kenneth (@kenneth3132)
 * Hardware:
 *   - ESP32-D0WD-V3 Dev Module
 *   - SmartElex RLS-08 (8 IR Sensors via Direct ADC)
 *   - TB6612FNG Dual H-Bridge Driver
 *   - 3 Push Buttons + Status LED (GPIO 2)
 * 
 * Features:
 *   - Pre-turn bitmask pattern memory (90° turns, hairpins, T-junctions)
 *   - Axle-alignment creep (80ms shift over corner vertex before spin)
 *   - Controlled in-place differential spin with center sensor (S3/S4) re-lock
 *   - Active electric counter-braking to eliminate rotational overshoot
 *   - 1500ms safety watchdog timeout
 *   - Closed-loop Proportional-Derivative (PD) tracking
 *   - 4-way cross junction (+) symmetrical balancing
 *   - 350ms gap traversal & runaway off-track auto-brake
 *   - Dynamic 4-sweep auto-calibration with EEPROM persistence
 * ======================================================================================
 */

#include "config.h"
#include "read_sensor.h"
#include "motor.h"
#include "turns.h"
#include "calibration.h"
#include "pid_controller.h"

// ================= GLOBAL VARIABLE DEFINITIONS =================
int raw_ADC[NUM_SENSORS];
int Reference_ADC[NUM_SENSORS];
uint8_t sensorDigital[NUM_SENSORS];

int sumOnSensor = 0;
long sensorWight = 0;
uint8_t bitSensor = 0;
uint8_t inverseON = 0; // 0 = Black on White, 1 = White on Black

int Min_ADC[NUM_SENSORS];
int Max_ADC[NUM_SENSORS];

String direction = "straight";
float previous_error = 0;
unsigned long lostLineStartTime = 0;

bool isRunning = false;
unsigned long lastBtnCheck = 0;
const unsigned long debounceDelay = 250;

// Helper: Blink status LED
void flashLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    if (i < times - 1) delay(delayMs);
  }
}

// ================= ARDUINO SETUP =================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("\n========================================================"));
  Serial.println(F("   v1 ESP32 Advanced PID Line Follower Initializing     "));
  Serial.println(F("========================================================"));
  Serial.println(F("Btn 1 (GPIO 27) : START / STOP PID run"));
  Serial.println(F("Btn 2 (GPIO 14) : CALIBRATE on Track"));
  Serial.println(F("Btn 3 (GPIO 13) : TOGGLE Black / White Track Mode"));
  Serial.println(F("========================================================\n"));

  // 1. Configure Push Buttons
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_CALIB, INPUT_PULLUP);
  pinMode(BTN_MODE,  INPUT_PULLUP);

  // 2. Configure Status LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // 3. Initialize TB6612FNG Motors and Universal PWM
  initMotors();
  stop();

  // 4. Load Calibrated Thresholds from EEPROM Flash
  LoadCalibration();

  // 5. Ready Indication (3-flash countdown)
  flashLED(3, 100);
  Serial.println(F("[SYSTEM READY] Press Button 1 to run, or Button 2 to calibrate."));
}

// ================= MAIN LOOP =================
void loop() {
  unsigned long now = millis();

  // ----------------------------------------------------
  // BUTTON 1: START / STOP TOGGLE
  // ----------------------------------------------------
  if (digitalRead(BTN_START) == LOW && (now - lastBtnCheck > debounceDelay)) {
    lastBtnCheck = now;
    isRunning = !isRunning;
    if (isRunning) {
      Serial.println(F("[STATE] >>> PID RUNNING <<<"));
      digitalWrite(LED_PIN, HIGH);
      previous_error = 0;
      lostLineStartTime = 0;
      direction = "straight";
      delay(300); // Brief launch delay
    } else {
      Serial.println(F("[STATE] >>> ROBOT STOPPED <<<"));
      stop();
      digitalWrite(LED_PIN, LOW);
    }
  }

  // ----------------------------------------------------
  // BUTTON 2: TRIGGER SENSOR CALIBRATION (While Stopped)
  // ----------------------------------------------------
  if (!isRunning && digitalRead(BTN_CALIB) == LOW && (now - lastBtnCheck > debounceDelay)) {
    lastBtnCheck = now;
    calibrateSensor();
  }

  // ----------------------------------------------------
  // BUTTON 3: TOGGLE TRACK POLARITY (While Stopped)
  // ----------------------------------------------------
  if (!isRunning && digitalRead(BTN_MODE) == LOW && (now - lastBtnCheck > debounceDelay)) {
    lastBtnCheck = now;
    inverseON = (inverseON == 0) ? 1 : 0;
    Serial.print(F("[MODE CHANGED] "));
    if (inverseON == 0) {
      Serial.println(F("NORMAL: Black line on white surface"));
      flashLED(1, 300);
    } else {
      Serial.println(F("INVERTED: White line on black surface"));
      flashLED(2, 150);
    }
  }

  // ----------------------------------------------------
  // EXECUTE CLOSED-LOOP PID NAVIGATION
  // ----------------------------------------------------
  if (isRunning) {
    PID_Controller(DEFAULT_BASE_SPEED, DEFAULT_KP, DEFAULT_KD);
  } else {
    // Idle Telemetry: Stream diagnostic bitmask every 500ms when stopped
    static unsigned long lastTelemetry = 0;
    if (now - lastTelemetry >= 500) {
      lastTelemetry = now;
      Bit_Sensor_Show();
    }
  }
}
