/*
 * ======================================================================================
 * HIGH-PERFORMANCE ESP32 PID LINE FOLLOWER ROBOT
 * ======================================================================================
 * Hardware Components:
 *   - Microcontroller : ESP32-D0WD-V3 Dev Module (on COM9)
 *   - Line Sensor     : SmartElex RLS-08 (8 IR Reflectance Sensors, Direct to ESP32 ADC)
 *   - Motor Driver    : TB6612FNG Dual H-Bridge Driver
 *   - UI Controls     : 3 Push Buttons + Onboard Blue LED (GPIO 2)
 * 
 * Button Controls:
 *   - Button 1 (GPIO 27) : START / STOP Robot
 *   - Button 2 (GPIO 14) : AUTO-CALIBRATE on track surface (sweep 4 seconds across line)
 *   - Button 3 (GPIO 13) : TOGGLE Track Mode (Black line on white / White line on black)
 * 
 * Pinout Connections:
 *   - SmartElex RLS-08:
 *       S1 -> GPIO 36 (VP)  | S2 -> GPIO 39 (VN)  | S3 -> GPIO 34  | S4 -> GPIO 35
 *       S5 -> GPIO 32       | S6 -> GPIO 33       | S7 -> GPIO 25  | S8 -> GPIO 26
 *   - TB6612FNG Motor Driver:
 *       PWMA -> GPIO 17     | AIN1 -> GPIO 4      | AIN2 -> GPIO 16 (Left Motor)
 *       PWMB -> GPIO 15     | BIN1 -> GPIO 5      | BIN2 -> GPIO 18 (Right Motor)
 *       STBY -> GPIO 19     (Must be HIGH to enable motor output)
 *   - Buttons (Internal PULL-UP, connect between GPIO and GND):
 *       Button 1 (Start/Stop) : GPIO 27
 *       Button 2 (Calibrate)  : GPIO 14
 *       Button 3 (Track Mode) : GPIO 13
 *   - Status LED: GPIO 2
 * ======================================================================================
 */

#include <Arduino.h>

// Universal ESP32 Core 2.x & 3.x Compatibility for LEDC PWM
#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // ESP32 Arduino Core 3.x
  #define PWM_SETUP(pin, ch, freq, res) ledcAttach(pin, freq, res)
  #define PWM_WRITE(pin, ch, val)       ledcWrite(pin, val)
#else
  // ESP32 Arduino Core 2.x
  #define PWM_SETUP(pin, ch, freq, res) do { ledcSetup(ch, freq, res); ledcAttachPin(pin, ch); } while(0)
  #define PWM_WRITE(pin, ch, val)       ledcWrite(ch, val)
#endif

// ================= HARDWARE PIN DEFINITIONS =================
// 8-Channel Sensor Pins (SmartElex RLS-08)
const int NUM_SENSORS = 8;
const int sensorPins[NUM_SENSORS] = {36, 39, 34, 35, 32, 33, 25, 26};

// TB6612FNG Motor Driver Pins
#define PIN_PWMA  17   // Left Motor Speed
#define PIN_AIN1  4    // Left Motor Direction 1
#define PIN_AIN2  16   // Left Motor Direction 2
#define PIN_PWMB  15   // Right Motor Speed
#define PIN_BIN1  5    // Right Motor Direction 1
#define PIN_BIN2  18   // Right Motor Direction 2
#define PIN_STBY  19   // Standby (Enable Driver)

// Push Buttons (Internal Pull-Up)
#define BTN_START 27   // Button 1: Start / Stop
#define BTN_CALIB 14   // Button 2: Auto-Calibrate
#define BTN_MODE  13   // Button 3: Black/White Mode

// Status LED
#define LED_PIN   2

// ================= PWM SETTINGS =================
const int pwmFreq = 10000; // 10 kHz for smooth & quiet motor drive
const int pwmRes  = 8;     // 8-bit resolution (0 to 255)
const int pwmA_ch = 0;     // PWM Channel 0 (for Core 2.x)
const int pwmB_ch = 1;     // PWM Channel 1 (for Core 2.x)

// ================= PID & SPEED TUNING PARAMETERS =================
// Adjust these parameters to match your motors, chassis, and speed requirements
float Kp = 0.045;          // Proportional gain (immediate steering response)
float Ki = 0.0000;         // Integral gain (usually 0 for high-speed line followers)
float Kd = 0.280;          // Derivative gain (anticipates curves, damps oscillation)

int baseSpeed  = 160;      // Base cruising speed (0 to 255)
int maxSpeed   = 240;      // Maximum motor speed limit
int minSpeed   = -100;     // Minimum motor speed (negative allows reverse braking for sharp turns)
int searchSpeed = 140;     // Recovery spin speed when line is lost

// Sensor weights from far-left to far-right (Center between S4 and S5 is 0)
const int weights[NUM_SENSORS] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

// ================= ROBOT STATE VARIABLES =================
int rawValues[NUM_SENSORS];
int minValues[NUM_SENSORS];
int maxValues[NUM_SENSORS];
int thresholds[NUM_SENSORS];
int normValues[NUM_SENSORS];

bool isRunning      = false;
bool isCalibrated   = false;
int trackMode       = 0;     // 0 = Black line on white surface; 1 = White line on black surface

int lastError       = 0;
float integral      = 0;
unsigned long lostTime = 0;

unsigned long lastBtnCheck = 0;
const unsigned long debounceDelay = 250;

// ================= FORWARD DECLARATIONS =================
void readSensors();
int getPosition();
void runPID();
void calibrateSensors();
void setMotors(int leftSpeed, int rightSpeed);
void stopMotors();
void flashLED(int times, int delayMs);

// ================= INITIAL SETUP =================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("\n========================================================"));
  Serial.println(F("   ESP32 PID Fast Line Follower (SmartElex RLS-08)      "));
  Serial.println(F("========================================================"));
  Serial.println(F("Btn 1 (GPIO 27) : START / STOP"));
  Serial.println(F("Btn 2 (GPIO 14) : CALIBRATE on Track"));
  Serial.println(F("Btn 3 (GPIO 13) : TOGGLE Black/White Mode"));
  Serial.println(F("========================================================\n"));

  // 1. Setup Button Pins
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_CALIB, INPUT_PULLUP);
  pinMode(BTN_MODE,  INPUT_PULLUP);

  // 2. Setup LED Pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // 3. Setup Motor Direction & Standby Pins
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH); // Enable motor driver

  // 4. Setup PWM using universal macro
  PWM_SETUP(PIN_PWMA, pwmA_ch, pwmFreq, pwmRes);
  PWM_SETUP(PIN_PWMB, pwmB_ch, pwmFreq, pwmRes);

  stopMotors();

  // 5. Default Calibration Values (in case user starts without calibrating)
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i]  = 400;
    maxValues[i]  = 3500;
    thresholds[i] = 1950;
  }

  flashLED(2, 100);
  Serial.println(F("[READY] Place robot on track, press Button 2 to calibrate, then Button 1 to run."));
}

// ================= MAIN LOOP =================
void loop() {
  unsigned long now = millis();

  // --- BUTTON 1: START / STOP ---
  if (digitalRead(BTN_START) == LOW && (now - lastBtnCheck > debounceDelay)) {
    lastBtnCheck = now;
    isRunning = !isRunning;
    if (isRunning) {
      Serial.println(F("[STATE] RUNNING!"));
      digitalWrite(LED_PIN, HIGH);
      lastError = 0;
      integral = 0;
      delay(400); // Small pause before rolling
    } else {
      Serial.println(F("[STATE] STOPPED."));
      stopMotors();
      digitalWrite(LED_PIN, LOW);
    }
  }

  // --- BUTTON 2: CALIBRATE ---
  if (!isRunning && digitalRead(BTN_CALIB) == LOW && (now - lastBtnCheck > debounceDelay)) {
    lastBtnCheck = now;
    calibrateSensors();
  }

  // --- BUTTON 3: TRACK MODE TOGGLE ---
  if (!isRunning && digitalRead(BTN_MODE) == LOW && (now - lastBtnCheck > debounceDelay)) {
    lastBtnCheck = now;
    trackMode = (trackMode == 0) ? 1 : 0;
    Serial.print(F("[MODE] "));
    if (trackMode == 0) {
      Serial.println(F("Black Line on White Surface"));
      flashLED(1, 300);
    } else {
      Serial.println(F("White Line on Black Surface"));
      flashLED(2, 150);
    }
  }

  // --- PID MOTOR CONTROL ---
  if (isRunning) {
    runPID();
  }
}

// ================= PID CONTROL ALGORITHM =================
void runPID() {
  readSensors();
  int position = getPosition();

  // --- CASE 1: LINE LOST (All sensors off the line) ---
  if (position == 9999) {
    // If line was lost, spin toward the direction it was last seen
    if (lastError > 0) {
      // Line was to the right -> Spin right
      setMotors(searchSpeed, -searchSpeed);
    } else {
      // Line was to the left -> Spin left
      setMotors(-searchSpeed, searchSpeed);
    }
    return;
  }

  // --- CASE 2: NORMAL PID TRACKING ---
  // Position is error from center (Center = 0, Left is negative, Right is positive)
  int error = position;

  // PID calculations
  integral += error;
  integral = constrain(integral, -10000, 10000); // Anti-windup
  int derivative = error - lastError;

  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  // Calculate differential motor speeds
  int leftMotorSpeed  = baseSpeed + (int)correction;
  int rightMotorSpeed = baseSpeed - (int)correction;

  // Constrain speeds to limits
  leftMotorSpeed  = constrain(leftMotorSpeed, minSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}

// ================= SENSOR READING & POSITION CALCULATION =================
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    rawValues[i] = analogRead(sensorPins[i]);

    // Normalize value from 0 (background) to 1000 (line)
    int norm = map(rawValues[i], minValues[i], maxValues[i], 0, 1000);
    normValues[i] = constrain(norm, 0, 1000);
  }
}

int getPosition() {
  long numerator = 0;
  long denominator = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    // Invert reading if following a white line on black surface
    int val = (trackMode == 0) ? normValues[i] : (1000 - normValues[i]);

    // Apply noise threshold: ignore weak ambient reflections
    if (val > 200) {
      numerator += (long)val * weights[i];
      denominator += val;
    }
  }

  // If no sensors detect the line
  if (denominator == 0) {
    return 9999; // Lost line indicator
  }

  return (int)(numerator / denominator);
}

// ================= AUTO-CALIBRATION ROUTINE =================
void calibrateSensors() {
  Serial.println(F("\n>>> CALIBRATING: Sweep robot over line for 4 seconds... <<<"));
  stopMotors();

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
    digitalWrite(LED_PIN, (millis() / 80) % 2); // Fast blink
    delay(5);
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    thresholds[i] = (minValues[i] + maxValues[i]) / 2;
  }

  isCalibrated = true;
  digitalWrite(LED_PIN, LOW);

  Serial.println(F(">>> CALIBRATION DONE! Saved dynamic thresholds. <<<"));
  flashLED(3, 100);
}

// ================= MOTOR CONTROL (TB6612FNG) =================
void setMotors(int leftSpeed, int rightSpeed) {
  // Left Motor Direction
  if (leftSpeed >= 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    leftSpeed = -leftSpeed;
  }

  // Right Motor Direction
  if (rightSpeed >= 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    rightSpeed = -rightSpeed;
  }

  // Clamp PWM to 0-255
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  PWM_WRITE(PIN_PWMA, pwmA_ch, leftSpeed);
  PWM_WRITE(PIN_PWMB, pwmB_ch, rightSpeed);
}

void stopMotors() {
  PWM_WRITE(PIN_PWMA, pwmA_ch, 0);
  PWM_WRITE(PIN_PWMB, pwmB_ch, 0);
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);
}

// Helper utility to blink LED
void flashLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    if (i < times - 1) delay(delayMs);
  }
}
