/*
 * ======================================================================================
 * v1 ESP32 HIGH-SPEED PID LINE FOLLOWER ROBOT (Single File Build)
 * ======================================================================================
 * Author: Kenneth (@kenneth3132)
 * Hardware Connections (Identical to robot_connections.txt):
 *   - SmartElex RLS-08 : S1(36), S2(39), S3(34), S4(35), S5(32), S6(33), S7(25), S8(26)
 *   - TB6612FNG Driver : PWMA(17), AIN1(4), AIN2(16), PWMB(15), BIN1(5), BIN2(18), STBY(19)
 *   - Push Buttons     : Start/Stop(27), Calibrate(14), Mode(13)
 *   - Status LED       : GPIO 2 (Onboard Blue LED)
 * 
 * Advanced Navigation Logic:
 *   1. Pre-Turn Pattern Memory: Evaluates 8-bit bitmask (bitSensor) for 90° corner footprints.
 *   2. Axle Alignment Creep: Moves forward 80ms to place wheel axle over vertex before turn.
 *   3. In-Place Pivot Spin: Differential spin with line re-lock on center sensors S3/S4.
 *   4. Active Electronic Braking: motor(0, 0) eliminates rotational momentum on re-lock.
 *   5. Safety Watchdog: 1500ms timeout aborts spin if robot is lifted or runs off-track.
 *   6. Closed-Loop PD Tracking: Centroid line position against setpoint 45.0.
 *   7. Cross Junction Balance: Symmetrical centroid drives straight through 4-way crosses (+).
 *   8. Gap Traversal & Auto-Brake: 350ms coasting across dashed lines, then auto-brakes.
 *   9. 4-Sweep Auto-Calibration: Dynamic midpoints saved to non-volatile EEPROM.
 * ======================================================================================
 */

#include <Arduino.h>
#include <EEPROM.h>

// Universal ESP32 Core 2.x & 3.x Compatibility for LEDC PWM
#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // ESP32 Arduino Core 3.x syntax
  #define PWM_SETUP(pin, ch, freq, res) ledcAttach(pin, freq, res)
  #define PWM_WRITE(pin, ch, val)       ledcWrite(pin, val)
#else
  // ESP32 Arduino Core 2.x syntax
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
#define PIN_STBY  19   // Standby (HIGH = Driver Enabled)

// 3 Push Buttons (Internal PULL-UP)
#define BTN_START 27   // Button 1: Start / Stop
#define BTN_CALIB 14   // Button 2: Auto-Calibrate on Track
#define BTN_MODE  13   // Button 3: Toggle Black / White Track Mode

// Status LED Pin
#define LED_PIN   2

// ================= PWM SETTINGS =================
const int pwmFreq = 10000; // 10 kHz
const int pwmRes  = 8;     // 8-bit resolution (0 to 255)
const int pwmA_ch = 0;
const int pwmB_ch = 1;

// ================= SENSOR WEIGHTS & CENTROID =================
const int WeightValue[NUM_SENSORS] = {10, 20, 30, 40, 50, 60, 70, 80};
const float CENTER_SETPOINT = 45.0;

// ================= MOTION CONSTANTS =================
const int DEFAULT_BASE_SPEED    = 160;
const int DEFAULT_TURN_SPEED    = 150;
const int DEFAULT_CREEP_SPEED   = 100;
const unsigned long DELAY_BEFORE_TURN = 80;   // ms forward creep before spin
const unsigned long WATCHDOG_TIMEOUT  = 1500; // ms max spin watchdog
const unsigned long GAP_TIMEOUT       = 350;  // ms max coast across dashed line

float Kp = 3.80;
float Kd = 14.50;

// ================= STATE VARIABLES =================
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

// EEPROM layout
const int EEPROM_SIZE = 64;
const uint8_t EEPROM_MAGIC = 0x5A;

// Forward declarations
void motor(int LPWM, int RPWM);
void stop();
void read_black_line();
void turnLeft(int L, int R);
void turnRight(int L, int R);
void PID_Controller(int base_speed, float p, float d);
void calibrateSensor();
void LoadCalibration();
void flashLED(int times, int delayMs);
void Bit_Sensor_Show();

// ================= MOTOR CONTROL =================
void motor(int LPWM, int RPWM) {
  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);

  // Left Motor
  if (LPWM > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    PWM_WRITE(PIN_PWMA, pwmA_ch, LPWM);
  } else if (LPWM < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    PWM_WRITE(PIN_PWMA, pwmA_ch, -LPWM);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    PWM_WRITE(PIN_PWMA, pwmA_ch, 0);
  }

  // Right Motor
  if (RPWM > 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    PWM_WRITE(PIN_PWMB, pwmB_ch, RPWM);
  } else if (RPWM < 0) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    PWM_WRITE(PIN_PWMB, pwmB_ch, -RPWM);
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    PWM_WRITE(PIN_PWMB, pwmB_ch, 0);
  }
}

void stop() {
  motor(0, 0);
}

// ================= SENSOR ACQUISITION =================
void read_black_line() {
  sumOnSensor = 0;
  sensorWight = 0;
  bitSensor = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    raw_ADC[i] = analogRead(sensorPins[i]);

    if (inverseON == 0) {
      sensorDigital[i] = (raw_ADC[i] > Reference_ADC[i]) ? 1 : 0;
    } else {
      sensorDigital[i] = (raw_ADC[i] <= Reference_ADC[i]) ? 1 : 0;
    }

    if (sensorDigital[i] == 1) {
      sumOnSensor++;
      sensorWight += (long)WeightValue[i];
      bitSensor |= (1 << (7 - i));
    }
  }
}

// ================= SHARP TURNS & RE-LOCK =================
void turnRight(int L, int R) {
  // 1. Axle alignment creep forward
  motor(DEFAULT_CREEP_SPEED, DEFAULT_CREEP_SPEED);
  delay(DELAY_BEFORE_TURN);

  // 2. In-place pivot (Left forward, Right reverse)
  motor(L, -R);
  unsigned long start = millis();

  // 3. Spin until center sensor S3 or S4 re-locks onto the line
  while (true) {
    read_black_line();
    if (sensorDigital[3] == 1 || sensorDigital[4] == 1) {
      break;
    }
    if (millis() - start > WATCHDOG_TIMEOUT) {
      Serial.println(F("[WATCHDOG] Right turn timeout! Aborting."));
      break;
    }
    delay(2);
  }

  // 4. Active counter-brake to eliminate inertia
  stop();
  delay(30);
  direction = "straight";
}

void turnLeft(int L, int R) {
  // 1. Axle alignment creep forward
  motor(DEFAULT_CREEP_SPEED, DEFAULT_CREEP_SPEED);
  delay(DELAY_BEFORE_TURN);

  // 2. In-place pivot (Left reverse, Right forward)
  motor(-L, R);
  unsigned long start = millis();

  // 3. Spin until center sensor S3 or S4 re-locks onto the line
  while (true) {
    read_black_line();
    if (sensorDigital[3] == 1 || sensorDigital[4] == 1) {
      break;
    }
    if (millis() - start > WATCHDOG_TIMEOUT) {
      Serial.println(F("[WATCHDOG] Left turn timeout! Aborting."));
      break;
    }
    delay(2);
  }

  // 4. Active counter-brake
  stop();
  delay(30);
  direction = "straight";
}

// ================= PID & NAVIGATION LOOP =================
void PID_Controller(int base_speed, float p, float d) {
  read_black_line();

  // CASE 1: LINE DETECTED
  if (sumOnSensor > 0) {
    lostLineStartTime = 0;

    float line_position = (float)sensorWight / sumOnSensor;
    float error = CENTER_SETPOINT - line_position;

    // Pattern Matching on bitSensor
    if (bitSensor == 0b11111111) {
      direction = "straight"; // 4-way cross: continue straight
    } else if ((bitSensor & 0b11100000) == 0b11100000 || bitSensor == 0b11000000 || bitSensor == 0b11111100) {
      direction = "left";     // Sharp left corner / left T
    } else if ((bitSensor & 0b00000111) == 0b00000111 || bitSensor == 0b00000011 || bitSensor == 0b00111111) {
      direction = "right";    // Sharp right corner / right T
    } else {
      direction = "straight"; // Center / mild curves
    }

    // PD steering
    float derivative = error - previous_error;
    float correction = (p * error) + (d * derivative);
    previous_error = error;

    int leftMotorSpeed  = base_speed - (int)correction;
    int rightMotorSpeed = base_speed + (int)correction;

    motor(leftMotorSpeed, rightMotorSpeed);
  }
  // CASE 2: ALL SENSORS OFF LINE
  else {
    if (direction == "left") {
      turnLeft(DEFAULT_TURN_SPEED, DEFAULT_TURN_SPEED);
      previous_error = 0;
      lostLineStartTime = 0;
    } else if (direction == "right") {
      turnRight(DEFAULT_TURN_SPEED, DEFAULT_TURN_SPEED);
      previous_error = 0;
      lostLineStartTime = 0;
    } else {
      // Straight line gap traversal
      if (lostLineStartTime == 0) lostLineStartTime = millis();
      unsigned long elapsed = millis() - lostLineStartTime;

      if (elapsed < GAP_TIMEOUT) {
        motor(base_speed, base_speed); // Coast across dashed line
      } else {
        stop(); // Safety brake
      }
    }
  }
}

// ================= AUTO-CALIBRATION =================
void calibrateSensor() {
  Serial.println(F("\n>>> CALIBRATING: Performing 4-sweep rotational scan... <<<"));
  stop();

  for (int i = 0; i < NUM_SENSORS; i++) {
    Min_ADC[i] = 4095;
    Max_ADC[i] = 0;
  }

  const int calibSpeed = 110;

  // Sweep Left (800ms)
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

  // Sweep Right (1600ms)
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

  // Sweep Back to Center (800ms)
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

  // Validate channels & set midpoint thresholds
  for (int i = 0; i < NUM_SENSORS; i++) {
    int contrast = Max_ADC[i] - Min_ADC[i];
    if (contrast >= 150) {
      Reference_ADC[i] = (Min_ADC[i] + Max_ADC[i]) / 2;
    } else {
      Reference_ADC[i] = 2000;
    }
  }

  // Save to EEPROM
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(0, EEPROM_MAGIC);
  for (int i = 0; i < NUM_SENSORS; i++) {
    EEPROM.put(1 + (i * 2), Reference_ADC[i]);
    EEPROM.put(17 + (i * 2), Min_ADC[i]);
    EEPROM.put(33 + (i * 2), Max_ADC[i]);
  }
  EEPROM.commit();

  digitalWrite(LED_PIN, LOW);
  Serial.println(F(">>> CALIBRATION SAVED TO EEPROM! <<<"));
  flashLED(3, 100);
}

void LoadCalibration() {
  EEPROM.begin(EEPROM_SIZE);
  uint8_t magic = EEPROM.read(0);

  if (magic == EEPROM_MAGIC) {
    Serial.println(F("[EEPROM] Calibration loaded from flash."));
    for (int i = 0; i < NUM_SENSORS; i++) {
      EEPROM.get(1 + (i * 2), Reference_ADC[i]);
      EEPROM.get(17 + (i * 2), Min_ADC[i]);
      EEPROM.get(33 + (i * 2), Max_ADC[i]);
    }
  } else {
    Serial.println(F("[EEPROM] No saved calibration. Defaults set to 2000."));
    for (int i = 0; i < NUM_SENSORS; i++) {
      Min_ADC[i] = 400;
      Max_ADC[i] = 3600;
      Reference_ADC[i] = 2000;
    }
  }
}

void flashLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    if (i < times - 1) delay(delayMs);
  }
}

void Bit_Sensor_Show() {
  read_black_line();
  Serial.print(F("bitSensor: 0b"));
  for (int i = 7; i >= 0; i--) {
    Serial.print((bitSensor >> i) & 1);
  }
  Serial.print(F(" | Active: ")); Serial.print(sumOnSensor);
  if (sumOnSensor > 0) {
    float pos = (float)sensorWight / sumOnSensor;
    Serial.print(F(" | Pos: ")); Serial.print(pos, 1);
    Serial.print(F(" | Err: ")); Serial.print(CENTER_SETPOINT - pos, 1);
  } else {
    Serial.print(F(" | Pos: LOST"));
  }
  Serial.print(F(" | Mode: "));
  Serial.println(inverseON == 0 ? F("BLACK") : F("WHITE"));
}

// ================= SETUP & LOOP =================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("\n========================================================"));
  Serial.println(F("   v1 ESP32 Advanced PID Line Follower Starting        "));
  Serial.println(F("========================================================"));

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_CALIB, INPUT_PULLUP);
  pinMode(BTN_MODE,  INPUT_PULLUP);
  pinMode(LED_PIN,   OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  PWM_SETUP(PIN_PWMA, pwmA_ch, pwmFreq, pwmRes);
  PWM_SETUP(PIN_PWMB, pwmB_ch, pwmFreq, pwmRes);
  stop();

  LoadCalibration();
  flashLED(3, 100);
  Serial.println(F("[READY] Press Button 1 to Start/Stop, Button 2 to Calibrate."));
}

void loop() {
  unsigned long now = millis();

  // Button 1: Start / Stop
  if (digitalRead(BTN_START) == LOW && (now - lastBtnCheck > debounceDelay)) {
    lastBtnCheck = now;
    isRunning = !isRunning;
    if (isRunning) {
      Serial.println(F("[STATE] RUNNING!"));
      digitalWrite(LED_PIN, HIGH);
      previous_error = 0;
      lostLineStartTime = 0;
      direction = "straight";
      delay(300);
    } else {
      Serial.println(F("[STATE] STOPPED."));
      stop();
      digitalWrite(LED_PIN, LOW);
    }
  }

  // Button 2: Calibrate
  if (!isRunning && digitalRead(BTN_CALIB) == LOW && (now - lastBtnCheck > debounceDelay)) {
    lastBtnCheck = now;
    calibrateSensor();
  }

  // Button 3: Track Mode
  if (!isRunning && digitalRead(BTN_MODE) == LOW && (now - lastBtnCheck > debounceDelay)) {
    lastBtnCheck = now;
    inverseON = (inverseON == 0) ? 1 : 0;
    if (inverseON == 0) {
      Serial.println(F("[MODE] Black Line on White Surface"));
      flashLED(1, 300);
    } else {
      Serial.println(F("[MODE] White Line on Black Surface"));
      flashLED(2, 150);
    }
  }

  // PID Execution
  if (isRunning) {
    PID_Controller(DEFAULT_BASE_SPEED, Kp, Kd);
  } else {
    static unsigned long lastTele = 0;
    if (now - lastTele >= 500) {
      lastTele = now;
      Bit_Sensor_Show();
    }
  }
}
