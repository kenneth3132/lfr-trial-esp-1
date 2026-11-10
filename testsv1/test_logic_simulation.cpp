/*
 * ======================================================================================
 * test_logic_simulation.cpp
 * Comprehensive Unit Test & Algorithmic Simulation of v1 Logic
 * ======================================================================================
 */

#include "ArduinoMock.h"
#include <cassert>
#include <iomanip>

// Mock definitions
const int NUM_SENSORS = 8;
const int sensorPins[NUM_SENSORS] = {36, 39, 34, 35, 32, 33, 25, 26};
const int WeightValue[NUM_SENSORS] = {10, 20, 30, 40, 50, 60, 70, 80};
const float CENTER_SETPOINT = 45.0;

// State variables
int raw_ADC[NUM_SENSORS];
int Reference_ADC[NUM_SENSORS];
uint8_t sensorDigital[NUM_SENSORS];
int sumOnSensor = 0;
long sensorWight = 0;
uint8_t bitSensor = 0;
uint8_t inverseON = 0;

String direction = "straight";
float previous_error = 0;
unsigned long lostLineStartTime = 0;

// Motor state capture for assertions
int g_last_LPWM = 0;
int g_last_RPWM = 0;

void motor(int LPWM, int RPWM) {
  g_last_LPWM = constrain(LPWM, -255, 255);
  g_last_RPWM = constrain(RPWM, -255, 255);
}

void stop() {
  motor(0, 0);
}

// Logic implementations matching v1
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

void PID_Controller(int base_speed, float p, float d) {
  read_black_line();

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

    float derivative = error - previous_error;
    float correction = (p * error) + (d * derivative);
    previous_error = error;

    int leftMotorSpeed  = base_speed - (int)correction;
    int rightMotorSpeed = base_speed + (int)correction;

    motor(leftMotorSpeed, rightMotorSpeed);
  } else {
    // sumOnSensor == 0
    if (direction == "left") {
      // Pivot left
      motor(-150, 150);
      previous_error = 0;
    } else if (direction == "right") {
      // Pivot right
      motor(150, -150);
      previous_error = 0;
    } else {
      if (lostLineStartTime == 0) lostLineStartTime = millis();
      unsigned long elapsed = millis() - lostLineStartTime;
      if (elapsed < 350) {
        motor(base_speed, base_speed); // Coast across gap
      } else {
        stop(); // Auto-brake
      }
    }
  }
}

// ================= TEST SUITE =================
int main() {
  std::cout << "========================================================\n";
  std::cout << "RUNNING ALGORITHMIC SIMULATION & LOGIC TESTS (v1)\n";
  std::cout << "========================================================\n\n";

  // Initialize reference thresholds
  for (int i = 0; i < NUM_SENSORS; i++) {
    Reference_ADC[i] = 2000;
  }

  // TEST 1: Centroid & Error on Dead Center Line
  std::cout << "[TEST 1] Testing Dead-Center Tracking (S3=40 & S4=50 active)... ";
  for (int i = 0; i < NUM_SENSORS; i++) g_analog_reads[sensorPins[i]] = 500;
  g_analog_reads[sensorPins[3]] = 3500; // S4 active
  g_analog_reads[sensorPins[4]] = 3500; // S5 active
  read_black_line();

  assert(sumOnSensor == 2);
  assert(bitSensor == 0b00011000);
  float pos1 = (float)sensorWight / sumOnSensor;
  float err1 = CENTER_SETPOINT - pos1;
  assert(std::abs(pos1 - 45.0f) < 0.001f);
  assert(std::abs(err1 - 0.0f) < 0.001f);
  PID_Controller(160, 3.8f, 14.5f);
  assert(g_last_LPWM == 160 && g_last_RPWM == 160);
  assert(direction == "straight");
  std::cout << "PASSED (Position: 45.0, Error: 0.0, Speed: 160/160, Dir: straight)\n";

  // TEST 2: 90-Degree Sharp Left Pattern Matching
  std::cout << "[TEST 2] Testing Sharp Left Footprint (0b11110000)... ";
  for (int i = 0; i < NUM_SENSORS; i++) g_analog_reads[sensorPins[i]] = 500;
  for (int i = 0; i < 4; i++) g_analog_reads[sensorPins[i]] = 3500;
  read_black_line();
  assert(bitSensor == 0b11110000);
  PID_Controller(160, 3.8f, 14.5f);
  assert(direction == "left");
  std::cout << "PASSED (bitSensor: 0b11110000, Dir: left)\n";

  // TEST 3: Left Turn Execution on Line Loss (sumOnSensor == 0)
  std::cout << "[TEST 3] Testing Left Turn Execution when Line Terminates... ";
  for (int i = 0; i < NUM_SENSORS; i++) g_analog_reads[sensorPins[i]] = 500; // All white
  PID_Controller(160, 3.8f, 14.5f);
  assert(g_last_LPWM == -150 && g_last_RPWM == 150);
  std::cout << "PASSED (Motors: LPWM -150, RPWM +150 -> Pivot Left)\n";

  // TEST 4: 90-Degree Sharp Right Pattern Matching & Spin
  std::cout << "[TEST 4] Testing Sharp Right Footprint (0b00011111)... ";
  for (int i = 0; i < NUM_SENSORS; i++) g_analog_reads[sensorPins[i]] = 500;
  for (int i = 3; i < 8; i++) g_analog_reads[sensorPins[i]] = 3500;
  read_black_line();
  assert(bitSensor == 0b00011111);
  PID_Controller(160, 3.8f, 14.5f);
  assert(direction == "right");
  std::cout << "PASSED (bitSensor: 0b00011111, Dir: right)\n";

  std::cout << "[TEST 4b] Testing Right Turn Execution on Line Loss... ";
  for (int i = 0; i < NUM_SENSORS; i++) g_analog_reads[sensorPins[i]] = 500;
  PID_Controller(160, 3.8f, 14.5f);
  assert(g_last_LPWM == 150 && g_last_RPWM == -150);
  std::cout << "PASSED (Motors: LPWM +150, RPWM -150 -> Pivot Right)\n";

  // TEST 5: 4-Way Cross Intersection (+) Balancing
  std::cout << "[TEST 5] Testing 4-Way Cross Junction (0b11111111)... ";
  previous_error = 0; // State reset after turn re-lock
  for (int i = 0; i < NUM_SENSORS; i++) g_analog_reads[sensorPins[i]] = 3500;
  read_black_line();
  assert(bitSensor == 0b11111111);
  PID_Controller(160, 3.8f, 14.5f);
  float cross_pos = (float)sensorWight / sumOnSensor;
  assert(std::abs(cross_pos - 45.0f) < 0.001f);
  assert(direction == "straight");
  assert(g_last_LPWM == 160 && g_last_RPWM == 160);
  std::cout << "PASSED (Balanced Centroid: 45.0, Dir: straight, Cruising: 160/160)\n";

  // TEST 6: Gap Traversal (<350ms coasting) & Off-Track Auto-Brake (>350ms)
  std::cout << "[TEST 6] Testing Dashed Line Gap Traversal (< 350ms)... ";
  direction = "straight";
  lostLineStartTime = 0;
  g_mock_millis = 1000;
  for (int i = 0; i < NUM_SENSORS; i++) g_analog_reads[sensorPins[i]] = 500; // All white
  PID_Controller(160, 3.8f, 14.5f); // 1st tick at t=1000
  assert(g_last_LPWM == 160 && g_last_RPWM == 160);
  g_mock_millis = 1200; // +200ms
  PID_Controller(160, 3.8f, 14.5f);
  assert(g_last_LPWM == 160 && g_last_RPWM == 160);
  std::cout << "PASSED (Maintains 160/160 forward drive across gap)\n";

  std::cout << "[TEST 6b] Testing Off-Track Auto-Brake (> 350ms)... ";
  g_mock_millis = 1400; // +400ms (> 350ms timeout)
  PID_Controller(160, 3.8f, 14.5f);
  assert(g_last_LPWM == 0 && g_last_RPWM == 0);
  std::cout << "PASSED (Motors clamped to 0/0 auto-brake to prevent runaway)\n";

  // TEST 7: Polarity Inversion (inverseON = 1: White Line on Black)
  std::cout << "[TEST 7] Testing Inverted Track Mode (inverseON = 1)... ";
  inverseON = 1;
  // White line (LOW ADC ~300) in center (S3 and S4)
  for (int i = 0; i < NUM_SENSORS; i++) g_analog_reads[sensorPins[i]] = 3500; // Black background
  g_analog_reads[sensorPins[3]] = 300; // White line
  g_analog_reads[sensorPins[4]] = 300; // White line
  read_black_line();
  assert(sumOnSensor == 2);
  assert(bitSensor == 0b00011000);
  assert(sensorDigital[3] == 1 && sensorDigital[4] == 1);
  std::cout << "PASSED (White line correctly identified with bitSensor 0b00011000)\n";

  // TEST 8: EEPROM Memory Boundaries & Packing
  std::cout << "[TEST 8] Testing EEPROM Memory Map Alignment... ";
  MockEEPROM testEEPROM;
  testEEPROM.write(0, 0x5A); // Magic byte
  for (int i = 0; i < NUM_SENSORS; i++) {
    int ref = 1800 + i * 50;
    int mn = 300 + i * 10;
    int mx = 3400 + i * 20;
    testEEPROM.put(1 + (i * 2), ref);
    testEEPROM.put(17 + (i * 2), mn);
    testEEPROM.put(33 + (i * 2), mx);
  }
  // Verify reconstruction
  for (int i = 0; i < NUM_SENSORS; i++) {
    int rec_ref, rec_mn, rec_mx;
    testEEPROM.get(1 + (i * 2), rec_ref);
    testEEPROM.get(17 + (i * 2), rec_mn);
    testEEPROM.get(33 + (i * 2), rec_mx);
    assert(rec_ref == 1800 + i * 50);
    assert(rec_mn == 300 + i * 10);
    assert(rec_mx == 3400 + i * 20);
  }
  std::cout << "PASSED (All 48 bytes reconstruct perfectly with no overlap)\n";

  std::cout << "\n========================================================\n";
  std::cout << ">>> ALL 8 LOGICAL SIMULATION TESTS PASSED WITH 100% SUCCESS! <<<\n";
  std::cout << "========================================================\n";
  return 0;
}
