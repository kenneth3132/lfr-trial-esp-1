/*
 * ======================================================================================
 * v1 Modular Line Follower: Configuration & Hardware Pinout
 * ======================================================================================
 * Hardware mapping matches robot_connections.txt:
 *   - SmartElex RLS-08 : 8 Analog Channels (GPIO 36, 39, 34, 35, 32, 33, 25, 26)
 *   - TB6612FNG Driver : PWMA(17), AIN1(4), AIN2(16), PWMB(15), BIN1(5), BIN2(18), STBY(19)
 *   - Push Buttons     : Start/Stop(27), Calibrate(14), Mode(13)
 *   - Status LED       : GPIO 2 (Onboard Blue LED)
 * ======================================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

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
// 8-Channel Sensor Pins (SmartElex RLS-08 connected to ESP32 ADC)
const int NUM_SENSORS = 8;
const int sensorPins[NUM_SENSORS] = {36, 39, 34, 35, 32, 33, 25, 26};

// TB6612FNG Motor Driver Pins
#define PIN_PWMA  17   // Left Motor Speed Control (PWM)
#define PIN_AIN1  4    // Left Motor Direction 1
#define PIN_AIN2  16   // Left Motor Direction 2
#define PIN_PWMB  15   // Right Motor Speed Control (PWM)
#define PIN_BIN1  5    // Right Motor Direction 1
#define PIN_BIN2  18   // Right Motor Direction 2
#define PIN_STBY  19   // Standby (HIGH = Driver Enabled)

// 3 Push Buttons (Internal PULL-UP: Pressed = LOW, Released = HIGH)
#define BTN_START 27   // Button 1: Start / Stop PID run
#define BTN_CALIB 14   // Button 2: Auto-Calibration on track
#define BTN_MODE  13   // Button 3: Toggle Black / White line mode

// Status LED (Built-in on ESP32 DevKit)
#define LED_PIN   2

// ================= PWM SETTINGS =================
const int pwmFreq = 10000; // 10 kHz for smooth & silent motor operation
const int pwmRes  = 8;     // 8-bit resolution (0 to 255)
const int pwmA_ch = 0;     // Channel 0 (for Core 2.x)
const int pwmB_ch = 1;     // Channel 1 (for Core 2.x)

// ================= SENSOR WEIGHTS & CENTROID SETPOINT =================
// Weights for 8 sensors (S0 to S7):
const int WeightValue[NUM_SENSORS] = {10, 20, 30, 40, 50, 60, 70, 80};
// Perfect center setpoint is between S3 (40) and S4 (50) -> 45.0
const float CENTER_SETPOINT = 45.0;

// ================= MOTION & NAVIGATION PARAMETERS =================
const int DEFAULT_BASE_SPEED    = 160;   // Cruising speed on straight/mild curves
const int DEFAULT_TURN_SPEED    = 150;   // Speed during 90° differential spin
const int DEFAULT_CREEP_SPEED   = 100;   // Axle-alignment creep speed (60% of base)
const unsigned long DELAY_BEFORE_TURN = 80;   // ms to creep forward before pivoting
const unsigned long WATCHDOG_TIMEOUT  = 1500; // Max ms to search before safety abort
const unsigned long GAP_TIMEOUT       = 350;  // Max ms to coast across dashed line gap

// Default PID Gains
const float DEFAULT_KP = 3.80;  // Proportional gain for centroid error
const float DEFAULT_KD = 14.50; // Derivative gain for damping oscillations

#endif // CONFIG_H
