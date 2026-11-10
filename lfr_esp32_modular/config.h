#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Universal ESP32 Core 2.x & 3.x Compatibility for LEDC PWM
#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define PWM_SETUP(pin, ch, freq, res) ledcAttach(pin, freq, res)
  #define PWM_WRITE(pin, ch, val)       ledcWrite(pin, val)
#else
  #define PWM_SETUP(pin, ch, freq, res) do { ledcSetup(ch, freq, res); ledcAttachPin(pin, ch); } while(0)
  #define PWM_WRITE(pin, ch, val)       ledcWrite(ch, val)
#endif

// ================= HARDWARE PIN DEFINITIONS =================
// 8-Channel Sensor Pins (SmartElex RLS-08)
const int NUM_SENSORS = 8;
const int sensorPins[NUM_SENSORS] = {36, 39, 34, 35, 32, 33, 25, 26};

// TB6612FNG Motor Driver Pins
#define PIN_PWMA  17   // Left Motor Speed (PWM)
#define PIN_AIN1  4    // Left Motor Direction 1
#define PIN_AIN2  16   // Left Motor Direction 2
#define PIN_PWMB  15   // Right Motor Speed (PWM)
#define PIN_BIN1  5    // Right Motor Direction 1
#define PIN_BIN2  18   // Right Motor Direction 2
#define PIN_STBY  19   // Standby (HIGH = Motors Enabled)

// 3 Push Buttons (Internal PULL-UP)
#define BTN_START 27   // Button 1: Start / Stop
#define BTN_CALIB 14   // Button 2: Auto-Calibrate on Track
#define BTN_MODE  13   // Button 3: Black/White Track Mode

// Status LED Pin
#define LED_PIN   2

// PWM Configuration
const int pwmFreq = 10000; // 10 kHz
const int pwmRes  = 8;     // 8-bit (0-255)
const int pwmA_ch = 0;
const int pwmB_ch = 1;

// Sensor Weights (-3500 to +3500, Center is 0)
const int weights[NUM_SENSORS] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

#endif // CONFIG_H
