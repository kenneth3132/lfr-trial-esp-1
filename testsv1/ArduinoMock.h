/*
 * ======================================================================================
 * ArduinoMock.h - Mock Arduino & ESP32 Environment for Host-Side Verification
 * ======================================================================================
 */

#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <algorithm>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define F(x) x

typedef std::string String;

// Mock Serial Output
struct MockSerial {
  template <typename T>
  MockSerial& print(const T& val) {
    std::cout << val;
    return *this;
  }
  template <typename T>
  MockSerial& print(const T& val, int) {
    std::cout << val;
    return *this;
  }
  template <typename T>
  MockSerial& println(const T& val) {
    std::cout << val << std::endl;
    return *this;
  }
  MockSerial& println() {
    std::cout << std::endl;
    return *this;
  }
  void begin(unsigned long) {}
};

static MockSerial Serial;

// Mock Timing
static unsigned long g_mock_millis = 1000;
inline unsigned long millis() { return g_mock_millis; }
inline void delay(unsigned long ms) { g_mock_millis += ms; }

// Mock GPIO & ADC State
static int g_pin_modes[40] = {0};
static int g_digital_writes[40] = {0};
static int g_digital_reads[40] = {1}; // Default pullup HIGH
static int g_analog_reads[40] = {0};

inline void pinMode(int pin, int mode) {
  if (pin >= 0 && pin < 40) g_pin_modes[pin] = mode;
}

inline void digitalWrite(int pin, int val) {
  if (pin >= 0 && pin < 40) g_digital_writes[pin] = val;
}

inline int digitalRead(int pin) {
  if (pin >= 0 && pin < 40) return g_digital_reads[pin];
  return HIGH;
}

inline int analogRead(int pin) {
  if (pin >= 0 && pin < 40) return g_analog_reads[pin];
  return 0;
}

// Mock PWM
static int g_pwm_duty[40] = {0};
inline void PWM_SETUP(int, int, int, int) {}
inline void PWM_WRITE(int pin, int, int val) {
  if (pin >= 0 && pin < 40) g_pwm_duty[pin] = val;
}

// Mock EEPROM
struct MockEEPROM {
  uint8_t memory[512] = {0};
  void begin(size_t) {}
  uint8_t read(int addr) { return memory[addr]; }
  void write(int addr, uint8_t val) { memory[addr] = val; }
  template <typename T>
  T& get(int addr, T& val) {
    std::memcpy(&val, &memory[addr], sizeof(T));
    return val;
  }
  template <typename T>
  const T& put(int addr, const T& val) {
    std::memcpy(&memory[addr], &val, sizeof(T));
    return val;
  }
  bool commit() { return true; }
};

static MockEEPROM EEPROM;

#endif // ARDUINO_MOCK_H
