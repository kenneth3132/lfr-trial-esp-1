#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

inline void initMotors() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH); // Enable driver

  PWM_SETUP(PIN_PWMA, pwmA_ch, pwmFreq, pwmRes);
  PWM_SETUP(PIN_PWMB, pwmB_ch, pwmFreq, pwmRes);
}

inline void setMotors(int leftSpeed, int rightSpeed) {
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

  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  PWM_WRITE(PIN_PWMA, pwmA_ch, leftSpeed);
  PWM_WRITE(PIN_PWMB, pwmB_ch, rightSpeed);
}

inline void stopMotors() {
  PWM_WRITE(PIN_PWMA, pwmA_ch, 0);
  PWM_WRITE(PIN_PWMB, pwmB_ch, 0);
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);
}

#endif // MOTORS_H
