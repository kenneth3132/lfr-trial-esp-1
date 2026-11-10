#ifndef PID_H
#define PID_H

#include "config.h"
#include "sensors.h"
#include "motors.h"

// PID Tuning Parameters
extern float Kp;
extern float Ki;
extern float Kd;
extern int baseSpeed;
extern int maxSpeed;
extern int minSpeed;
extern int searchSpeed;

extern int lastError;
extern float integral;

inline void runPID() {
  readSensors();
  int position = getPosition();

  // Lost line handling
  if (position == 9999) {
    if (lastError > 0) {
      setMotors(searchSpeed, -searchSpeed); // Spin right
    } else {
      setMotors(-searchSpeed, searchSpeed); // Spin left
    }
    return;
  }

  int error = position;
  integral += error;
  integral = constrain(integral, -10000, 10000);
  int derivative = error - lastError;

  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  int leftMotorSpeed  = baseSpeed + (int)correction;
  int rightMotorSpeed = baseSpeed - (int)correction;

  leftMotorSpeed  = constrain(leftMotorSpeed, minSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}

#endif // PID_H
