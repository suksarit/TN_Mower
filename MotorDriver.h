//MotorDriver.h

#pragma once
#include <Arduino.h>

// PWM
void setupPWM15K(void);
void setupFanPWM15K(void);

// MOTOR
void setPWM_L(uint16_t v);
void setPWM_R(uint16_t v);

// FAN
void setFanPWM_L(uint16_t pwm);
void setFanPWM_R(uint16_t pwm);

// SAFETY
void driveSafe(void);
void motorShortBrake(void);