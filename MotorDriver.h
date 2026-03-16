// MotorDriver.h

#pragma once
#include <Arduino.h>

void setupPWM15K();
void setupFanPWM15K();

void setPWM_L(uint16_t v);
void setPWM_R(uint16_t v);

void setFanPWM_L(uint8_t pwm);
void setFanPWM_R(uint8_t pwm);

void driveSafe();
void motorShortBrake();