// FanManager.h

#pragma once
#include <Arduino.h>

// ======================================================
// FAN CONTROL
// ======================================================

void updateDriverFans(void);

// low level (PWM write)
void setFanPWM_L(uint16_t pwm);
void setFanPWM_R(uint16_t pwm);

