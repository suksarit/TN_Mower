// FanManager.cpp

#include <Arduino.h>

void setFanPWM_L(uint8_t pwm)
{
  analogWrite(6, pwm);
}

void setFanPWM_R(uint8_t pwm)
{
  analogWrite(7, pwm);
}