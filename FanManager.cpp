//FanManager.cpp

#include <Arduino.h>

#include "FanManager.h"
#include "MotorDriver.h"
#include "ThermalManager.h"
#include "HardwareConfig.h"

// ======================================================
// SIMPLE FAN DRIVER (NO LOGIC)
// ======================================================

void updateDriverFans(void)
{
  uint8_t level = getThermalFanLevel();

  // map → PWM_TOP
  uint16_t pwm =
    (uint16_t)map(level, 0, 255, 0, PWM_TOP);

  setFanPWM_L(pwm);
  setFanPWM_R(pwm);
}