#include <Arduino.h>

#include "FanManager.h"
#include "GlobalState.h"
#include "HardwareConfig.h"
#include "MotorDriver.h"
#include "SafetyManager.h"

// ======================================================
// CONFIG
// ======================================================

constexpr int16_t FAN_L_START_C = 55;
constexpr int16_t FAN_L_FULL_C  = 85;

constexpr int16_t FAN_R_START_C = 60;
constexpr int16_t FAN_R_FULL_C  = 88;

constexpr uint8_t FAN_MIN_PWM  = 80;
constexpr uint8_t FAN_PWM_HYST = 8;
constexpr uint8_t FAN_IDLE_PWM = 50;

// cooldown
constexpr uint32_t FAN_COOLDOWN_MS = 10000;

// emergency
constexpr int16_t FAN_EMERGENCY_TEMP = 95;

// sensor fail guard
constexpr int16_t TEMP_MIN_VALID = -20;
constexpr int16_t TEMP_MAX_VALID = 150;

// ======================================================
// STATE MACHINE
// ======================================================

enum class FanState : uint8_t
{
  NORMAL,
  COOLING,
  EMERGENCY
};

static FanState fanState = FanState::NORMAL;
static uint32_t lastHotTime = 0;

// ======================================================
// INTERNAL
// ======================================================

static uint8_t calcFanPwm(int temp, int tStart, int tFull)
{
  if (temp < tStart) return 0;
  if (temp >= tFull) return 255;

  int mid = (tStart + tFull) / 2;

  if (temp <= mid)
  {
    return FAN_MIN_PWM +
      (temp - tStart) * (140 - FAN_MIN_PWM) / (mid - tStart);
  }
  else
  {
    return 140 +
      (temp - mid) * (255 - 140) / (tFull - mid);
  }
}

// ======================================================
// VALIDATE TEMP
// ======================================================

static bool isTempValid(int t)
{
  return (t > TEMP_MIN_VALID && t < TEMP_MAX_VALID);
}

// ======================================================
// MAIN FAN CONTROL (STABLE VERSION)
// ======================================================

void updateDriverFans(void)
{
  uint32_t now = millis();

  static uint8_t lastPwmL = 0;
  static uint8_t lastPwmR = 0;

  int tL = tempDriverL;
  int tR = tempDriverR;

  // --------------------------------------------------
  // SENSOR FAIL SAFE
  // --------------------------------------------------
  if (!isTempValid(tL) || !isTempValid(tR))
  {
    // เปิดพัดลมเต็ม → ปลอดภัยสุด
    setFanPWM_L(PWM_TOP);
    setFanPWM_R(PWM_TOP);
    return;
  }

  int tMax = max(tL, tR);

  // ==================================================
  // STATE TRANSITION
  // ==================================================

  // EMERGENCY PRIORITY
  if (tMax >= FAN_EMERGENCY_TEMP ||
      getDriveSafety() == SafetyState::EMERGENCY)
  {
    fanState = FanState::EMERGENCY;
  }
  else
  {
    switch (fanState)
    {
      case FanState::NORMAL:

        if (tL >= FAN_L_FULL_C - 5 ||
            tR >= FAN_R_FULL_C - 5)
        {
          fanState = FanState::COOLING;
          lastHotTime = now;
        }
        break;

      case FanState::COOLING:

        if (tL >= FAN_L_FULL_C - 5 ||
            tR >= FAN_R_FULL_C - 5)
        {
          lastHotTime = now;
        }

        // cooldown hold
        if (now - lastHotTime > FAN_COOLDOWN_MS)
        {
          fanState = FanState::NORMAL;
        }
        break;

      case FanState::EMERGENCY:

        // hysteresis exit (กันสลับ state)
        if (tMax < FAN_L_FULL_C - 3)
        {
          fanState = FanState::COOLING;
          lastHotTime = now;
        }
        break;
    }
  }

  // ==================================================
  // OUTPUT LOGIC
  // ==================================================

  uint8_t pwmL = 0;
  uint8_t pwmR = 0;

  switch (fanState)
  {
    case FanState::NORMAL:
      pwmL = calcFanPwm(tL, FAN_L_START_C, FAN_L_FULL_C);
      pwmR = calcFanPwm(tR, FAN_R_START_C, FAN_R_FULL_C);
      break;

    case FanState::COOLING:
      pwmL = 200;
      pwmR = 200;
      break;

    case FanState::EMERGENCY:
      pwmL = 255;
      pwmR = 255;
      break;
  }

  // ==================================================
  // HYSTERESIS (PWM LEVEL)
  // ==================================================
  if (abs((int)pwmL - (int)lastPwmL) < FAN_PWM_HYST) pwmL = lastPwmL;
  if (abs((int)pwmR - (int)lastPwmR) < FAN_PWM_HYST) pwmR = lastPwmR;

  // ==================================================
  // IDLE SPIN
  // ==================================================
  if (pwmL == 0 && tL >= FAN_L_START_C) pwmL = FAN_IDLE_PWM;
  if (pwmR == 0 && tR >= FAN_R_START_C) pwmR = FAN_IDLE_PWM;

  // ==================================================
  // OUTPUT
  // ==================================================
  setFanPWM_L((uint16_t)map(pwmL, 0, 255, 0, PWM_TOP));
  setFanPWM_R((uint16_t)map(pwmR, 0, 255, 0, PWM_TOP));

  lastPwmL = pwmL;
  lastPwmR = pwmR;
}