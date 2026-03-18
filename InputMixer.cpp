// ============================================================================
// InputMixer.cpp (FIXED + STABLE + SAFE CONTROL)
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "HardwareConfig.h"
#include "SystemTypes.h"
#include "SafetyManager.h"

// ======================================================
// MAIN MIXER
// ======================================================

void updateDriveTarget()
{
  // ==================================================
  // SYSTEM SAFETY
  // ==================================================
  if (ibusCommLost ||
      getDriveSafety() == SafetyState::EMERGENCY ||
      systemState != SystemState::ACTIVE)
  {
    targetL = 0;
    targetR = 0;
    return;
  }

  // ==================================================
  // READ INPUT
  // ==================================================
  uint16_t rawThr = rcThrottle;
  uint16_t rawStr = rcSteer;

  // ==================================================
  // PLAUSIBILITY
  // ==================================================
  if (rawThr < 900 || rawThr > 2100 ||
      rawStr < 900 || rawStr > 2100)
  {
    targetL = 0;
    targetR = 0;
    return;
  }

  // ==================================================
  // RECOVERY CONFIRM
  // ==================================================
  if (requireIbusConfirm)
  {
    if (neutral(rawThr) && neutral(rawStr))
      requireIbusConfirm = false;
    else {
      targetL = 0;
      targetR = 0;
      return;
    }
  }

  // ==================================================
  // SPIKE + RATE FILTER
  // ==================================================
  static uint16_t lastThr = 1500;
  static uint16_t lastStr = 1500;

  constexpr int16_t SPIKE = 300;
  constexpr int16_t RATE  = 80;

  if (abs((int)rawThr - (int)lastThr) > SPIKE)
    rawThr = lastThr;

  if (abs((int)rawStr - (int)lastStr) > SPIKE)
    rawStr = lastStr;

  rawThr = constrain(rawThr, lastThr - RATE, lastThr + RATE);
  rawStr = constrain(rawStr, lastStr - RATE, lastStr + RATE);

  // clamp final
  rawThr = constrain(rawThr, 1000, 2000);
  rawStr = constrain(rawStr, 1000, 2000);

  lastThr = rawThr;
  lastStr = rawStr;

  // ==================================================
  // AXIS MAP (SOFT DEADZONE)
  // ==================================================
  auto mapAxis = [](int16_t v) -> int16_t {

    constexpr int16_t IN_MIN = 1000;
    constexpr int16_t IN_MAX = 2000;

    constexpr int16_t DB = 50;
    constexpr int16_t CENTER = 1500;

    constexpr int16_t OUT_MAX = PWM_TOP;

    int16_t diff = v - CENTER;

    if (abs(diff) <= DB)
      return 0;

    if (diff < 0)
    {
      long m = map(v, IN_MIN, CENTER - DB, -OUT_MAX, 0);
      return constrain((int16_t)m, -OUT_MAX, 0);
    }
    else
    {
      long m = map(v, CENTER + DB, IN_MAX, 0, OUT_MAX);
      return constrain((int16_t)m, 0, OUT_MAX);
    }
  };

  int16_t thr = mapAxis(rawThr);
  int16_t str = mapAxis(rawStr);

  // ==================================================
  // ZERO THROTTLE HARD STOP
  // ==================================================
  if (abs(thr) < 40)
  {
    targetL = 0;
    targetR = 0;
    return;
  }

  // ==================================================
  // LOW SPEED STEER SMOOTH
  // ==================================================
  int16_t absThr = abs(thr);

  if (absThr < 200)
  {
    float scale = absThr / 200.0f;
    str = str * scale;
  }

  // ==================================================
  // MIXING
  // ==================================================
  int32_t arcL = thr + str;
  int32_t arcR = thr - str;

  arcL = constrain(arcL, -PWM_TOP, PWM_TOP);
  arcR = constrain(arcR, -PWM_TOP, PWM_TOP);

  int32_t maxMag = max(abs(arcL), abs(arcR));

  if (maxMag > PWM_TOP)
  {
    arcL = (arcL * PWM_TOP) / maxMag;
    arcR = (arcR * PWM_TOP) / maxMag;
  }

  // ==================================================
  // OUTPUT
  // ==================================================
  targetL = constrain(arcL, -PWM_TOP, PWM_TOP);
  targetR = constrain(arcR, -PWM_TOP, PWM_TOP);
}

