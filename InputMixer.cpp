// ============================================================================
// InputMixer.cpp (TORQUE MODE - FINAL)
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "HardwareConfig.h"
#include "SystemTypes.h"
#include "SafetyManager.h"

// ======================================================
static float applyExpo(float x, float expo)
{
  return x * (1.0f - expo) + (x * x * x) * expo;
}

// ======================================================
static float applySlew(float in, float last, float rate)
{
  if (in > last + rate) return last + rate;
  if (in < last - rate) return last - rate;
  return in;
}

// ======================================================
void updateDriveTarget()
{
  if (ibusCommLost ||
      getDriveSafety() == SafetyState::EMERGENCY ||
      systemState != SystemState::ACTIVE)
  {
    targetL = 0;
    targetR = 0;
    return;
  }

  uint16_t rawThr = rcThrottle;
  uint16_t rawStr = rcSteer;

  if (rawThr < 900 || rawThr > 2100 ||
      rawStr < 900 || rawStr > 2100)
  {
    targetL = 0;
    targetR = 0;
    return;
  }

  if (requireIbusConfirm)
  {
    if (neutral(rawThr) && neutral(rawStr))
      requireIbusConfirm = false;
    else
    {
      targetL = 0;
      targetR = 0;
      return;
    }
  }

  float thr = (rawThr - 1500) / 500.0f;
  float str = (rawStr - 1500) / 500.0f;

  thr = constrain(thr, -1.0f, 1.0f);
  str = constrain(str, -1.0f, 1.0f);

  static float lastThr = 0;
  static float lastStr = 0;

  thr = applySlew(thr, lastThr, 0.06f);
  str = applySlew(str, lastStr, 0.06f);

  lastThr = thr;
  lastStr = str;

  thr = applyExpo(thr, 0.35f);
  str = applyExpo(str, 0.45f);

  float absThr = fabs(thr);
  if (absThr < 0.4f)
    str *= absThr / 0.4f;

  float arcL = thr + str;
  float arcR = thr - str;

  float maxMag = max(fabs(arcL), fabs(arcR));
  if (maxMag > 1.0f)
  {
    arcL /= maxMag;
    arcR /= maxMag;
  }

  // 🔴 เปลี่ยนเป็น TORQUE (current target)
  constexpr float MAX_CURRENT = 30.0f;

  float torqueL = arcL * MAX_CURRENT;
  float torqueR = arcR * MAX_CURRENT;

  // deadzone
  if (fabs(torqueL) < 0.5f) torqueL = 0;
  if (fabs(torqueR) < 0.5f) torqueR = 0;

  targetL = torqueL;
  targetR = torqueR;
}

