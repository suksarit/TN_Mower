// ============================================================================
// InputMixer.cpp (IMPROVED - EXPO + SMOOTH + STABLE)
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "HardwareConfig.h"
#include "SystemTypes.h"
#include "SafetyManager.h"

// ======================================================
// EXPO FUNCTION
// ======================================================

static float applyExpo(float x, float expo)
{
  // expo: 0 = linear, 1 = soft
  return x * (1.0f - expo) + x * x * x * expo;
}

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
  // INPUT
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
    else
    {
      targetL = 0;
      targetR = 0;
      return;
    }
  }

  // ==================================================
  // FILTER (SPIKE + RATE)
  // ==================================================
  static uint16_t lastThr = 1500;
  static uint16_t lastStr = 1500;

  constexpr int16_t SPIKE = 300;
  constexpr int16_t RATE  = 60;  // 🔴 ลดลงให้นุ่มขึ้น

  if (abs((int)rawThr - (int)lastThr) > SPIKE)
    rawThr = lastThr;

  if (abs((int)rawStr - (int)lastStr) > SPIKE)
    rawStr = lastStr;

  rawThr = constrain(rawThr, lastThr - RATE, lastThr + RATE);
  rawStr = constrain(rawStr, lastStr - RATE, lastStr + RATE);

  rawThr = constrain(rawThr, 1000, 2000);
  rawStr = constrain(rawStr, 1000, 2000);

  lastThr = rawThr;
  lastStr = rawStr;

  // ==================================================
  // NORMALIZE (-1 → 1)
  // ==================================================
  float thr = (rawThr - 1500) / 500.0f;
  float str = (rawStr - 1500) / 500.0f;

  thr = constrain(thr, -1.0f, 1.0f);
  str = constrain(str, -1.0f, 1.0f);

  // ==================================================
  // EXPO (สำคัญมาก)
  // ==================================================
  thr = applyExpo(thr, 0.4f);  // throttle นุ่ม
  str = applyExpo(str, 0.5f);  // เลี้ยวนุ่มกว่า

  // ==================================================
  // LOW SPEED STEER REDUCTION
  // ==================================================
  float absThr = fabs(thr);

  if (absThr < 0.4f)
  {
    float scale = absThr / 0.4f;
    str *= scale;
  }

  // ==================================================
  // MIX (ARC DRIVE)
  // ==================================================
  float arcL = thr + str;
  float arcR = thr - str;

  // normalize
  float maxMag = max(fabs(arcL), fabs(arcR));
  if (maxMag > 1.0f)
  {
    arcL /= maxMag;
    arcR /= maxMag;
  }

  // ==================================================
  // SCALE TO PWM
  // ==================================================
  int16_t outL = arcL * PWM_TOP;
  int16_t outR = arcR * PWM_TOP;

  // ==================================================
  // SOFT ZERO (กัน jerk)
  // ==================================================
  if (fabs(thr) < 0.05f)
  {
    outL = 0;
    outR = 0;
  }

  // ==================================================
  // OUTPUT
  // ==================================================
  targetL = constrain(outL, -PWM_TOP, PWM_TOP);
  targetR = constrain(outR, -PWM_TOP, PWM_TOP);
}

