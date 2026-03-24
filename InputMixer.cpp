// ============================================================================
// InputMixer.cpp (FIXED - STABLE + NO JERK + REAL CONTROL)
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "HardwareConfig.h"
#include "SystemTypes.h"
#include "SafetyManager.h"

// ======================================================
// EXPO (ปรับใหม่ - ไม่อืดปลาย)
// ======================================================
static float applyExpo(float x, float expo)
{
  // blend linear + cubic แต่ไม่ให้ปลายหายแรง
  return x * (1.0f - expo) + (x * x * x) * expo;
}

// ======================================================
// SLEW LIMIT (float domain)
// ======================================================
static float applySlew(float in, float last, float rate)
{
  if (in > last + rate) return last + rate;
  if (in < last - rate) return last - rate;
  return in;
}

// ======================================================
// MAIN MIXER
// ======================================================
void updateDriveTarget()
{
  // ==================================================
  // SAFETY
  // ==================================================
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
  // RECOVERY
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
  // NORMALIZE
  // ==================================================
  float thr = (rawThr - 1500) / 500.0f;
  float str = (rawStr - 1500) / 500.0f;

  thr = constrain(thr, -1.0f, 1.0f);
  str = constrain(str, -1.0f, 1.0f);

  // ==================================================
  // SLEW (สำคัญ)
  // ==================================================
  static float lastThr = 0;
  static float lastStr = 0;

  constexpr float RATE = 0.06f;  // ~6% ต่อรอบ

  thr = applySlew(thr, lastThr, RATE);
  str = applySlew(str, lastStr, RATE);

  lastThr = thr;
  lastStr = str;

  // ==================================================
  // EXPO
  // ==================================================
  thr = applyExpo(thr, 0.35f);
  str = applyExpo(str, 0.45f);

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
  // MIX
  // ==================================================
  float arcL = thr + str;
  float arcR = thr - str;

  float maxMag = max(fabs(arcL), fabs(arcR));
  if (maxMag > 1.0f)
  {
    arcL /= maxMag;
    arcR /= maxMag;
  }

  // ==================================================
  // OUTPUT SLEW (กัน jerk จริง)
  // ==================================================
  static float lastOutL = 0;
  static float lastOutR = 0;

  constexpr float OUT_RATE = 0.08f;

  arcL = applySlew(arcL, lastOutL, OUT_RATE);
  arcR = applySlew(arcR, lastOutR, OUT_RATE);

  lastOutL = arcL;
  lastOutR = arcR;

  // ==================================================
  // SCALE
  // ==================================================
  int16_t outL = arcL * PWM_TOP;
  int16_t outR = arcR * PWM_TOP;

  // ==================================================
  // DEADZONE + MIN DRIVE
  // ==================================================
  constexpr int16_t DEAD = 40;
  constexpr int16_t MIN_DRIVE = 120;

  if (abs(outL) < DEAD) outL = 0;
  else if (abs(outL) < MIN_DRIVE)
    outL = (outL > 0) ? MIN_DRIVE : -MIN_DRIVE;

  if (abs(outR) < DEAD) outR = 0;
  else if (abs(outR) < MIN_DRIVE)
    outR = (outR > 0) ? MIN_DRIVE : -MIN_DRIVE;

  // ==================================================
  // FINAL
  // ==================================================
  targetL = constrain(outL, -PWM_TOP, PWM_TOP);
  targetR = constrain(outR, -PWM_TOP, PWM_TOP);
}

