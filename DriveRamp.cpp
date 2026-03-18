// ============================================================================
// DriveRamp.cpp (TUNED - SMOOTH + STABLE + REAL LOAD)
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "HardwareConfig.h"
#include "SystemTypes.h"
#include "SensorManager.h"
#include "DriveRamp.h"

// ======================================================
// LOCAL STATE (ANTI-STUCK)
// ======================================================
static uint32_t stuckStart_ms = 0;
static uint8_t stuckPhase = 0;

static float tractionScale = 1.0f;

// ============================================================================
// MAIN RAMP FUNCTION
// ============================================================================

void updateDriveRamp(float &finalTargetL,
                     float &finalTargetR)
{
  finalTargetL = constrain(finalTargetL, -PWM_TOP, PWM_TOP);
  finalTargetR = constrain(finalTargetR, -PWM_TOP, PWM_TOP);

  uint32_t now = millis();

  // ==================================================
  // ERROR
  // ==================================================
  float errL = finalTargetL - curL;
  float errR = finalTargetR - curR;

  float errMag = max(abs(errL), abs(errR));

  // ==================================================
  // BASE ACCEL (S-CURVE)
  // ==================================================
  float accel;

  if (driveState == DriveState::LIMP)
    accel = 2.0f;
  else if (errMag > 700)
    accel = 2.5f;
  else if (errMag > 300)
    accel = 3.5f;
  else if (errMag > 100)
    accel = 5.0f;
  else
    accel = 7.0f;

  // ==================================================
  // LOAD
  // ==================================================
  float curA_L = getMotorCurrentL();
  float curA_R = getMotorCurrentR();

  float load = max(curA_L, curA_R);

  // ==================================================
  // ✅ FIX 1: TRACTION (adaptive + ไม่ดิ่ง)
  // ==================================================
  float slip = abs(curA_L - curA_R);

  float slipThreshold =
      (load < 30.0f) ? 8.0f :
      (load < 50.0f) ? 12.0f : 16.0f;

  if (slip > slipThreshold)
  {
    float reduce = constrain((slip - slipThreshold) * 0.02f, 0.0f, 0.3f);
    tractionScale = 1.0f - reduce;

    lastDriveEvent = DriveEvent::TRACTION_LOSS;
  }
  else
  {
    tractionScale += 0.01f;
  }

  tractionScale = constrain(tractionScale, 0.4f, 1.0f);

  accel *= tractionScale;

  // ==================================================
  // LOAD ADAPTIVE
  // ==================================================
  if (load > 40.0f)
  {
    float scale = 1.0f - (load - 40.0f) * 0.015f;
    scale = constrain(scale, 0.4f, 1.0f);
    accel *= scale;
  }

  // ==================================================
  // ✅ FIX 2: ANTI-STUCK (นิ่งขึ้น)
  // ==================================================
  bool stuck = (load > 55.0f && errMag > 250);

  if (stuck)
  {
    if (stuckStart_ms == 0)
      stuckStart_ms = now;

    uint32_t t = now - stuckStart_ms;

    if (t < 600) stuckPhase = 1;
    else if (t < 1400) stuckPhase = 2;
    else stuckPhase = 3;
  }
  else
  {
    stuckStart_ms = 0;
    stuckPhase = 0;
  }

  // ==================================================
  // APPLY STUCK ACTION (ลดแรงกระชาก)
  // ==================================================
  switch (stuckPhase)
  {
    case 1:
      accel *= 1.2f; // เดิม 1.4 → แรงไป
      break;

    case 2:
    {
      int toggle = (now / 250) % 2;

      if (toggle == 0)
      {
        finalTargetL *= 0.8f;
        finalTargetR *= 1.1f;
      }
      else
      {
        finalTargetL *= 1.1f;
        finalTargetR *= 0.8f;
      }
      break;
    }

    case 3:
      lastDriveEvent = DriveEvent::WHEEL_STUCK;
      stuckStart_ms = 0;
      stuckPhase = 0;
      break;
  }

  // ==================================================
  // SAFE REVERSE
  // ==================================================
  constexpr int16_t REVERSE_SAFE_PWM = 120;

  bool reverseL =
    (curL > 0 && finalTargetL < 0) ||
    (curL < 0 && finalTargetL > 0);

  bool reverseR =
    (curR > 0 && finalTargetR < 0) ||
    (curR < 0 && finalTargetR > 0);

  if (reverseL)
  {
    if (abs(curL) > REVERSE_SAFE_PWM || now < revBlockUntilL)
      finalTargetL = 0;
    else
    {
      revBlockUntilL = now + REVERSE_DEADTIME_MS;
      finalTargetL = 0;
    }
  }

  if (reverseR)
  {
    if (abs(curR) > REVERSE_SAFE_PWM || now < revBlockUntilR)
      finalTargetR = 0;
    else
    {
      revBlockUntilR = now + REVERSE_DEADTIME_MS;
      finalTargetR = 0;
    }
  }

  // ==================================================
  // ✅ FIX 3: S-CURVE (ลด overshoot)
  // ==================================================
  float stepL = constrain(errL, -accel, accel);
  float stepR = constrain(errR, -accel, accel);

  stepL = stepL * 0.8f + errL * 0.2f;
  stepR = stepR * 0.8f + errR * 0.2f;

  curL += stepL;
  curR += stepR;

  // ==================================================
  // LOW SPEED SMOOTH
  // ==================================================
  if (abs(curL) < 120)
    curL *= 0.92f;

  if (abs(curR) < 120)
    curR *= 0.92f;

  // ==================================================
  // TRACTION BALANCE
  // ==================================================
  constexpr int16_t MAX_DIFF = 300;

  int16_t diff = curL - curR;

  if (diff > MAX_DIFF)
    curL = curR + MAX_DIFF;

  if (diff < -MAX_DIFF)
    curR = curL + MAX_DIFF;

  // ==================================================
  // FINAL CLAMP
  // ==================================================
  curL = constrain(curL, -PWM_TOP, PWM_TOP);
  curR = constrain(curR, -PWM_TOP, PWM_TOP);
}

// ============================================================================
// RESET DRIVE RAMP
// ============================================================================

void resetDriveRamp()
{
  curL = 0;
  curR = 0;

  revBlockUntilL = 0;
  revBlockUntilR = 0;

  tractionScale = 1.0f;
  stuckStart_ms = 0;
  stuckPhase = 0;
}