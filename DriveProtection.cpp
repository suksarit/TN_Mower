// ============================================================================
// DriveProtection.cpp 
// ============================================================================

#include <Arduino.h>
#include <math.h>

#include "GlobalState.h"
#include "SystemTypes.h"
#include "HardwareConfig.h"
#include "SensorManager.h"
#include "FaultManager.h"
#include "AutoReverse.h"
#include "DriveProtection.h"

// ======================================================
// INTERNAL STATE
// ======================================================

uint32_t stallStartTime = 0;
static uint32_t stuckStartTime = 0;
static uint32_t lockStart_ms = 0;

static float terrainLoad = 0.0f;
static float slipRatio = 0.0f;

static uint32_t lastTime_ms = 0;
static float stallEnergy = 0.0f;

// 🔴 เพิ่ม: สำหรับกัน jerk imbalance
static float lastCorr = 0.0f;

// ============================================================================
// TERRAIN UPDATE
// ============================================================================

static void updateTerrain(float curL, float curR)
{
  float avg = (curL + curR) * 0.5f;

  terrainLoad = terrainLoad * 0.95f + avg * 0.05f;
  terrainLoad = constrain(terrainLoad, 0.0f, CUR_LIMP_A * 1.5f);
}

// ============================================================================
// SLIP DETECTION
// ============================================================================

static float computeSlip(float curL, float curR)
{
  float maxCur = max(curL, curR);
  if (maxCur < 5.0f) return 0.0f;

  if (abs(targetL - targetR) > 200)
    return 0.0f;

  float diff = fabs(curL - curR) / (maxCur + 0.001f);

  slipRatio = slipRatio * 0.8f + diff * 0.2f;

  return slipRatio;
}

// ============================================================================
// STALL SCALE
// ============================================================================

float computeStallScale(uint32_t now, float curL_A, float curR_A)
{
  float curMax = max(curL_A, curR_A);

  if (lastTime_ms == 0)
  {
    lastTime_ms = now;
    return 1.0f;
  }

  uint32_t dt = now - lastTime_ms;
  lastTime_ms = now;

  if (dt > 200) dt = 200;

  float dtSec = dt * 0.001f;

  updateTerrain(curL_A, curR_A);

  float terrainFactor = constrain(terrainLoad / CUR_WARN_A, 0.8f, 1.5f);
  float dynamicThreshold = STALL_CURRENT_A * terrainFactor;

  if (curMax > dynamicThreshold)
  {
    float excess = curMax - dynamicThreshold;
    stallEnergy += excess * dtSec * 0.03f;
  }
  else
  {
    stallEnergy *= 0.90f;
  }

  if (curMax < 8.0f)
    stallEnergy = 0.0f;

  if (stallEnergy < STALL_POWER_LIMIT)
    return 1.0f;

  float scale = STALL_POWER_LIMIT / stallEnergy;

  setDriveEvent(DriveEvent::WHEEL_LOCK);

  return constrain(scale, 0.3f, 1.0f);
}

// ============================================================================
// DRIVE LIMITS
// ============================================================================

void applyDriveLimits(float &tL,
                     float &tR,
                     float curL_A,
                     float curR_A)
{
  float slip = computeSlip(curL_A, curR_A);

  if (slip > 0.25f)
  {
    float reduce = constrain((slip - 0.25f) * 1.5f, 0.0f, 0.4f);

    if (curL_A < curR_A)
      tL *= (1.0f - reduce);
    else
      tR *= (1.0f - reduce);

    setDriveEvent(DriveEvent::TRACTION_LOSS);
  }

  float terrainFactor = constrain(terrainLoad / CUR_WARN_A, 0.8f, 1.4f);

  float limitL = CUR_LIMP_A * terrainFactor;
  float limitR = CUR_LIMP_A * terrainFactor;

  if (curL_A > limitL)
    tL *= 0.5f;

  if (curR_A > limitR)
    tR *= 0.5f;

  tL = constrain(tL, -PWM_TOP, PWM_TOP);
  tR = constrain(tR, -PWM_TOP, PWM_TOP);
}

// ============================================================================
// IMBALANCE (🔥 ปรับใหม่ให้เนียน + ไม่กระชาก)
// ============================================================================

void detectSideImbalanceAndSteer(float &tL,
                                 float &tR,
                                 float curL_A,
                                 float curR_A)
{
  // --------------------------------------------------
  // LOW POWER → SKIP
  // --------------------------------------------------
  if (abs(tL) < 50 && abs(tR) < 50)
    return;

  // --------------------------------------------------
  // STRONG TURN → SKIP (กัน over-correct)
  // --------------------------------------------------
  if (abs(tL - tR) > 300)
    return;

  float diff = curL_A - curR_A;

  // --------------------------------------------------
  // DEAD BAND
  // --------------------------------------------------
  if (fabs(diff) < 15.0f)
    return;

  // --------------------------------------------------
  // GAIN (adaptive)
  // --------------------------------------------------
  float terrainFactor = constrain(terrainLoad / CUR_WARN_A, 0.5f, 1.5f);

  float speedFactor =
    constrain((fabs(tL) + fabs(tR)) * 0.5f / PWM_TOP, 0.3f, 1.0f);

  float gain = 0.4f * terrainFactor * speedFactor;

  // --------------------------------------------------
  // RAW CORRECTION
  // --------------------------------------------------
  float corr = diff * gain;

  // --------------------------------------------------
  // 🔴 RATE LIMIT (กัน jerk)
  // --------------------------------------------------
  float d = corr - lastCorr;
  const float MAX_STEP = 20.0f;

  if (d > MAX_STEP) d = MAX_STEP;
  if (d < -MAX_STEP) d = -MAX_STEP;

  corr = lastCorr + d;
  lastCorr = corr;

  corr = constrain(corr, -100.0f, 100.0f);

  // --------------------------------------------------
  // APPLY
  // --------------------------------------------------
  float newL = tL - corr;
  float newR = tR + corr;

  // --------------------------------------------------
  // SIGN PROTECTION
  // --------------------------------------------------
  if ((tL > 0 && newL < 0) || (tL < 0 && newL > 0))
    newL = 0;

  if ((tR > 0 && newR < 0) || (tR < 0 && newR > 0))
    newR = 0;

  tL = newL;
  tR = newR;

  setDriveEvent(DriveEvent::IMBALANCE);
}

// ============================================================================
// STUCK DETECT
// ============================================================================

void detectWheelStuck(uint32_t now)
{
  if (autoReverseActive)
    return;

  if (slipRatio < 0.35f || terrainLoad < CUR_WARN_A)
  {
    stuckStartTime = 0;
    return;
  }

  if (stuckStartTime == 0)
    stuckStartTime = now;

  if (now - stuckStartTime > 700)
  {
    stuckStartTime = 0;

    setDriveEvent(DriveEvent::STUCK_LEFT);

    if (abs(curL) < 120 && abs(curR) < 120)
      startAutoReverse(now);
  }
}

// ============================================================================
// LOCK DETECT
// ============================================================================

void detectWheelLock()
{
  float cL = curLeft();
  float cR = curRight();

  if (cL > CUR_LIMP_A && cR > CUR_LIMP_A)
  {
    if (lockStart_ms == 0)
      lockStart_ms = millis();

    if (millis() - lockStart_ms > 120)
    {
      setDriveEvent(DriveEvent::WHEEL_LOCK);
      latchFault(FaultCode::OVER_CURRENT);
    }
  }
  else
  {
    lockStart_ms = 0;
  }
}

// ============================================================================
// MOTOR STALL
// ============================================================================

bool detectMotorStall(uint32_t now, float curL_A, float curR_A)
{
  float curMax = max(curL_A, curR_A);

  if (curMax > CUR_LIMP_A * 1.2f)
  {
    setDriveEvent(DriveEvent::WHEEL_LOCK);
    return true;
  }

  return false;
}

