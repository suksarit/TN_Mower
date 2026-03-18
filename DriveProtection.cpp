// ============================================================================
// DriveProtection.cpp (ADAPTIVE TERRAIN + SLIP CONTROL - PRODUCTION)
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
uint32_t stuckStartTime = 0;

// ======================================================
// TERRAIN MODEL
// ======================================================

static float terrainLoad = 0.0f;  // ค่าแรงต้านสะสม
static float slipRatio = 0.0f;

static uint32_t lastTime_ms = 0;
static float stallEnergy = 0.0f;

// ============================================================================
// TERRAIN UPDATE
// ============================================================================

static void updateTerrain(float curL, float curR) {
  float avg = (curL + curR) * 0.5f;

  // LPF
  terrainLoad = terrainLoad * 0.95f + avg * 0.05f;

  // clamp
  terrainLoad = constrain(terrainLoad, 0.0f, CUR_LIMP_A * 1.5f);
}

// ============================================================================
// SLIP DETECTION
// ============================================================================

static float computeSlip(float curL, float curR) {
  float maxCur = max(curL, curR);
  if (maxCur < 5.0f) return 0.0f;

  float diff = fabs(curL - curR) / (maxCur + 0.001f);

  // LPF
  slipRatio = slipRatio * 0.8f + diff * 0.2f;

  return slipRatio;
}

// ============================================================================
// STALL SCALE (ADAPTIVE)
// ============================================================================

float computeStallScale(uint32_t now, float curL_A, float curR_A) {
  float curMax = max(curL_A, curR_A);

  if (lastTime_ms == 0) {
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

  if (curMax > dynamicThreshold) {
    float excess = curMax - dynamicThreshold;
    stallEnergy += excess * dtSec * 0.03f;
  } else {
    stallEnergy *= 0.90f;
  }

  if (stallEnergy < STALL_POWER_LIMIT)
    return 1.0f;

  float scale = STALL_POWER_LIMIT / stallEnergy;

  lastDriveEvent = DriveEvent::WHEEL_LOCK;

  return constrain(scale, 0.3f, 1.0f);
}

// ============================================================================
// SLIP + LIMIT
// ============================================================================

void applyDriveLimits(float &tL,
                      float &tR,
                      float curL_A,
                      float curR_A) {
  float slip = computeSlip(curL_A, curR_A);

  // ==================================================
  // SLIP CONTROL
  // ==================================================
  if (slip > 0.25f) {
    float reduce = constrain((slip - 0.25f) * 1.5f, 0.0f, 0.4f);

    if (curL_A < curR_A)
      tL *= (1.0f - reduce);
    else
      tR *= (1.0f - reduce);

    lastDriveEvent = DriveEvent::TRACTION_LOSS;
  }

  // ==================================================
  // CURRENT LIMIT (ADAPTIVE)
  // ==================================================
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
// IMBALANCE CORRECTION (SMART)
// ============================================================================

void detectSideImbalanceAndSteer(float &tL,
                                 float &tR,
                                 float curL_A,
                                 float curR_A) {
  if (abs(tL) < 50 && abs(tR) < 50)
    return;

  float diff = curL_A - curR_A;

  if (fabs(diff) < 15.0f)
    return;

  float terrainFactor = constrain(terrainLoad / CUR_WARN_A, 0.5f, 1.5f);

  float gain = 0.4f * terrainFactor;

  float corr = constrain(diff * gain, -100.0f, 100.0f);

  tL -= corr;
  tR += corr;

  lastDriveEvent = DriveEvent::IMBALANCE;
}

// ============================================================================
// STUCK DETECT (ADAPTIVE)
// ============================================================================

void detectWheelStuck(uint32_t now) {
  if (autoReverseActive)
    return;

  float slip = slipRatio;

  if (slip < 0.35f) {
    stuckStartTime = 0;
    return;
  }

  if (terrainLoad < CUR_WARN_A) {
    stuckStartTime = 0;
    return;
  }

  if (stuckStartTime == 0)
    stuckStartTime = now;

  if (now - stuckStartTime > 700) {
    stuckStartTime = 0;
    lastDriveEvent = DriveEvent::STUCK_LEFT;
    // หรือ
    lastDriveEvent = DriveEvent::STUCK_RIGHT;

    startAutoReverse(now);
  }
}

// ============================================================================
// LOCK DETECT
// ============================================================================

void detectWheelLock() {
  float cL = curLeft();
  float cR = curRight();

  if (cL > CUR_LIMP_A && cR > CUR_LIMP_A) {
    lastDriveEvent = DriveEvent::WHEEL_LOCK;
    latchFault(FaultCode::OVER_CURRENT);
  }
}

// ============================================================================
// MOTOR STALL
// ============================================================================

bool detectMotorStall(uint32_t now, float curL_A, float curR_A) {
  float curMax = max(curL_A, curR_A);

  if (curMax > CUR_LIMP_A * 1.2f) {
   lastDriveEvent = DriveEvent::WHEEL_LOCK;
    return true;
  }

  return false;
}