// ============================================================================
// DriveProtection.cpp (FIXED + STABLE + CLEAN)
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "SystemTypes.h"
#include "HardwareConfig.h"
#include "SensorManager.h"
#include "FaultManager.h"
#include "AutoReverse.h"
#include "DriveProtection.h"

// ======================================================
// STALL ENERGY
// ======================================================
float stallEnergy = 0.0f;
uint32_t stallEnergyLast_ms = 0;

// ============================================================================
// WHEEL STUCK DETECTION
// ============================================================================

void detectWheelStuck(uint32_t now)
{
  if (autoReverseActive)
    return;

  static uint32_t lastStuckTrigger_ms = 0;
  constexpr uint32_t STUCK_COOLDOWN_MS = 1500;

  if (now - lastStuckTrigger_ms < STUCK_COOLDOWN_MS)
    return;

  static uint32_t stuckStart_ms = 0;

  constexpr int16_t MIN_PWM_FOR_STUCK = 300;
  constexpr int16_t TURNING_DIFF_MAX = 260;

  int16_t pwmMag = max(abs(curL), abs(curR));

  if (pwmMag < MIN_PWM_FOR_STUCK)
  {
    terrainDragAvg *= 0.90f;
    stuckStart_ms = 0;
    return;
  }

  if (abs(curL - curR) > TURNING_DIFF_MAX)
  {
    stuckStart_ms = 0;
    return;
  }

  float cL = curLeft();
  float cR = curRight();

  if (!isfinite(cL) || !isfinite(cR))
  {
    stuckStart_ms = 0;
    return;
  }

  float maxCur = max(cL, cR);

  if (maxCur < CUR_WARN_A)
  {
    terrainDragAvg *= 0.85f;
    stuckStart_ms = 0;
    return;
  }

  float curAvg = (cL + cR) * 0.5f;

  constexpr float DRAG_ALPHA = 0.06f;

  terrainDragAvg =
      terrainDragAvg * (1.0f - DRAG_ALPHA) + curAvg * DRAG_ALPHA;

  terrainDragAvg =
      constrain(terrainDragAvg, 0.0f, CUR_LIMP_A * 1.6f);

  bool heavyGrass =
      terrainDragAvg > CUR_WARN_A * 1.4f;

  bool torqueHigh =
      heavyGrass ? (maxCur > CUR_LIMP_A * 1.2f)
                 : (maxCur > CUR_LIMP_A);

  if (!torqueHigh)
  {
    stuckStart_ms = 0;
    return;
  }

  float percentDiff =
      fabs(cL - cR) / (maxCur + 0.001f);

  float imbalanceThreshold =
      (pwmMag < 400) ? 0.45f :
      (pwmMag < 700) ? 0.55f : 0.65f;

  if (heavyGrass)
    imbalanceThreshold *= 1.2f;

  if (percentDiff <= imbalanceThreshold)
  {
    stuckStart_ms = 0;
    return;
  }

  static int16_t prevL = 0;
  static int16_t prevR = 0;

  int16_t dL = abs(curL - prevL);
  int16_t dR = abs(curR - prevR);

  prevL = curL;
  prevR = curR;

  constexpr int16_t MIN_ACCEL = 60;

  if (!(dL < MIN_ACCEL && dR < MIN_ACCEL))
  {
    stuckStart_ms = 0;
    return;
  }

  uint32_t stuckTime =
      (pwmMag < 400) ? 800 :
      (pwmMag < 700) ? 600 : 420;

  if (stuckStart_ms == 0)
    stuckStart_ms = now;

  if (now - stuckStart_ms >= stuckTime)
  {
    stuckStart_ms = 0;
    lastStuckTrigger_ms = now;

    lastDriveEvent =
        (cL > cR) ? DriveEvent::STUCK_LEFT : DriveEvent::STUCK_RIGHT;

    startAutoReverse(now);
  }
}

// ============================================================================
// WHEEL LOCK (HARD FAULT)
// ============================================================================

void detectWheelLock()
{
  static uint8_t lockCnt = 0;
  static uint32_t lockStart_ms = 0;

  constexpr int16_t MIN_PWM_FOR_LOCK = 350;
  constexpr float CURRENT_BALANCE_RATIO = 0.25f;
  constexpr uint8_t LOCK_CONFIRM_CNT = 4;
  constexpr uint16_t LOCK_CONFIRM_MS = 400;

  uint32_t now = millis();

  int16_t pwmMag = max(abs(curL), abs(curR));

  if (pwmMag < MIN_PWM_FOR_LOCK)
  {
    lockCnt = 0;
    lockStart_ms = 0;
    return;
  }

  float cL = curLeft();
  float cR = curRight();

  float maxCur = max(cL, cR);
  float diffCur = fabs(cL - cR);

  if (maxCur <= 0.1f)
  {
    lockCnt = 0;
    lockStart_ms = 0;
    return;
  }

  float balanceRatio = diffCur / maxCur;

  if (balanceRatio > CURRENT_BALANCE_RATIO)
  {
    lockCnt = 0;
    lockStart_ms = 0;
    return;
  }

  float threshold =
      (pwmMag < 500) ? CUR_LIMP_A * 0.85f :
      (pwmMag < 800) ? CUR_LIMP_A :
                       CUR_LIMP_A * 1.15f;

  if (cL > threshold && cR > threshold)
  {
    int16_t errL = abs(targetL - curL);
    int16_t errR = abs(targetR - curR);

    if (errL < 50 && errR < 50)
    {
      if (lockCnt == 0)
        lockStart_ms = now;

      lockCnt++;

      if (lockCnt >= LOCK_CONFIRM_CNT &&
          (now - lockStart_ms) >= LOCK_CONFIRM_MS)
      {
        lastDriveEvent = DriveEvent::WHEEL_LOCK;
        latchFault(FaultCode::OVER_CURRENT);
      }
    }
    else
    {
      lockCnt = 0;
      lockStart_ms = 0;
    }
  }
  else
  {
    lockCnt = 0;
    lockStart_ms = 0;
  }
}

// ============================================================================
// STALL SCALE (ENERGY BASED)
// ============================================================================

float computeStallScale(uint32_t now, float curL_A, float curR_A)
{
  float curMax = max(curL_A, curR_A);

  if (stallEnergyLast_ms == 0)
  {
    stallEnergyLast_ms = now;
    return 1.0f;
  }

  uint32_t dt = now - stallEnergyLast_ms;
  stallEnergyLast_ms = now;

  if (dt > 200) dt = 200;

  float dtSec = dt * 0.001f;

  if (curMax > STALL_CURRENT_A)
  {
    float excess = curMax - STALL_CURRENT_A;
    stallEnergy += excess * dtSec * 0.02f;
  }
  else
  {
    stallEnergy *= STALL_DECAY;
  }

  stallEnergy = max(stallEnergy, 0.0f);

  if (stallEnergy < STALL_POWER_LIMIT)
    return 1.0f;

  float scale = STALL_POWER_LIMIT / stallEnergy;

  return constrain(scale, 0.25f, 1.0f);
}

// ============================================================================
// FINAL DRIVE LIMITS
// ============================================================================

void applyDriveLimits(float &finalTargetL,
                      float &finalTargetR,
                      float curA_L,
                      float curA_R)
{
  constexpr float TRACTION_SLIP_DIFF = 18.0f;

  if (fabs(curA_L - curA_R) > TRACTION_SLIP_DIFF)
  {
    if (curA_L < curA_R)
      finalTargetL *= 0.75f;
    else
      finalTargetR *= 0.75f;
  }

  if (curA_L > CUR_LIMP_A)
    finalTargetL *= 0.5f;

  if (curA_R > CUR_LIMP_A)
    finalTargetR *= 0.5f;

  finalTargetL += imbalanceCorrL;
  finalTargetR += imbalanceCorrR;

  finalTargetL = constrain(finalTargetL, -PWM_TOP, PWM_TOP);
  finalTargetR = constrain(finalTargetR, -PWM_TOP, PWM_TOP);
}

// ============================================================================
// MOTOR STALL DETECTION
// ============================================================================

bool detectMotorStall()
{
  static float prevCurL = 0;
  static float prevCurR = 0;
  static uint8_t stallCnt = 0;

  float cL = curLeft();
  float cR = curRight();

  if (abs(targetL) < 200 && abs(targetR) < 200)
  {
    prevCurL = cL;
    prevCurR = cR;
    return false;
  }

  if (cL < 5 && cR < 5)
    return false;

  float dCurL = cL - prevCurL;
  float dCurR = cR - prevCurR;

  constexpr float STALL_CURRENT_STEP = 18.0f;
  constexpr float STALL_CURRENT_MIN  = 40.0f;
  constexpr uint8_t STALL_CONFIRM_CNT = 2;

  if (cL > STALL_CURRENT_MIN || cR > STALL_CURRENT_MIN)
  {
    bool stallDetected = false;

    if (cL > STALL_CURRENT_MIN && dCurL > STALL_CURRENT_STEP)
      stallDetected = true;

    if (cR > STALL_CURRENT_MIN && dCurR > STALL_CURRENT_STEP)
      stallDetected = true;

    if (stallDetected)
    {
      if (++stallCnt >= STALL_CONFIRM_CNT)
      {
        stallCnt = 0;
        prevCurL = cL;
        prevCurR = cR;
        return true;
      }
    }
    else
    {
      stallCnt = 0;
    }
  }
  else
  {
    stallCnt = 0;
  }

  prevCurL = cL;
  prevCurR = cR;

  return false;
}

// ============================================================================
// SIDE IMBALANCE CORRECTION
// ============================================================================

void detectSideImbalanceAndSteer()
{
  // reset ทุก loop (transient correction)
  imbalanceCorrL = 0;
  imbalanceCorrR = 0;

  // --------------------------------------------------
  // STOP GUARD
  // --------------------------------------------------
  if (abs(targetL) < 50 && abs(targetR) < 50)
    return;

  // --------------------------------------------------
  // MUST BE MOVING
  // --------------------------------------------------
  if (abs(curL) < 200 && abs(curR) < 200)
    return;

  // --------------------------------------------------
  // OPERATOR TURNING → SKIP
  // --------------------------------------------------
  if (abs(targetL - targetR) > 200)
    return;

  // --------------------------------------------------
  // CURRENT
  // --------------------------------------------------
  float cL = curLeft();
  float cR = curRight();

  // --------------------------------------------------
  // SENSOR VALID
  // --------------------------------------------------
  if (cL < 0 || cR < 0 || cL > 200 || cR > 200)
    return;

  // --------------------------------------------------
  // HIGH LOAD → let WheelStuck handle
  // --------------------------------------------------
  float maxCur = max(cL, cR);

  if (maxCur > CUR_WARN_A)
    return;

  // --------------------------------------------------
  // HIGH SPEED → SKIP
  // --------------------------------------------------
  int16_t pwmMag = max(abs(curL), abs(curR));

  if (pwmMag > 600)
    return;

  constexpr float CUR_IMBALANCE_A = 25.0f;
  constexpr int16_t STEER_COMP = 140;

  // --------------------------------------------------
  // DEBOUNCE
  // --------------------------------------------------
  static uint8_t imbCnt = 0;

  if (abs(cL - cR) > CUR_IMBALANCE_A)
  {
    if (++imbCnt < 2)
      return;
  }
  else
  {
    imbCnt = 0;
    return;
  }

  // --------------------------------------------------
  // APPLY CORRECTION
  // --------------------------------------------------
  lastDriveEvent = DriveEvent::IMBALANCE;

  if (cL > cR)
  {
    imbalanceCorrL = -STEER_COMP;
    imbalanceCorrR = +STEER_COMP;
  }
  else
  {
    imbalanceCorrL = +STEER_COMP;
    imbalanceCorrR = -STEER_COMP;
  }
}
