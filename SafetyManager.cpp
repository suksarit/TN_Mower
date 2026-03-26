// ============================================================================
// SafetyManager.cpp 
// ============================================================================

#include <Arduino.h>
#include "SafetyManager.h"
#include "SystemTypes.h"
#include "GlobalState.h"

// 🔴 รับ killRequest จากระบบหลัก
extern KillType killRequest;

void driveSafe(); 

// ============================================================================
// INTERNAL STATE
// ============================================================================
static SafetyState driveSafetyInternal = SafetyState::SAFE;

// 🔴 EMERGENCY LOCK (sticky)
static bool emergencyLatched = false;

// 🔴 กันยิงซ้ำ
static bool killActive = false;

// ============================================================================
// HYSTERESIS COUNTERS
// ============================================================================
static uint8_t limpConfirmCnt = 0;
static uint8_t warnConfirmCnt = 0;

static constexpr uint8_t LIMP_CONFIRM_CNT = 3;
static constexpr uint8_t WARN_CONFIRM_CNT = 2;

// 🔴 HOLD TIME
static uint32_t limpHoldStart_ms = 0;
static constexpr uint32_t LIMP_HOLD_TIME_MS = 1500;

// ============================================================================
// STABILITY TRACKER
// ============================================================================
enum class SafetyStabilityState : uint8_t {
  SAFE_TRANSIENT = 0,
  SAFE_STABLE
};

static SafetyStabilityState safetyStability =
  SafetyStabilityState::SAFE_TRANSIENT;

static uint32_t safeStableStart_ms = 0;
static constexpr uint32_t SAFE_STABLE_TIME_MS = 2000;

// ============================================================================
// 🔴 APPLY KILL REQUEST (สำคัญที่สุด)
// ============================================================================
void applyKillRequest(uint32_t now)
{
  if (killRequest == KillType::NONE)
  {
    killActive = false;
    return;
  }

  // ==================================================
  // 🔴 SOFT STOP
  // ==================================================
  if (killRequest == KillType::SOFT)
  {
    if (!killActive)
    {
      targetL = 0;
      targetR = 0;

      driveState = DriveState::SOFT_STOP;
      killActive = true;
    }
  }

  // ==================================================
  // 🔴 HARD STOP
  // ==================================================
  else if (killRequest == KillType::HARD)
  {
    // ตัดจริง
    driveSafe();

    // 🔴 reset target กันเด้ง
    targetL = 0;
    targetR = 0;

    driveState = DriveState::LOCKED;

    emergencyLatched = true;
    killActive = true;
  }

  // 🔴 เคลียร์ request
  killRequest = KillType::NONE;
}

// ============================================================================
// RAW SAFETY
// ============================================================================
SafetyState evaluateSafetyRaw(
  const SafetyInput& in,
  const SafetyThresholds& th)
{
  if (in.faultLatched)
    return SafetyState::EMERGENCY;

  float curMax = max(
    max(fabs(in.curA[0]), fabs(in.curA[1])),
    max(fabs(in.curA[2]), fabs(in.curA[3])));

  if (curMax > th.CUR_LIMP_A ||
      in.tempDriverL > th.TEMP_LIMP_C ||
      in.tempDriverR > th.TEMP_LIMP_C)
  {
    return SafetyState::LIMP;
  }

  if (in.driveEvent == DriveEvent::STUCK_LEFT ||
      in.driveEvent == DriveEvent::STUCK_RIGHT)
  {
    return SafetyState::LIMP;
  }

  if (in.driveEvent == DriveEvent::IMBALANCE)
    return SafetyState::WARN;

  if (curMax > th.CUR_WARN_A ||
      in.tempDriverL > th.TEMP_WARN_C ||
      in.tempDriverR > th.TEMP_WARN_C)
  {
    return SafetyState::WARN;
  }

  return SafetyState::SAFE;
}

// ============================================================================
// STABILITY + HYSTERESIS
// ============================================================================
void updateSafetyStability(
  SafetyState raw,
  uint32_t now,
  uint8_t& autoReverseCount,
  bool& autoReverseActive,
  DriveEvent& lastDriveEvent)
{
  if (raw == SafetyState::EMERGENCY)
  {
    emergencyLatched = true;
  }

  if (emergencyLatched)
  {
    driveSafetyInternal = SafetyState::EMERGENCY;

    limpConfirmCnt = 0;
    warnConfirmCnt = 0;

    safetyStability = SafetyStabilityState::SAFE_TRANSIENT;
    safeStableStart_ms = 0;

    return;
  }

  SafetyState filtered = driveSafetyInternal;

  switch (raw)
  {
    case SafetyState::LIMP:
      warnConfirmCnt = 0;

      if (++limpConfirmCnt >= LIMP_CONFIRM_CNT)
      {
        filtered = SafetyState::LIMP;
        limpConfirmCnt = LIMP_CONFIRM_CNT;
        limpHoldStart_ms = now;
      }
      break;

    case SafetyState::WARN:
      limpConfirmCnt = 0;

      if (++warnConfirmCnt >= WARN_CONFIRM_CNT)
      {
        filtered = SafetyState::WARN;
        warnConfirmCnt = WARN_CONFIRM_CNT;
      }
      break;

    case SafetyState::SAFE:
      limpConfirmCnt = 0;
      warnConfirmCnt = 0;
      break;

    default:
      break;
  }

  if (driveSafetyInternal == SafetyState::LIMP)
  {
    if (now - limpHoldStart_ms < LIMP_HOLD_TIME_MS)
    {
      filtered = SafetyState::LIMP;
    }
  }

  driveSafetyInternal = filtered;

  if (raw != SafetyState::SAFE)
  {
    safetyStability = SafetyStabilityState::SAFE_TRANSIENT;
    safeStableStart_ms = 0;
    return;
  }

  if (safetyStability == SafetyStabilityState::SAFE_TRANSIENT)
  {
    if (safeStableStart_ms == 0)
    {
      safeStableStart_ms = now;
      return;
    }

    if (now - safeStableStart_ms >= SAFE_STABLE_TIME_MS)
    {
      safetyStability = SafetyStabilityState::SAFE_STABLE;

      driveSafetyInternal = SafetyState::SAFE;

      autoReverseCount = 0;
      autoReverseActive = false;
      lastDriveEvent = DriveEvent::NONE;
    }
  }
}

// ============================================================================
// ACCESSORS
// ============================================================================
SafetyState getDriveSafety()
{
  return driveSafetyInternal;
}

void forceSafetyState(SafetyState s)
{
  driveSafetyInternal = s;

  if (s == SafetyState::EMERGENCY)
  {
    emergencyLatched = true;
  }

  limpConfirmCnt = 0;
  warnConfirmCnt = 0;

  safetyStability = SafetyStabilityState::SAFE_TRANSIENT;
  safeStableStart_ms = 0;
}

// ============================================================================
// CLEAR EMERGENCY LATCH
// ============================================================================
void clearSafetyLatch()
{
  emergencyLatched = false;
  killActive = false;
}

