// ============================================================================
// SafetyManager.cpp (HARDENED - NO FLAP + EMERGENCY LOCK)
// ============================================================================

#include <Arduino.h>

#include "SafetyManager.h"

// ============================================================================
// INTERNAL STATE
// ============================================================================
static SafetyState driveSafetyInternal = SafetyState::SAFE;

// ============================================================================
// HYSTERESIS COUNTERS
// ============================================================================
static uint8_t limpConfirmCnt = 0;
static uint8_t warnConfirmCnt = 0;

static constexpr uint8_t LIMP_CONFIRM_CNT = 3;
static constexpr uint8_t WARN_CONFIRM_CNT = 2;

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
// RAW SAFETY
// ============================================================================
SafetyState evaluateSafetyRaw(
  const SafetyInput& in,
  const SafetyThresholds& th)
{
  if (in.faultLatched)
    return SafetyState::EMERGENCY;

  float curMax = max(
    max(in.curA[0], in.curA[1]),
    max(in.curA[2], in.curA[3]));

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
  // ==================================================
  // 🔴 EMERGENCY LOCK (override everything)
  // ==================================================
  if (raw == SafetyState::EMERGENCY)
  {
    driveSafetyInternal = SafetyState::EMERGENCY;

    limpConfirmCnt = 0;
    warnConfirmCnt = 0;

    safetyStability = SafetyStabilityState::SAFE_TRANSIENT;
    safeStableStart_ms = 0;

    return;
  }

  SafetyState filtered = driveSafetyInternal;

  // ==================================================
  // HYSTERESIS (NO DOWN-FLAP)
  // ==================================================
  switch (raw)
  {
    case SafetyState::LIMP:
      warnConfirmCnt = 0;

      if (++limpConfirmCnt >= LIMP_CONFIRM_CNT)
      {
        filtered = SafetyState::LIMP;
        limpConfirmCnt = LIMP_CONFIRM_CNT;
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
      // 🔴 ไม่ลด state ทันที → ต้องรอ stability
      limpConfirmCnt = 0;
      warnConfirmCnt = 0;
      break;

    default:
      break;
  }

  // ==================================================
  // APPLY STATE
  // ==================================================
  driveSafetyInternal = filtered;

  // ==================================================
  // SAFE STABILITY (STRICT)
  // ==================================================
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

    // 🔴 ต้องนิ่งจริง (ไม่มี event)
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

  limpConfirmCnt = 0;
  warnConfirmCnt = 0;

  safetyStability = SafetyStabilityState::SAFE_TRANSIENT;
  safeStableStart_ms = 0;
}

