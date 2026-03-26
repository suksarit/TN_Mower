// ========================================================================================
// SafetyManager.h   
// ========================================================================================

#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <Arduino.h>        // 🔴 ต้องมี (uint32_t)
#include "SystemTypes.h"    // SafetyState, DriveEvent

// ============================================================================
// ACCESSORS (GLOBAL SAFETY STATE)
// ============================================================================
SafetyState getDriveSafety();
void forceSafetyState(SafetyState s);
void clearSafetyLatch();

// ============================================================================
// 🔴 EXECUTION LAYER (สำคัญมาก)
// ใช้ apply killRequest → ควบคุม motor จริง
// ============================================================================
void applyKillRequest(uint32_t now);

// ============================================================================
// SAFETY INPUT SNAPSHOT (PURE DATA)
// ============================================================================
struct SafetyInput
{
  float curA[4];        // กระแสแต่ละช่อง
  int16_t tempDriverL;  // อุณหภูมิไดรเวอร์ซ้าย
  int16_t tempDriverR;  // อุณหภูมิไดรเวอร์ขวา
  bool faultLatched;    // fault ระดับระบบ
  DriveEvent driveEvent;
};

// ============================================================================
// SAFETY THRESHOLDS
// ============================================================================
struct SafetyThresholds
{
  int16_t CUR_WARN_A;
  int16_t CUR_LIMP_A;
  int16_t TEMP_WARN_C;
  int16_t TEMP_LIMP_C;
};

// ============================================================================
// PURE RAW SAFETY (NO SIDE EFFECT)
// ============================================================================
SafetyState evaluateSafetyRaw(
  const SafetyInput& in,
  const SafetyThresholds& th);

// ============================================================================
// STABILITY LAYER (ANTI-FLAP / HYSTERESIS)
// ============================================================================
void updateSafetyStability(
  SafetyState raw,
  uint32_t now,
  uint8_t& autoReverseCount,
  bool& autoReverseActive,
  DriveEvent& lastDriveEvent);

// ============================================================================
// 🔴 HELPER (INLINE)
// ============================================================================
inline bool isSafetyCritical(SafetyState s)
{
  return (s == SafetyState::EMERGENCY);
}

inline bool isSafetyLimited(SafetyState s)
{
  return (s == SafetyState::LIMP);
}

#endif

