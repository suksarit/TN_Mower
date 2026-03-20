// ============================================================================
// DriveProtection.h (PRODUCTION - SAFE + NO CONFLICT + EXTENDABLE)
// ============================================================================

#pragma once

#include <stdint.h>
#include "SystemTypes.h"   // ใช้ DriveEvent จากศูนย์กลาง

// ======================================================
// CONFIG (TUNABLE PARAMETERS)
// ======================================================

namespace DriveProtectionConfig
{
  // ---------------- STALL ----------------
  constexpr float STALL_CURRENT_A        = 55.0f;
  constexpr float STALL_MAX_A            = 70.0f;
  constexpr uint32_t STALL_TIME_MS       = 300;

  // ---------------- STUCK ----------------
  constexpr uint32_t STUCK_TIME_MS       = 500;
  constexpr int16_t  STUCK_PWM_MIN       = 150;

  // ---------------- LOCK ----------------
  constexpr float LOCK_CURRENT_A         = 80.0f;
  constexpr uint16_t LOCK_CONFIRM_MS     = 120;   // 🔴 เพิ่ม debounce

  // ---------------- IMBALANCE ----------------
  constexpr float IMBALANCE_DIFF_A       = 20.0f;
  constexpr float IMBALANCE_CORRECT_GAIN = 0.1f;

  // ---------------- CURRENT LIMIT ----------------
  constexpr float MAX_CURRENT_SOFT_A     = 60.0f;
  constexpr float MAX_CURRENT_HARD_A     = 75.0f;

  // ---------------- SLIP ----------------
  constexpr float SLIP_THRESHOLD         = 0.25f;
  constexpr float SLIP_MAX_REDUCTION     = 0.4f;
}

// ======================================================
// GLOBAL STATE
// ======================================================

// 🔴 event หลัก (ใช้ร่วมทั้งระบบ)
extern DriveEvent lastDriveEvent;

// 🔴 state สำหรับ detection
extern uint32_t stallStartTime;
extern uint32_t stuckStartTime;

// ======================================================
// SAFE EVENT SET (CRITICAL - ห้าม overwrite)
// ======================================================

inline void setDriveEvent(DriveEvent e)
{
  if (lastDriveEvent == DriveEvent::NONE)
    lastDriveEvent = e;
}

// ======================================================
// DETECTION FUNCTIONS
// ======================================================

// ตรวจ stuck → trigger auto reverse
void detectWheelStuck(uint32_t now);

// ตรวจ lock → fault
void detectWheelLock();

// ตรวจ stall (high current + low movement)
bool detectMotorStall(uint32_t now,
                      float curL_A,
                      float curR_A);

// ตรวจ imbalance → ปรับ steering
void detectSideImbalanceAndSteer(float &targetL,
                                 float &targetR,
                                 float curL_A,
                                 float curR_A);

// ======================================================
// CONTROL FUNCTIONS
// ======================================================

// scale กำลังตามโหลด (adaptive stall model)
float computeStallScale(uint32_t now,
                        float curL_A,
                        float curR_A);

// จำกัด output ขั้นสุดท้าย (slip + current limit)
void applyDriveLimits(float &finalTargetL,
                      float &finalTargetR,
                      float curA_L,
                      float curA_R);

// ======================================================
// OPTIONAL (DEBUG / EXTEND)
// ======================================================

// reset internal protection state (ใช้ตอน fault / reset system)
inline void resetDriveProtectionState()
{
  stallStartTime = 0;
  stuckStartTime = 0;
}

