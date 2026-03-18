// ============================================================================
// DriveProtection.h (FIXED - NO DUPLICATE TYPE)
// ============================================================================

#pragma once

#include <stdint.h>
#include "SystemTypes.h"   // ✅ ใช้ DriveEvent จากที่นี่

// ======================================================
// CONFIG
// ======================================================

namespace DriveProtectionConfig
{
  constexpr float STALL_CURRENT_A        = 55.0f;
  constexpr float STALL_MAX_A            = 70.0f;
  constexpr uint32_t STALL_TIME_MS       = 300;

  constexpr uint32_t STUCK_TIME_MS       = 500;
  constexpr int16_t  STUCK_PWM_MIN       = 150;

  constexpr float LOCK_CURRENT_A         = 80.0f;

  constexpr float IMBALANCE_DIFF_A       = 20.0f;
  constexpr float IMBALANCE_CORRECT_GAIN = 0.1f;

  constexpr float MAX_CURRENT_SOFT_A     = 60.0f;
  constexpr float MAX_CURRENT_HARD_A     = 75.0f;
}

// ======================================================
// STATE
// ======================================================

extern DriveEvent lastDriveEvent;

extern uint32_t stallStartTime;
extern uint32_t stuckStartTime;

// ======================================================
// DETECTION
// ======================================================

void detectWheelStuck(uint32_t now);
void detectWheelLock();

bool detectMotorStall(uint32_t now,
                      float curL_A,
                      float curR_A);

// 🔴 ต้องมี parameter
void detectSideImbalanceAndSteer(float &targetL,
                                 float &targetR,
                                 float curL_A,
                                 float curR_A);

// ======================================================
// CONTROL
// ======================================================

float computeStallScale(uint32_t now,
                        float curL_A,
                        float curR_A);

void applyDriveLimits(float &finalTargetL,
                      float &finalTargetR,
                      float curA_L,
                      float curA_R);