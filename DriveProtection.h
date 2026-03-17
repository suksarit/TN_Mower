// ============================================================================
// DriveProtection.h (CLEAN + CONSISTENT)
// ============================================================================

#pragma once

#include <stdint.h>

// ======================================================
// DETECTION FUNCTIONS
// ======================================================

// ตรวจ stuck → เรียก auto reverse
void detectWheelStuck(uint32_t now);

// ตรวจ lock → fault ทันที
void detectWheelLock();

// ตรวจ stall แบบ dynamic
bool detectMotorStall();

// ตรวจ imbalance → correction
void detectSideImbalanceAndSteer();

// ======================================================
// CONTROL FUNCTIONS
// ======================================================

// scale PWM ตามโหลด (ใช้ใน DriveController)
float computeStallScale(uint32_t now,
                        float curL_A,
                        float curR_A);

// จำกัดกำลังขั้นสุดท้าย
void applyDriveLimits(float &finalTargetL,
                      float &finalTargetR,
                      float curA_L,
                      float curA_R);