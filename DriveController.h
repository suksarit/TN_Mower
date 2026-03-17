// ============================================================================
// DriveController.h  (TN MOWER - FIXED STRUCTURE)
// ============================================================================

#pragma once

#include "SystemTypes.h"

// ============================================================================
// 🔴 CORE REAL-TIME CONTROL (เรียกใน control loop เท่านั้น)
// ============================================================================

// คำนวณ state + control logic
void runDrive(uint32_t now);

// apply state → buffer PWM (ไม่เขียน hardware ตรง)
void applyDrive(uint32_t now);

// ตัดมอเตอร์ทันที (hard safe)
void driveSafe();

// ============================================================================
// 🔴 INPUT → TARGET STAGE
// ============================================================================

// แปลง RC → targetL / targetR
void updateDriveTarget();

// ============================================================================
// 🔴 TARGET PIPELINE
// compute → limit → ramp → output
// ============================================================================

// คำนวณ target จริง (mix + logic)
void computeDriveTarget(
  float &finalTargetL,
  float &finalTargetR,
  uint32_t now);

// จำกัดกำลังตาม current / safety
void applyDriveLimits(
  float &finalTargetL,
  float &finalTargetR,
  float curA_L,
  float curA_R);

// ramp ป้องกันกระชาก
void updateDriveRamp(
  float finalTargetL,
  float finalTargetR);

// ส่งค่าไป PWM buffer
void outputMotorPWM();

// ============================================================================
// 🔴 LOAD / STALL MANAGEMENT
// ============================================================================

// scale PWM ตามโหลด
float computeStallScale(
  uint32_t now,
  float curLeft,
  float curRight);

// ============================================================================
// 🔴 AUTO REVERSE / SAFETY ACTION
// ============================================================================

// เริ่ม auto reverse
void startAutoReverse(uint32_t now);

// 🔴 CRITICAL: ตัด drive แบบ soft stop (ใช้ตอน spike / fault)
void forceDriveSoftStop(uint32_t now);

// ============================================================================
// 🔴 FAULT DETECTION (ใช้ใน background task)
// ============================================================================

void detectWheelStuck(uint32_t now);
void detectWheelLock();
bool detectMotorStall();
void detectSideImbalanceAndSteer();

// ============================================================================
// 🔴 STATE QUERY
// ============================================================================

// ใช้ตรวจว่าไม่มีคำสั่งขับ
bool driveCommandZero();


