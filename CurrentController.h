// ============================================================================
// CurrentController.h (TORQUE PID CONTROL  
// ============================================================================

#pragma once

#include <stdint.h>

// ======================================================
// TORQUE CONTROL (CURRENT PID)
// ======================================================
//
// INPUT:
//   targetCurrentL / targetCurrentR  → Amp (กระแสเป้าหมาย)
//
// OUTPUT:
//   เขียนค่าไปที่ curL / curR (PWM global)
//
// ใช้ใน: DriveController.cpp
//
void applyCurrentPID(float targetCurrentL, float targetCurrentR);

// ======================================================
// CONTROL RESET (CRITICAL SAFETY)
// ======================================================
//
// ต้องเรียกเมื่อ:
// - fault
// - soft stop
// - driver disable
// - system reset
//
void resetCurrentLoop();



