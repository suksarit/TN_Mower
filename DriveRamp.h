// ============================================================================
// DriveRamp.h (FULL INTERFACE + SAFE)
// ============================================================================

#pragma once

#include <stdint.h>

// ======================================================
// MAIN RAMP
// ======================================================

// ใช้ใน DriveController
void updateDriveRamp(float &finalTargetL, float &finalTargetR);

// ======================================================
// CONTROL / MAINTENANCE
// ======================================================

// reset ramp state (สำคัญมาก)
// ต้องเรียกเมื่อ:
// - fault
// - driver disable
// - soft stop
void resetDriveRamp();