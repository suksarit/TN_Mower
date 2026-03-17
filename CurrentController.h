// ============================================================================
// CurrentController.h (FULL + SAFE CONTROL)
// ============================================================================

#pragma once

#include <stdint.h>

// ======================================================
// MAIN CURRENT LOOP
// ======================================================

// ใช้ใน DriveController
void applyCurrentLoop(float &targetL, float &targetR);

// ======================================================
// CONTROL / MAINTENANCE
// ======================================================

// reset integrator (ต้องเรียกเมื่อ:
// - fault
// - soft stop
// - driver disable)
void resetCurrentLoop();