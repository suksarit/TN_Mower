// ============================================================================
// AutoReverse.h (CLEAN TRUE - NO GLOBAL DUPLICATE)
// ============================================================================

#pragma once

#include <stdint.h>

// ======================================================
// MAIN FUNCTIONS
// ======================================================

// ใช้ใน DriveController
void applyAutoReverse(float &L, float &R, uint32_t now);

// ใช้ใน DriveProtection
void startAutoReverse(uint32_t now);