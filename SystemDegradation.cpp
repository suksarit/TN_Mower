// ============================================================================
// SystemDegradation.cpp (REFACTORED - CENTRAL MODE CONTROL)
// ============================================================================

#include "SystemDegradation.h"

// ======================================================
// INTERNAL STATE
// ======================================================
static SystemMode currentMode = SystemMode::NORMAL;

// ======================================================
// 🔴 MODE PRIORITY TABLE
// ค่ายิ่งมาก = รุนแรงกว่า
// ======================================================
static uint8_t getModePriority(SystemMode m)
{
    switch (m)
    {
        case SystemMode::NORMAL:       return 0;
        case SystemMode::DEGRADED_L1:  return 1;
        case SystemMode::DEGRADED_L2:  return 2;
        case SystemMode::FAULT:        return 3;
    }
    return 0;
}

// ======================================================
// SET MODE (มี priority กัน downgrade)
// ======================================================
void setSystemMode(SystemMode mode)
{
    // ❗ ไม่ให้ downgrade ง่าย (เช่น FAULT → NORMAL)
    if (getModePriority(mode) >= getModePriority(currentMode))
    {
        currentMode = mode;
    }
}

// ======================================================
SystemMode getSystemMode(void)
{
    return currentMode;
}

// ======================================================
// HELPER
// ======================================================
bool isSystemDegraded(void)
{
    return (currentMode == SystemMode::DEGRADED_L1 ||
            currentMode == SystemMode::DEGRADED_L2);
}

// ======================================================
bool isSystemFault(void)
{
    return (currentMode == SystemMode::FAULT);
}

// ======================================================
// 🔴 SYSTEM POWER SCALE (ใช้โดย PowerManager)
// ======================================================
float getSystemPowerScale(void)
{
    switch (currentMode)
    {
        case SystemMode::NORMAL:       return 1.0f;

        case SystemMode::DEGRADED_L1:  return 0.6f;

        case SystemMode::DEGRADED_L2:  return 0.3f;

        case SystemMode::FAULT:        return 0.0f;
    }

    return 1.0f;
}

