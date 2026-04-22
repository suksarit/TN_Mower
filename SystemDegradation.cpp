// SystemDegradation.cpp

#include "SystemDegradation.h"

// ======================================================
// INTERNAL STATE
// ======================================================
static SystemMode currentMode = SystemMode::NORMAL;

// ======================================================
// SET MODE (priority based)
// ======================================================
void setSystemMode(SystemMode mode)
{
    // 🔴 priority: ห้าม downgrade ง่าย ๆ
    if ((uint8_t)mode > (uint8_t)currentMode)
        currentMode = mode;
}

// ======================================================
SystemMode getSystemMode(void)
{
    return currentMode;
}

// ======================================================
bool isDegraded(void)
{
    return currentMode != SystemMode::NORMAL &&
           currentMode != SystemMode::FAULT;
}

// ======================================================
// POWER SCALE (หัวใจของ degraded)
// ======================================================
float getPowerScale(void)
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

// ======================================================
float getRampScale(void)
{
    switch (currentMode)
    {
        case SystemMode::NORMAL:       return 1.0f;
        case SystemMode::DEGRADED_L1:  return 0.7f;
        case SystemMode::DEGRADED_L2:  return 0.4f;
        case SystemMode::FAULT:        return 0.0f;
    }
    return 1.0f;
}

