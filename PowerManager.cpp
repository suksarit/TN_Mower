// PowerManager.cpp

// ============================================================================
// PowerManager.cpp (CENTRAL POWER SCALING)
// ============================================================================

#include <Arduino.h>

#include "PowerManager.h"
#include "SystemDegradation.h"
#include "ThermalManager.h"

// ======================================================
// 🔴 INTERNAL FILTER (กัน jitter)
// ======================================================
static float filteredScale = 1.0f;

// ======================================================
// 🔴 FINAL POWER SCALE
// ======================================================
float getFinalPowerScale(void)
{
    // --------------------------------------------------
    // 1. READ FROM SYSTEM (degradation)
    // --------------------------------------------------
    float sysScale = getSystemPowerScale();

    // --------------------------------------------------
    // 2. READ FROM THERMAL
    // --------------------------------------------------
    float thermalScale = getThermalPowerScale();

    // --------------------------------------------------
    // 3. COMBINE (หัวใจ)
    // --------------------------------------------------
    float raw = sysScale * thermalScale;

    // --------------------------------------------------
    // 4. CLAMP
    // --------------------------------------------------
    if (raw > 1.0f) raw = 1.0f;
    if (raw < 0.0f) raw = 0.0f;

    // --------------------------------------------------
    // 5. LOW-PASS FILTER (กันกระชาก)
    // --------------------------------------------------
    constexpr float ALPHA = 0.2f;   // ปรับได้ (0.1 = นุ่ม, 0.5 = ตอบไว)

    filteredScale = filteredScale * (1.0f - ALPHA) + raw * ALPHA;

    // --------------------------------------------------
    // 6. SAFETY SNAP
    // --------------------------------------------------
    if (filteredScale < 0.01f)
        filteredScale = 0.0f;

    return filteredScale;
}

