// ThermalManager.cpp

#include <Arduino.h>

#include "ThermalManager.h"
#include "GlobalState.h"
#include "SafetyManager.h"

// ======================================================
// CONFIG
// ======================================================

constexpr int TEMP_WARN_C = 75;
constexpr int TEMP_LIMP_C = 90;
constexpr int TEMP_CRITICAL_C = 105;

// smoothing
static float thermalScale = 1.0f;
static bool thermalEmergency = false;

// ======================================================
// MAIN UPDATE
// ======================================================

void updateThermalManager(uint32_t now)
{
  int tMax = max(tempDriverL, tempDriverR);

  // --------------------------------------------------
  // EMERGENCY
  // --------------------------------------------------
  if (tMax >= TEMP_CRITICAL_C ||
      getDriveSafety() == SafetyState::EMERGENCY)
  {
    thermalEmergency = true;
    thermalScale = 0.0f;
    return;
  }

  thermalEmergency = false;

  // --------------------------------------------------
  // NORMAL SCALE
  // --------------------------------------------------
  float targetScale = 1.0f;

  if (tMax >= TEMP_LIMP_C)
  {
    targetScale = 0.3f;
  }
  else if (tMax >= TEMP_WARN_C)
  {
    float ratio =
      (float)(tMax - TEMP_WARN_C) /
      (float)(TEMP_LIMP_C - TEMP_WARN_C);

    targetScale = 1.0f - (ratio * 0.7f);
  }

  // --------------------------------------------------
  // SMOOTHING (กันกระตุก)
  // --------------------------------------------------
  thermalScale += (targetScale - thermalScale) * 0.1f;

  if (thermalScale < 0.0f) thermalScale = 0.0f;
  if (thermalScale > 1.0f) thermalScale = 1.0f;
}

// ======================================================
// GETTERS
// ======================================================

float getThermalScale(void)
{
  return thermalScale;
}

bool isThermalEmergency(void)
{
  return thermalEmergency;
}