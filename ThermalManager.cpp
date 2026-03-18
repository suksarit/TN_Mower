// ============================================================================
// ThermalManager.cpp (FINAL - PMS + FAN SYNC)
// ============================================================================

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

// ======================================================
// STATE
// ======================================================

static float powerScale = 1.0f;
static float thermalScale = 1.0f;

static bool thermalEmergency = false;

static uint8_t fanLevel = 0;
static float   fanLevelF = 0.0f;   // 🔴 smoothing

// ======================================================
// MAIN UPDATE
// ======================================================

void updateThermalManager(uint32_t now)
{
  int tMax = max(tempDriverL, tempDriverR);

  // ==================================================
  // EMERGENCY
  // ==================================================
  if (tMax >= TEMP_CRITICAL_C ||
      getDriveSafety() == SafetyState::EMERGENCY)
  {
    thermalEmergency = true;

    fanLevel = 255;

    thermalScale = 0.0f;
    powerScale   = 0.0f;   // 🔴 FIX สำคัญ

    return;
  }

  thermalEmergency = false;

  // ==================================================
  // THERMAL SCALE
  // ==================================================
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

  // ==================================================
  // CURRENT SCALE
  // ==================================================
  float curMax = max(curA[0], curA[1]);
  curMax = max(curMax, curA[2]);
  curMax = max(curMax, curA[3]);

  float currentScale = 1.0f;

  constexpr float CUR_WARN  = 35.0f;
  constexpr float CUR_LIMIT = 60.0f;

  if (curMax > CUR_LIMIT)
  {
    currentScale = 0.3f;
  }
  else if (curMax > CUR_WARN)
  {
    float r =
      (curMax - CUR_WARN) /
      (CUR_LIMIT - CUR_WARN);

    currentScale = 1.0f - (r * 0.7f);
  }

  // ==================================================
  // VOLTAGE SCALE
  // ==================================================
  float voltageScale = 1.0f;

  constexpr float V_WARN     = 22.5f;
  constexpr float V_CRITICAL = 20.5f;

  float v = engineVolt;

  if (v < V_CRITICAL)
  {
    voltageScale = 0.3f;
  }
  else if (v < V_WARN)
  {
    float r =
      (V_WARN - v) /
      (V_WARN - V_CRITICAL);

    voltageScale = 1.0f - (r * 0.7f);
  }

  // ==================================================
  // COMBINE SCALE
  // ==================================================
  float combined =
    targetScale * currentScale * voltageScale;

  powerScale += (combined - powerScale) * 0.1f;

  if (powerScale < 0.0f) powerScale = 0.0f;
  if (powerScale > 1.0f) powerScale = 1.0f;

  // ==================================================
  // FAN LEVEL (SMOOTHED)
  // ==================================================
  uint8_t targetFan;

  if (tMax < TEMP_WARN_C)
  {
    targetFan = 60;
  }
  else if (tMax < TEMP_LIMP_C)
  {
    float ratio =
      (float)(tMax - TEMP_WARN_C) /
      (float)(TEMP_LIMP_C - TEMP_WARN_C);

    targetFan = 80 + ratio * 120;
  }
  else
  {
    targetFan = 255;
  }

  // 🔴 smoothing fan
  fanLevelF += (targetFan - fanLevelF) * 0.1f;
  fanLevel = (uint8_t)fanLevelF;

  // ==================================================
  // THERMAL SMOOTH (optional)
  // ==================================================
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

float getPowerScale(void)
{
  return powerScale;
}

uint8_t getThermalFanLevel(void)
{
  return fanLevel;
}

