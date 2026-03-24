// ============================================================================
// ThermalManager.cpp (FINAL - ROBUST + SENSOR SAFE)
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

// 🔴 sanity limit
constexpr int TEMP_MIN_VALID = -20;
constexpr int TEMP_MAX_VALID = 150;

// ======================================================
// STATE
// ======================================================

static float powerScale = 1.0f;
static float thermalScale = 1.0f;
static bool thermalEmergency = false;

static uint8_t fanLevel = 0;
static float fanLevelF = 0.0f;

// ======================================================
// MAIN UPDATE
// ======================================================

void updateThermalManager(uint32_t now)
{
  int tL = tempDriverL;
  int tR = tempDriverR;

  // ==================================================
  // 🔴 SANITY CHECK SENSOR
  // ==================================================
  if (tL < TEMP_MIN_VALID || tL > TEMP_MAX_VALID) tL = 80;
  if (tR < TEMP_MIN_VALID || tR > TEMP_MAX_VALID) tR = 80;

  int tMax = max(tL, tR);

  // ==================================================
  // EMERGENCY
  // ==================================================
  if (tMax >= TEMP_CRITICAL_C ||
      getDriveSafety() == SafetyState::EMERGENCY)
  {
    thermalEmergency = true;
    fanLevel = 255;
    thermalScale = 0.0f;
    powerScale = 0.0f;
    return;
  }

  thermalEmergency = false;

  // ==================================================
  // THERMAL SCALE
  // ==================================================
  float targetScale = 1.0f;

  if (tMax >= TEMP_LIMP_C)
    targetScale = 0.3f;
  else if (tMax >= TEMP_WARN_C)
  {
    float r = (float)(tMax - TEMP_WARN_C) /
              (TEMP_LIMP_C - TEMP_WARN_C);

    targetScale = 1.0f - (r * 0.7f);
  }

  // ==================================================
  // CURRENT SCALE (🔴 FIX abs)
  // ==================================================
  float curMax = 0;

  for (uint8_t i = 0; i < 4; i++)
    curMax = max(curMax, fabs(curA[i]));

  float currentScale = 1.0f;

  constexpr float CUR_WARN = 35.0f;
  constexpr float CUR_LIMIT = 60.0f;

  if (curMax > CUR_LIMIT)
    currentScale = 0.3f;
  else if (curMax > CUR_WARN)
  {
    float r = (curMax - CUR_WARN) /
              (CUR_LIMIT - CUR_WARN);

    currentScale = 1.0f - (r * 0.7f);
  }

  // ==================================================
  // VOLTAGE SCALE (SANITY)
  // ==================================================
  float v = engineVolt;

  if (v < 10 || v > 35) v = 24.0f;

  float voltageScale = 1.0f;

  constexpr float V_WARN = 22.5f;
  constexpr float V_CRIT = 20.5f;

  if (v < V_CRIT)
    voltageScale = 0.3f;
  else if (v < V_WARN)
  {
    float r = (V_WARN - v) / (V_WARN - V_CRIT);
    voltageScale = 1.0f - (r * 0.7f);
  }

  // ==================================================
  // COMBINE (🔴 clamp กัน drop แรงเกิน)
  // ==================================================
  float combined = targetScale * currentScale * voltageScale;

  if (combined < 0.2f) combined = 0.2f;

  powerScale += (combined - powerScale) * 0.1f;

  // ==================================================
  // FAN
  // ==================================================
  uint8_t targetFan;

  if (tMax < TEMP_WARN_C)
    targetFan = 60;
  else if (tMax < TEMP_LIMP_C)
  {
    float r = (float)(tMax - TEMP_WARN_C) /
              (TEMP_LIMP_C - TEMP_WARN_C);

    targetFan = 80 + r * 120;
  }
  else
    targetFan = 255;

  fanLevelF += (targetFan - fanLevelF) * 0.1f;
  fanLevel = (uint8_t)fanLevelF;

  thermalScale += (targetScale - thermalScale) * 0.1f;
}

// ======================================================
// GETTERS
// ======================================================

float getThermalScale(void) { return thermalScale; }
bool isThermalEmergency(void) { return thermalEmergency; }
float getPowerScale(void) { return powerScale; }
uint8_t getThermalFanLevel(void) { return fanLevel; }


