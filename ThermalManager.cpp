// ============================================================================
// ThermalManager.cpp 
// ============================================================================

#include <Arduino.h>

#include "ThermalManager.h"
#include "GlobalState.h"
#include "SafetyManager.h"
#include "FaultManager.h"   

// ======================================================
// CONFIG
// ======================================================

constexpr int TEMP_WARN_C = 75;
constexpr int TEMP_LIMP_C = 90;
constexpr int TEMP_CRITICAL_C = 105;

// 🔴 sanity limit
constexpr int TEMP_MIN_VALID = -20;
constexpr int TEMP_MAX_VALID = 150;

// 🔴 current sanity
constexpr float CURRENT_MAX_VALID = 120.0f;

// 🔴 voltage sanity
constexpr float VOLT_MIN_VALID = 10.0f;
constexpr float VOLT_MAX_VALID = 35.0f;

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
  (void)now;

  int tL = tempDriverL;
  int tR = tempDriverR;

  bool tempFault = false;

  // ==================================================
  // 🔴 SENSOR VALIDATION
  // ==================================================
  if (tL < TEMP_MIN_VALID || tL > TEMP_MAX_VALID)
  {
    tL = 80;
    tempFault = true;
  }

  if (tR < TEMP_MIN_VALID || tR > TEMP_MAX_VALID)
  {
    tR = 80;
    tempFault = true;
  }

  if (tempFault)
  {
#if DEBUG_SERIAL
    Serial.println(F("[THERMAL] TEMP SENSOR FAULT"));
#endif
    requestFault(FaultCode::SENSOR_TIMEOUT);
  }

  int tMax = max(tL, tR);

  // ==================================================
  // 🔴 EMERGENCY
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
  // 🔴 THERMAL SCALE
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
  // 🔴 CURRENT SCALE (FILTER + CLAMP)
  // ==================================================
  float curMax = 0;

  for (uint8_t i = 0; i < 4; i++)
  {
    float c = fabs(curA[i]);

    if (c > CURRENT_MAX_VALID)
    {
#if DEBUG_SERIAL
      Serial.println(F("[THERMAL] CURRENT SENSOR FAULT"));
#endif
      requestFault(FaultCode::SENSOR_TIMEOUT);
      c = CURRENT_MAX_VALID;
    }

    curMax = max(curMax, c);
  }

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
  // 🔴 VOLTAGE SCALE (SANITY)
  // ==================================================
  float v = engineVolt;

  if (v < VOLT_MIN_VALID || v > VOLT_MAX_VALID)
  {
#if DEBUG_SERIAL
    Serial.println(F("[THERMAL] VOLT SENSOR FAULT"));
#endif
    requestFault(FaultCode::SENSOR_TIMEOUT);
    v = 24.0f;
  }

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
  // 🔴 COMBINE (ANTI DROP)
  // ==================================================
  float combined = targetScale * currentScale * voltageScale;

  if (combined < 0.25f)
    combined = 0.25f;

  powerScale += (combined - powerScale) * 0.08f;

  if (powerScale > 1.0f) powerScale = 1.0f;
  if (powerScale < 0.0f) powerScale = 0.0f;

  // ==================================================
  // 🔴 FAN CONTROL (SMOOTH)
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

  // ==================================================
  // 🔴 THERMAL SCALE SMOOTH
  // ==================================================
  thermalScale += (targetScale - thermalScale) * 0.1f;

  if (thermalScale > 1.0f) thermalScale = 1.0f;
  if (thermalScale < 0.0f) thermalScale = 0.0f;
}

// ======================================================
// GETTERS
// ======================================================

float getThermalScale(void) { return thermalScale; }
bool isThermalEmergency(void) { return thermalEmergency; }
float getPowerScale(void) { return powerScale; }
uint8_t getThermalFanLevel(void) { return fanLevel; }

