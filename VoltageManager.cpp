// ============================================================================
// VoltageManager.cpp (FINAL - HYSTERESIS REAL + STABLE STATE MACHINE)
// ============================================================================

#include <Arduino.h>

#include "VoltageManager.h"
#include "GlobalState.h"
#include "HardwareConfig.h"

// ======================================================
// FUNCTION: updateVoltageWarning
// ROLE: ควบคุม buzzer + relay ตามระดับแรงดัน (กันแกว่งจริง)
// ======================================================
void updateVoltageWarning(uint32_t now)
{
  // ======================================================
  // CONFIG (ปรับได้)
  // ======================================================
  const float HYST = 0.5f;        // กันแกว่ง (โวลต์)
  const uint16_t WARN_BLINK = 500;
  const uint16_t CRIT_BLINK = 150;

  // ======================================================
  // STATIC STATE
  // ======================================================
  static uint8_t level = 0;       // 0=NORMAL,1=WARN,2=CRITICAL
  static uint32_t lastToggle_ms = 0;
  static bool buzzerOn = false;

  float v24 = engineVolt;

  // ======================================================
  // SENSOR TIMEOUT
  // ======================================================
  if (now - wdSensor.lastUpdate_ms > wdSensor.timeout_ms)
  {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(RELAY_WARN, LOW);

    level = 0;
    buzzerOn = false;
    return;
  }

  // ======================================================
  // LEVEL DECISION (TRUE HYSTERESIS)
  // ======================================================
  uint8_t newLevel = level;

  switch (level)
  {
    // ----------------------------
    // NORMAL
    // ----------------------------
    case 0:
      if (v24 < V_WARN_LOW)
        newLevel = 1;
      break;

    // ----------------------------
    // WARN
    // ----------------------------
    case 1:
      if (v24 < V_WARN_CRITICAL)
        newLevel = 2;
      else if (v24 > (V_WARN_LOW + HYST))
        newLevel = 0;
      break;

    // ----------------------------
    // CRITICAL
    // ----------------------------
    case 2:
      if (v24 > (V_WARN_CRITICAL + HYST))
        newLevel = 1;
      break;
  }

  // ======================================================
  // LEVEL CHANGE RESET
  // ======================================================
  if (newLevel != level)
  {
    level = newLevel;
    lastToggle_ms = now;
    buzzerOn = false;
  }

  // ======================================================
  // OUTPUT CONTROL
  // ======================================================
  switch (level)
  {
    // ----------------------------
    // NORMAL
    // ----------------------------
    case 0:
      digitalWrite(PIN_BUZZER, LOW);
      digitalWrite(RELAY_WARN, LOW);
      break;

    // ----------------------------
    // WARNING (กระพริบช้า)
    // ----------------------------
    case 1:
    {
      digitalWrite(RELAY_WARN, HIGH);

      if (now - lastToggle_ms >= WARN_BLINK)
      {
        lastToggle_ms = now;
        buzzerOn = !buzzerOn;
      }

      digitalWrite(PIN_BUZZER, buzzerOn ? HIGH : LOW);
      break;
    }

    // ----------------------------
    // CRITICAL (กระพริบเร็ว)
    // ----------------------------
    case 2:
    {
      digitalWrite(RELAY_WARN, HIGH);

      if (now - lastToggle_ms >= CRIT_BLINK)
      {
        lastToggle_ms = now;
        buzzerOn = !buzzerOn;
      }

      digitalWrite(PIN_BUZZER, buzzerOn ? HIGH : LOW);
      break;
    }
  }
}


