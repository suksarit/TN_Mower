// ============================================================================
// VoltageManager.cpp (FINAL - ROBUST + SENSOR SAFE)
// ============================================================================

#include <Arduino.h>
#include "VoltageManager.h"
#include "GlobalState.h"
#include "HardwareConfig.h"

void updateVoltageWarning(uint32_t now)
{
  const float HYST = 0.5f;
  const uint16_t WARN_BLINK = 500;
  const uint16_t CRIT_BLINK = 150;

  static uint8_t level = 0;
  static uint32_t lastToggle_ms = 0;
  static bool buzzerOn = false;

  float v24 = engineVolt;

  // ==================================================
  // 🔴 SANITY CHECK
  // ==================================================
  if (v24 < 10 || v24 > 35)
    return;  // ignore ค่าเพี้ยน

  // ==================================================
  // SENSOR TIMEOUT
  // ==================================================
  if (now - wdSensor.lastUpdate_ms > wdSensor.timeout_ms)
  {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(RELAY_WARN, LOW);
    level = 0;
    return;
  }

  uint8_t newLevel = level;

  switch (level)
  {
    case 0:
      if (v24 < V_WARN_LOW)
        newLevel = 1;
      break;

    case 1:
      if (v24 < V_WARN_CRITICAL)
        newLevel = 2;
      else if (v24 > (V_WARN_LOW + HYST))
        newLevel = 0;
      break;

    case 2:
      if (v24 > (V_WARN_CRITICAL + HYST))
        newLevel = 1;
      break;
  }

  if (newLevel != level)
  {
    level = newLevel;
    lastToggle_ms = now;
    buzzerOn = false;
  }

  switch (level)
  {
    case 0:
      digitalWrite(PIN_BUZZER, LOW);
      digitalWrite(RELAY_WARN, LOW);
      break;

    case 1:
    {
      digitalWrite(RELAY_WARN, HIGH);

      if (now - lastToggle_ms >= WARN_BLINK)
      {
        lastToggle_ms = now;
        buzzerOn = !buzzerOn;
      }

      digitalWrite(PIN_BUZZER, buzzerOn);
      break;
    }

    case 2:
    {
      digitalWrite(RELAY_WARN, HIGH);

      if (now - lastToggle_ms >= CRIT_BLINK)
      {
        lastToggle_ms = now;
        buzzerOn = !buzzerOn;
      }

      digitalWrite(PIN_BUZZER, buzzerOn);
      break;
    }
  }
}

