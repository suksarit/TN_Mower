// ============================================================================
// VoltageManager.cpp 
// ============================================================================

#include <Arduino.h>

#include "VoltageManager.h"
#include "GlobalState.h"
#include "HardwareConfig.h"
#include "FaultManager.h"   

void updateVoltageWarning(uint32_t now)
{
  const float HYST = 0.5f;
  const uint16_t WARN_BLINK = 500;
  const uint16_t CRIT_BLINK = 150;

  static uint8_t level = 0;
  static uint32_t lastToggle_ms = 0;
  static bool buzzerOn = false;

  // ==================================================
  // 🔴 MEDIAN FILTER (3 sample)
  // ==================================================
  static float vHist[3] = {24.0f, 24.0f, 24.0f};

  vHist[0] = vHist[1];
  vHist[1] = vHist[2];
  vHist[2] = engineVolt;

  float a = vHist[0];
  float b = vHist[1];
  float c = vHist[2];

  float v24 =
    max(min(a, b),
    min(max(a, b), c));  // median 3

  // ==================================================
  // 🔴 SENSOR SANITY
  // ==================================================
  if (v24 < 10.0f || v24 > 35.0f)
  {
#if DEBUG_SERIAL
    Serial.println(F("[VOLT] SENSOR FAULT"));
#endif
    latchFault(FaultCode::SENSOR_TIMEOUT);
    v24 = 24.0f;  // fallback
  }

  // ==================================================
  // 🔴 SENSOR TIMEOUT
  // ==================================================
  if (now - wdSensor.lastUpdate_ms > wdSensor.timeout_ms)
  {
#if DEBUG_SERIAL
    Serial.println(F("[VOLT] SENSOR TIMEOUT"));
#endif

    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(RELAY_WARN, LOW);

    level = 0;
    buzzerOn = false;

    latchFault(FaultCode::SENSOR_TIMEOUT);
    return;
  }

  // ==================================================
  // 🔴 LEVEL DECISION (HYSTERESIS)
  // ==================================================
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

  // ==================================================
  // 🔴 CHANGE DETECT (debounce)
  // ==================================================
  static uint32_t levelChangeStart = 0;

  if (newLevel != level)
  {
    if (levelChangeStart == 0)
    {
      levelChangeStart = now;
    }
    else if (now - levelChangeStart > 100) // debounce 100ms
    {
      level = newLevel;
      lastToggle_ms = now;
      buzzerOn = false;
      levelChangeStart = 0;
    }
  }
  else
  {
    levelChangeStart = 0;
  }

  // ==================================================
  // 🔴 OUTPUT CONTROL
  // ==================================================
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

      digitalWrite(PIN_BUZZER, buzzerOn ? HIGH : LOW);
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

      digitalWrite(PIN_BUZZER, buzzerOn ? HIGH : LOW);
      break;
    }
  }
}

