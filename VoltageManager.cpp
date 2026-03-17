// VoltageManager.cpp

#include <Arduino.h>

#include "VoltageManager.h"
#include "GlobalState.h"
#include "HardwareConfig.h"

// ======================================================
// VOLTAGE WARNING
// ======================================================

void updateVoltageWarning(uint32_t now)
{
  static uint32_t lastToggle_ms = 0;
  static bool buzzerOn = false;

  float v24 = engineVolt;

  // sensor timeout
  if (now - wdSensor.lastUpdate_ms > wdSensor.timeout_ms)
  {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(RELAY_WARN, LOW);
    buzzerOn = false;
    return;
  }

  // normal
  if (v24 >= V_WARN_LOW)
  {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(RELAY_WARN, LOW);
    buzzerOn = false;
    return;
  }

  // LEVEL 1 (blink)
  if (v24 < V_WARN_LOW && v24 >= V_WARN_CRITICAL)
  {
    digitalWrite(RELAY_WARN, HIGH);

    if (now - lastToggle_ms >= 500)
    {
      lastToggle_ms = now;
      buzzerOn = !buzzerOn;
      digitalWrite(PIN_BUZZER, buzzerOn ? HIGH : LOW);
    }
    return;
  }

  // LEVEL 2 (continuous)
  if (v24 < V_WARN_CRITICAL)
  {
    digitalWrite(PIN_BUZZER, HIGH);
    digitalWrite(RELAY_WARN, HIGH);
  }
}

