// ============================================================================
// FanManager.cpp 
// ============================================================================

#include <Arduino.h>

#include "FanManager.h"
#include "MotorDriver.h"
#include "ThermalManager.h"
#include "HardwareConfig.h"
#include "FaultManager.h"

// ======================================================
// FUNCTION: updateDriverFans
// ROLE: ควบคุมพัดลมแบบ smooth + fail-safe + industrial grade
// ======================================================
void updateDriverFans(void)
{
  // ======================================================
  // CONFIG
  // ======================================================
  const uint8_t LEVEL_HYST = 5;        // hysteresis กันแกว่ง
  const uint16_t RAMP_STEP = 15;       // ramp นุ่มขึ้น

  const uint16_t FAN_PWM_TOP = 255;    // 🔴 แยกจาก PWM มอเตอร์
  const uint16_t FAN_MIN_PWM = 60;     // 🔴 ต่ำสุดที่พัดลมหมุนได้

  // ======================================================
  // STATIC STATE
  // ======================================================
  static uint8_t lastLevel = 0;
  static uint16_t currentPWM = 0;

  // ======================================================
  // 🔴 HARD FAILSAFE (สำคัญ)
  // ======================================================
  if (isFaultActive())
  {
    currentPWM = FAN_PWM_TOP;

    setFanPWM_L(currentPWM);
    setFanPWM_R(currentPWM);
    return;
  }

  // ======================================================
  // READ INPUT
  // ======================================================
  uint8_t rawLevel = getThermalFanLevel();

  // ======================================================
  // 🔴 SENSOR SANITY CHECK
  // ======================================================
  if (rawLevel > 255) rawLevel = 255;

  // กัน sensor drop (เช่นสายหลุด)
  if (rawLevel == 0 && lastLevel > 100)
  {
    rawLevel = lastLevel;
  }

  // ======================================================
  // HYSTERESIS (กันค่าเด้ง)
  // ======================================================
  if (abs((int)rawLevel - (int)lastLevel) < LEVEL_HYST)
  {
    rawLevel = lastLevel;
  }
  else
  {
    lastLevel = rawLevel;
  }

  // ======================================================
  // MAP → PWM
  // ======================================================
  uint16_t targetPWM =
    (uint16_t)map(rawLevel, 0, 255, 0, FAN_PWM_TOP);

  if (targetPWM > FAN_PWM_TOP)
    targetPWM = FAN_PWM_TOP;

  // ======================================================
  // 🔴 MIN SPIN PROTECTION
  // ======================================================
  if (targetPWM > 0 && targetPWM < FAN_MIN_PWM)
  {
    targetPWM = FAN_MIN_PWM;
  }

  // ======================================================
  // RAMP (smooth ไม่มีสะดุด)
  // ======================================================
  if (currentPWM < targetPWM)
  {
    uint16_t diff = targetPWM - currentPWM;
    currentPWM += (diff > RAMP_STEP) ? RAMP_STEP : diff;
  }
  else if (currentPWM > targetPWM)
  {
    uint16_t diff = currentPWM - targetPWM;
    currentPWM -= (diff > RAMP_STEP) ? RAMP_STEP : diff;
  }

  // ======================================================
  // APPLY PWM
  // ======================================================
  setFanPWM_L(currentPWM);
  setFanPWM_R(currentPWM);
}

