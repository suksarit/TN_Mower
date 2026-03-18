// ============================================================================
// FanManager.cpp (FINAL - STABLE + RAMP + FAILSAFE)
// ============================================================================

#include <Arduino.h>

#include "FanManager.h"
#include "MotorDriver.h"
#include "ThermalManager.h"
#include "HardwareConfig.h"

// ======================================================
// FUNCTION: updateDriverFans
// ROLE: ควบคุมพัดลมแบบ smooth + กันแกว่ง + ปลอดภัย
// ======================================================
void updateDriverFans(void)
{
  // ======================================================
  // CONFIG
  // ======================================================
  const uint8_t LEVEL_HYST = 5;      // hysteresis กันแกว่ง
  const uint16_t RAMP_STEP = 20;     // ความเร็วในการไล่ PWM

  // ======================================================
  // STATIC STATE
  // ======================================================
  static uint8_t lastLevel = 0;
  static uint16_t currentPWM = 0;

  // ======================================================
  // READ INPUT
  // ======================================================
  uint8_t rawLevel = getThermalFanLevel();

  // ======================================================
  // FAILSAFE (กัน sensor พัง)
  // ======================================================
  if (rawLevel > 255) rawLevel = 255;   // กัน overflow

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
    (uint16_t)map(rawLevel, 0, 255, 0, PWM_TOP);

  // clamp กันหลุด range
  if (targetPWM > PWM_TOP) targetPWM = PWM_TOP;

  // ======================================================
  // RAMP (กันกระชาก)
  // ======================================================
  if (currentPWM < targetPWM)
  {
    currentPWM += RAMP_STEP;
    if (currentPWM > targetPWM)
      currentPWM = targetPWM;
  }
  else if (currentPWM > targetPWM)
  {
    if (currentPWM > RAMP_STEP)
      currentPWM -= RAMP_STEP;
    else
      currentPWM = 0;

    if (currentPWM < targetPWM)
      currentPWM = targetPWM;
  }

  // ======================================================
  // APPLY PWM
  // ======================================================
  setFanPWM_L(currentPWM);
  setFanPWM_R(currentPWM);
}


