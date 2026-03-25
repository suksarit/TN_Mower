// ============================================================================
// AutoReverse.cpp (PRODUCTION  
// ============================================================================

#include <Arduino.h>
#include <math.h>

#include "GlobalState.h"
#include "SystemTypes.h"
#include "HardwareConfig.h"
#include "SensorManager.h"
#include "AutoReverse.h"

void startAutoReverse(uint32_t now);
void applyAutoReverse(float &tL, float &tR, uint32_t now);
void resetAutoReverse();

// ======================================================
// INTERNAL STATE
// ======================================================

static uint32_t reverseStart_ms = 0;
static uint8_t reversePhase = 0;

static float reversePower = 0.0f; // ramp

// CONFIG
constexpr uint16_t REVERSE_DURATION_MS = 600;
constexpr uint16_t PAUSE_DURATION_MS   = 300;
constexpr float REVERSE_POWER_MAX = 0.5f;
constexpr float REVERSE_RAMP_RATE = 0.05f;

// ============================================================================
// START AUTO REVERSE
// ============================================================================

void startAutoReverse(uint32_t now)
{
  // 🔴 ห้ามเริ่มถ้าระบบไม่พร้อม
  if (autoReverseActive ||
      systemState != SystemState::ACTIVE ||
      driveState == DriveState::LOCKED)
    return;

  autoReverseActive = true;
  reverseStart_ms = now;
  reversePhase = 0;
  reversePower = 0.0f;
}

// ============================================================================
// APPLY AUTO REVERSE
// ============================================================================

void applyAutoReverse(float &tL, float &tR, uint32_t now)
{
  if (systemState != SystemState::ACTIVE ||
      driverState != DriverState::ACTIVE)
  {
    resetAutoReverse();
    return;
  }

  if (!autoReverseActive)
    return;

  uint32_t dt_ms = now - reverseStart_ms;

  switch (reversePhase)
  {
    case 0:
      tL = 0;
      tR = 0;

      if (dt_ms > PAUSE_DURATION_MS)
      {
        reversePhase = 1;
        reverseStart_ms = now;
      }
      break;

    case 1:
    {
      // 🔴 FIX: time-based ramp
      reversePower += REVERSE_RAMP_RATE * controlDt_s;
      reversePower = constrain(reversePower, 0.0f, REVERSE_POWER_MAX);

      float power = PWM_TOP * reversePower;

      tL = -power;
      tR = -power;

      if (dt_ms > REVERSE_DURATION_MS)
      {
        reversePhase = 2;
        reverseStart_ms = now;
      }
      break;
    }

    case 2:
      tL = 0;
      tR = 0;

      autoReverseActive = false;
      reversePhase = 0;
      reversePower = 0.0f;
      break;
  }
}

// ============================================================================
// RESET
// ============================================================================

void resetAutoReverse()
{
  autoReverseActive = false;
  reversePhase = 0;
  reverseStart_ms = 0;
  reversePower = 0.0f;
}

