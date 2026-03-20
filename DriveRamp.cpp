// ============================================================================
// DriveRamp.cpp (TORQUE SYSTEM SAFE  
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "HardwareConfig.h"
#include "SystemTypes.h"
#include "DriveRamp.h"

// ======================================================
// CONFIG
// ======================================================

constexpr float ACCEL_RATE = 2200.0f;   // PWM/sec
constexpr float DECEL_RATE = 3000.0f;
constexpr float STOP_RATE  = 4000.0f;

constexpr float SNAP_ZERO = 6.0f;

// ======================================================
// LOCAL STATE
// ======================================================

static uint32_t stuckStart_ms = 0;

// ============================================================================
// MAIN RAMP FUNCTION
// ============================================================================

void updateDriveRamp(float &finalTargetL,
                    float &finalTargetR)
{
  finalTargetL = constrain(finalTargetL, -PWM_TOP, PWM_TOP);
  finalTargetR = constrain(finalTargetR, -PWM_TOP, PWM_TOP);

  float dt = controlDt_s;
  if (dt <= 0.0001f) return;

  uint32_t now = millis();

  // ==================================================
  // ERROR (ใช้ target เดิมเป็น base)
  // ==================================================
  static float rampL = 0;
  static float rampR = 0;

  float errL = finalTargetL - rampL;
  float errR = finalTargetR - rampR;

  // ==================================================
  // SELECT RATE
  // ==================================================
  float rateL =
    (abs(finalTargetL) < SNAP_ZERO) ? STOP_RATE :
    (abs(finalTargetL) > abs(rampL)) ? ACCEL_RATE :
                                      DECEL_RATE;

  float rateR =
    (abs(finalTargetR) < SNAP_ZERO) ? STOP_RATE :
    (abs(finalTargetR) > abs(rampR)) ? ACCEL_RATE :
                                      DECEL_RATE;

  float stepL = rateL * dt;
  float stepR = rateR * dt;

  errL = constrain(errL, -stepL, stepL);
  errR = constrain(errR, -stepR, stepR);

  rampL += errL;
  rampR += errR;

  // ==================================================
  // SNAP TO ZERO
  // ==================================================
  if (abs(rampL) < SNAP_ZERO && abs(finalTargetL) < SNAP_ZERO)
    rampL = 0;

  if (abs(rampR) < SNAP_ZERO && abs(finalTargetR) < SNAP_ZERO)
    rampR = 0;

  // ==================================================
  // SAFE REVERSE
  // ==================================================
  constexpr int16_t REVERSE_SAFE_PWM = 120;

  bool reverseL =
    (rampL > 0 && finalTargetL < 0) ||
    (rampL < 0 && finalTargetL > 0);

  bool reverseR =
    (rampR > 0 && finalTargetR < 0) ||
    (rampR < 0 && finalTargetR > 0);

  if (reverseL)
  {
    if (abs(rampL) > REVERSE_SAFE_PWM || now < revBlockUntilL)
      rampL = 0;
    else
    {
      revBlockUntilL = now + REVERSE_DEADTIME_MS;
      rampL = 0;
    }
  }

  if (reverseR)
  {
    if (abs(rampR) > REVERSE_SAFE_PWM || now < revBlockUntilR)
      rampR = 0;
    else
    {
      revBlockUntilR = now + REVERSE_DEADTIME_MS;
      rampR = 0;
    }
  }

  // ==================================================
  // OUTPUT (แก้ target เท่านั้น)
  // ==================================================
  finalTargetL = rampL;
  finalTargetR = rampR;
}

// ============================================================================
// RESET DRIVE RAMP
// ============================================================================

void resetDriveRamp()
{
  // reset internal ramp state
  static float rampL = 0;
  static float rampR = 0;

  rampL = 0;
  rampR = 0;

  revBlockUntilL = 0;
  revBlockUntilR = 0;

  stuckStart_ms = 0;
}

