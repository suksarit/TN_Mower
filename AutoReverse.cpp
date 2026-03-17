// ============================================================================
// AutoReverse.cpp (FIXED + STABLE + FULL FLOW)
// ============================================================================

#include <Arduino.h>

#include "AutoReverse.h"
#include "GlobalState.h"
#include "SafetyManager.h"
#include "FaultManager.h"
#include "HardwareConfig.h"

// ======================================================
// LOCAL STATE
// ======================================================

static uint32_t autoReverseStart_ms = 0;
static bool reverseRecoveryActive = false;
static uint32_t reverseLockUntil_ms = 0;

// ======================================================
// APPLY AUTO REVERSE (MAIN ENTRY)
// ======================================================

void applyAutoReverse(float &finalTargetL,
                      float &finalTargetR,
                      uint32_t now)
{
  // ==================================================
  // 1️⃣ ACTIVE AUTO REVERSE
  // ==================================================

  if (autoRev.active)
  {
    if (ibusCommLost || requireIbusConfirm || systemState != SystemState::ACTIVE)
    {
      autoRev.active = false;
      autoReverseActive = false;
      return;
    }

    // ต้องมีโหลดจริง (กัน false trigger)
    if (abs(curL) < 120 && abs(curR) < 120)
    {
      autoRev.active = false;
      autoReverseActive = false;
      return;
    }

    // ---------------- RUNNING ----------------
    if (now - autoRev.start_ms < autoRev.duration_ms)
    {
      int8_t dirL = 0;
      int8_t dirR = 0;

      // LEFT
      if (lastDirL > 0) dirL = -1;
      else if (lastDirL < 0) dirL = 1;
      else if (curL > 0) dirL = -1;
      else if (curL < 0) dirL = 1;
      else {
        autoRev.active = false;
        autoReverseActive = false;
        return;
      }

      // RIGHT
      if (lastDirR > 0) dirR = -1;
      else if (lastDirR < 0) dirR = 1;
      else if (curR > 0) dirR = -1;
      else if (curR < 0) dirR = 1;
      else {
        autoRev.active = false;
        autoReverseActive = false;
        return;
      }

      finalTargetL = constrain(dirL * autoRev.pwm, -PWM_TOP, PWM_TOP);
      finalTargetR = constrain(dirR * autoRev.pwm, -PWM_TOP, PWM_TOP);

      return;
    }

    // ---------------- FINISHED ----------------
    autoRev.active = false;
    autoReverseActive = false;

    reverseRecoveryActive = true;
    autoReverseStart_ms = now;
  }

  // ==================================================
  // 2️⃣ RECOVERY WINDOW (รวมใน flow เดียวกัน)
  // ==================================================

  if (reverseRecoveryActive)
  {
    if (now - autoReverseStart_ms < REVERSE_RECOVERY_MS)
    {
      finalTargetL *= REVERSE_RECOVERY_LIMIT;
      finalTargetR *= REVERSE_RECOVERY_LIMIT;
      return;
    }

    reverseRecoveryActive = false;
  }
}

// ======================================================
// START AUTO REVERSE
// ======================================================

void startAutoReverse(uint32_t now)
{
  if (getDriveSafety() != SafetyState::SAFE)
    return;

  if (reverseRecoveryActive)
    return;

  if (abs(targetL) < 200 && abs(targetR) < 200)
    return;

  if (driverState != DriverState::ACTIVE)
    return;

  if (now < reverseLockUntil_ms)
    return;

  reverseLockUntil_ms = now + 1500;

  static uint32_t lastReverse_ms = 0;
  constexpr uint32_t REVERSE_COOLDOWN_MS = 1200;

  if (lastReverse_ms != 0 &&
      now - lastReverse_ms < REVERSE_COOLDOWN_MS)
    return;

  if (autoReverseCount >= MAX_AUTO_REVERSE)
  {
    latchFault(FaultCode::OVER_CURRENT);
    return;
  }

  if (abs(curL) < 200 && abs(curR) < 200)
    return;

  // --------------------------------------------------
  // PARAM ESCALATION
  // --------------------------------------------------
  uint16_t reverseTime;
  int16_t reversePWM;

  switch (autoReverseCount)
  {
    case 0: reverseTime = 320; reversePWM = 260; break;
    case 1: reverseTime = 450; reversePWM = 320; break;
    case 2: reverseTime = 650; reversePWM = 380; break;
    default: reverseTime = 700; reversePWM = 420; break;
  }

  reversePWM = constrain(reversePWM, 200, PWM_TOP / 2);

  // --------------------------------------------------
  // START
  // --------------------------------------------------
  autoRev.active = true;
  autoRev.start_ms = now;
  autoRev.duration_ms = reverseTime;
  autoRev.pwm = reversePWM;

  autoReverseActive = true;
  autoReverseStart_ms = now;

  autoReverseCount++;
  lastReverse_ms = now;

  lastDriveEvent = DriveEvent::AUTO_REVERSE;
}