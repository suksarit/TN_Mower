// ============================================================================
// AutoReverse.cpp (FIXED - TRUE CURRENT + STABLE DIRECTION + SAFE FLOW)
// ============================================================================

#include <Arduino.h>

#include "AutoReverse.h"
#include "GlobalState.h"
#include "SafetyManager.h"
#include "FaultManager.h"
#include "HardwareConfig.h"
#include "SensorManager.h"

// ======================================================
// LOCAL STATE
// ======================================================

static uint32_t autoReverseStart_ms = 0;
static bool reverseRecoveryActive = false;
static uint32_t reverseLockUntil_ms = 0;

// ======================================================
// APPLY AUTO REVERSE
// ======================================================

void applyAutoReverse(float &finalTargetL,
                      float &finalTargetR,
                      uint32_t now)
{
  // ==================================================
  // ACTIVE AUTO REVERSE
  // ==================================================

  if (autoRev.active)
  {
    // ---------------- SAFETY EXIT ----------------
    if (ibusCommLost || requireIbusConfirm || systemState != SystemState::ACTIVE)
    {
      autoRev.active = false;
      autoReverseActive = false;
      return;
    }

    // ---------------- TRUE CURRENT CHECK ----------------
    float curA_L = getMotorCurrentL();
    float curA_R = getMotorCurrentR();

    // ไม่มีโหลดจริง → ยกเลิก
    if (abs(curA_L) < 2.0f && abs(curA_R) < 2.0f)
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

      // ===== LEFT =====
      if (lastDirL != 0)
        dirL = -lastDirL;
      else
        dirL = (finalTargetL >= 0) ? -1 : 1;

      // ===== RIGHT =====
      if (lastDirR != 0)
        dirR = -lastDirR;
      else
        dirR = (finalTargetR >= 0) ? -1 : 1;

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
  // RECOVERY WINDOW
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
  // ---------------- SAFETY ----------------
  if (getDriveSafety() != SafetyState::SAFE)
    return;

  if (reverseRecoveryActive)
    return;

  if (driverState != DriverState::ACTIVE)
    return;

  if (now < reverseLockUntil_ms)
    return;

  // ---------------- TRUE CURRENT CHECK ----------------
  float curA_L = getMotorCurrentL();
  float curA_R = getMotorCurrentR();

  if (abs(curA_L) < 3.0f && abs(curA_R) < 3.0f)
    return;

  // ---------------- COMMAND CHECK ----------------
  if (abs(targetL) < 200 && abs(targetR) < 200)
    return;

  // ---------------- LOCK ----------------
  reverseLockUntil_ms = now + 1500;

  static uint32_t lastReverse_ms = 0;
  constexpr uint32_t REVERSE_COOLDOWN_MS = 1200;

  if (lastReverse_ms != 0 &&
      now - lastReverse_ms < REVERSE_COOLDOWN_MS)
    return;

  // ---------------- LIMIT ----------------
  if (autoReverseCount >= MAX_AUTO_REVERSE)
  {
    latchFault(FaultCode::OVER_CURRENT);
    return;
  }

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