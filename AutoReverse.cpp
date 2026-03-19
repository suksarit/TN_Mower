// ============================================================================
// AutoReverse.cpp (PRODUCTION SAFE - NO JERK + STABLE + LOCKED DIRECTION)
// ============================================================================

#include <Arduino.h>

#include "AutoReverse.h"
#include "GlobalState.h"
#include "SafetyManager.h"
#include "FaultManager.h"
#include "HardwareConfig.h"
#include "SensorManager.h"
#include "DriveProtection.h"   // 🔴 ใช้ setDriveEvent()

// ======================================================
// LOCAL STATE
// ======================================================

static uint32_t autoReverseStart_ms = 0;
static bool reverseRecoveryActive = false;
static uint32_t reverseLockUntil_ms = 0;

// 🔴 lock direction ระหว่าง reverse
static int8_t revDirL = 0;
static int8_t revDirR = 0;

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
    // ---------------- HARD SAFETY EXIT ----------------
    if (ibusCommLost || requireIbusConfirm ||
        systemState != SystemState::ACTIVE)
    {
      autoRev.active = false;
      autoReverseActive = false;
      return;
    }

    // ---------------- CURRENT CHECK ----------------
    float curA_L = getMotorCurrentL();
    float curA_R = getMotorCurrentR();

    // โหลดหาย = ยกเลิก
    if (curA_L < 2.0f && curA_R < 2.0f)
    {
      autoRev.active = false;
      autoReverseActive = false;
      return;
    }

    // ---------------- RUNNING ----------------
    if (now - autoRev.start_ms < autoRev.duration_ms)
    {
      finalTargetL = constrain(revDirL * autoRev.pwm, -PWM_TOP, PWM_TOP);
      finalTargetR = constrain(revDirR * autoRev.pwm, -PWM_TOP, PWM_TOP);
      return;
    }

    // ---------------- FINISHED ----------------
    autoRev.active = false;
    autoReverseActive = false;

    reverseRecoveryActive = true;
    autoReverseStart_ms = now;
  }

  // ==================================================
  // RECOVERY WINDOW (soft output)
  // ==================================================

  if (reverseRecoveryActive)
  {
    if (now - autoReverseStart_ms < REVERSE_RECOVERY_MS)
    {
      finalTargetL *= 0.6f;  // 🔴 ลดแรงลงอีก
      finalTargetR *= 0.6f;
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
  // ==================================================
  // HARD SAFETY GUARD
  // ==================================================

  if (getDriveSafety() != SafetyState::SAFE)
    return;

  if (reverseRecoveryActive)
    return;

  if (driverState != DriverState::ACTIVE)
    return;

  if (now < reverseLockUntil_ms)
    return;

  // 🔴 FIX: ห้าม reverse ถ้ายังวิ่งเร็ว
  if (abs(curL) > 120 || abs(curR) > 120)
    return;

  // ==================================================
  // CURRENT CHECK
  // ==================================================
  float curA_L = getMotorCurrentL();
  float curA_R = getMotorCurrentR();

  if (curA_L < 3.0f && curA_R < 3.0f)
    return;

  // ==================================================
  // COMMAND CHECK
  // ==================================================
  if (abs(targetL) < 200 && abs(targetR) < 200)
    return;

  // ==================================================
  // COOLDOWN
  // ==================================================
  static uint32_t lastReverse_ms = 0;
  constexpr uint32_t REVERSE_COOLDOWN_MS = 1200;

  if (lastReverse_ms != 0 &&
      now - lastReverse_ms < REVERSE_COOLDOWN_MS)
    return;

  // ==================================================
  // LIMIT
  // ==================================================
  if (autoReverseCount >= MAX_AUTO_REVERSE)
  {
    latchFault(FaultCode::OVER_CURRENT);
    return;
  }

  // ==================================================
  // PARAM ESCALATION
  // ==================================================
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

  // ==================================================
  // 🔴 LOCK DIRECTION (สำคัญมาก)
  // ==================================================
  revDirL = (curL > 0) ? -1 : 1;
  revDirR = (curR > 0) ? -1 : 1;

  // ==================================================
  // START
  // ==================================================
  autoRev.active = true;
  autoRev.start_ms = now;
  autoRev.duration_ms = reverseTime;
  autoRev.pwm = reversePWM;

  autoReverseActive = true;
  autoReverseStart_ms = now;

  autoReverseCount++;
  lastReverse_ms = now;

  // 🔴 FIX: ใช้ safe event
  setDriveEvent(DriveEvent::AUTO_REVERSE);
}

