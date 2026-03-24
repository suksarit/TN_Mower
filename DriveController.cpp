// ============================================================================
// DriveController.cpp (TORQUE TRUE CONTROL - FINAL FIX)
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "SystemTypes.h"
#include "HardwareConfig.h"

#include "DriveController.h"
#include "MotorDriver.h"
#include "SafetyManager.h"
#include "ThermalManager.h"
#include "AutoReverse.h"
#include "DriveProtection.h"
#include "CurrentController.h"
#include "DriveRamp.h"
#include "SensorManager.h"
#include "TractionControl.h"

// ============================================================================
// MAIN DRIVE PIPELINE
// ============================================================================
void applyDrive(uint32_t now)
{
  lastDriveEvent = DriveEvent::NONE;

  // ==================================================
  // SYSTEM GUARD
  // ==================================================
  if (driverState == DriverState::SETTLING)
  {
    curL = 0;
    curR = 0;
    targetL = 0;
    targetR = 0;

    resetCurrentLoop();
    resetDriveRamp();
    return;
  }

  // ==================================================
  // HARD FAULT
  // ==================================================
  if (systemState == SystemState::FAULT)
  {
    forceDriveSoftStop(now);
    return;
  }

  // ==================================================
  // BASE TARGET (TORQUE COMMAND)
  // ==================================================
  float finalTargetL = targetL;
  float finalTargetR = targetR;

  // ==================================================
  // AUTO REVERSE
  // ==================================================
  applyAutoReverse(finalTargetL, finalTargetR, now);

  // ==================================================
  // CURRENT READ
  // ==================================================
  float curA_L = getMotorCurrentSafeL();
  float curA_R = getMotorCurrentSafeR();

  // ==================================================
  // DETECTION
  // ==================================================
  detectWheelStuck(now);
  detectWheelLock();

  detectSideImbalanceAndSteer(
    finalTargetL,
    finalTargetR,
    curA_L,
    curA_R);

  // ==================================================
  // STALL SCALE
  // ==================================================
  float stallScale = computeStallScale(now, curA_L, curA_R);

  finalTargetL *= stallScale;
  finalTargetR *= stallScale;

  // ==================================================
  // DEADZONE
  // ==================================================
  if (fabs(finalTargetL) < 0.5f) finalTargetL = 0;
  if (fabs(finalTargetR) < 0.5f) finalTargetR = 0;

  // ==================================================
  // POWER LIMIT
  // ==================================================
  float powerScale = getPowerScale();

  finalTargetL *= powerScale;
  finalTargetR *= powerScale;

  // ==================================================
  // SAFETY SCALE
  // ==================================================
  SafetyState s = getDriveSafety();

  if (s == SafetyState::LIMP)
  {
    finalTargetL *= 0.4f;
    finalTargetR *= 0.4f;
  }
  else if (s == SafetyState::WARN)
  {
    finalTargetL *= 0.7f;
    finalTargetR *= 0.7f;
  }

  // ==================================================
  // DRIVE LIMITS
  // ==================================================
  applyDriveLimits(finalTargetL, finalTargetR, curA_L, curA_R);

  // ==================================================
  // LOAD COMP
  // ==================================================
  if (fabs(finalTargetL) > 2.0f || fabs(finalTargetR) > 2.0f)
  {
    static float filtL = 0;
    static float filtR = 0;

    filtL = filtL * 0.9f + curA_L * 0.1f;
    filtR = filtR * 0.9f + curA_R * 0.1f;

    constexpr float BASE_LOAD = 5.0f;
    constexpr float K_COMP = 0.1f;

    float compL = constrain((filtL - BASE_LOAD) * K_COMP, -5.0f, 5.0f);
    float compR = constrain((filtR - BASE_LOAD) * K_COMP, -5.0f, 5.0f);

    finalTargetL += compL;
    finalTargetR += compR;
  }

  // ==================================================
  // RAMP (ยังใช้ได้)
  // ==================================================
  updateDriveRamp(finalTargetL, finalTargetR);

  // ==================================================
  // 🔴 TORQUE TARGET = CURRENT TARGET (แก้หลัก)
  // ==================================================
  float targetCurrentL = finalTargetL;
  float targetCurrentR = finalTargetR;

  // ==================================================
  // TRACTION CONTROL
  // ==================================================
  applyTractionControl(
    targetCurrentL,
    targetCurrentR,
    curA_L,
    curA_R);

  // ==================================================
  // CURRENT PID
  // ==================================================
  applyCurrentPID(targetCurrentL, targetCurrentR);

  // ==================================================
  // CURRENT LIMIT (ADAPTIVE)
  // ==================================================
  const float CUR_LIMIT = 30.0f;
  const float CUR_HARD = 50.0f;

  if (curA_L > CUR_LIMIT)
    curL *= (CUR_LIMIT / curA_L);

  if (curA_R > CUR_LIMIT)
    curR *= (CUR_LIMIT / curA_R);

  if (curA_L > CUR_HARD) curL = 0;
  if (curA_R > CUR_HARD) curR = 0;

  // ==================================================
  // EMERGENCY
  // ==================================================
  if (s == SafetyState::EMERGENCY)
  {
    forceDriveSoftStop(now);
    return;
  }

  // ==================================================
  // FINAL SAFETY
  // ==================================================
  if (systemState != SystemState::ACTIVE ||
      driverState != DriverState::ACTIVE)
  {
    forceDriveSoftStop(now);
    return;
  }

  // ==================================================
  // KILL HANDLING
  // ==================================================
  if (killRequest == KillType::HARD)
    return;

  if (killRequest == KillType::SOFT)
    forceDriveSoftStop(now);

  // ==================================================
  // OUTPUT
  // ==================================================
  outputMotorPWM();

  wdDrive.lastUpdate_ms = now;
}

// ============================================================================
// FORCE SOFT STOP
// ============================================================================
void forceDriveSoftStop(uint32_t now)
{
  if (driveState != DriveState::SOFT_STOP)
  {
    driveState = DriveState::SOFT_STOP;
    driveSoftStopStart_ms = now;

    resetCurrentLoop();
    resetDriveRamp();
  }

  uint32_t dt = now - driveSoftStopStart_ms;

  float k = 1.0f - constrain(dt / 300.0f, 0.0f, 1.0f);

  curL *= k;
  curR *= k;

  if (abs(curL) < 3) curL = 0;
  if (abs(curR) < 3) curR = 0;
}

// ============================================================================
// CHECK COMMAND ZERO
// ============================================================================
bool driveCommandZero()
{
  return (abs(targetL) < 0.5f && abs(targetR) < 0.5f);
}

