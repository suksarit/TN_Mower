// ============================================================================
// DriveController.cpp (PRODUCTION SAFE - FINAL PIPELINE FIXED)
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

// ======================================================
// CORE STATE
// ======================================================
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

// ============================================================================
// MAIN DRIVE PIPELINE
// ============================================================================

void applyDrive(uint32_t now)
{
  // ==================================================
  // 🔴 RESET EVENT (สำคัญมาก)
  // ==================================================
  lastDriveEvent = DriveEvent::NONE;

  // ==================================================
  // 0. SYSTEM GUARD
  // ==================================================
  if (driverState == DriverState::SETTLING ||
      systemState != SystemState::ACTIVE)
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
  // 1. HARD FAULT
  // ==================================================
  if (systemState == SystemState::FAULT ||
      driveState == DriveState::LOCKED)
  {
    forceDriveSoftStop(now);

    curL = 0;
    curR = 0;
    return;
  }

  // ==================================================
  // 2. BASE TARGET
  // ==================================================
  float finalTargetL = (float)targetL;
  float finalTargetR = (float)targetR;

  // ==================================================
  // 3. AUTO REVERSE (ต้องมาก่อนทุกอย่าง)
  // ==================================================
  applyAutoReverse(finalTargetL, finalTargetR, now);

  // ==================================================
  // 4. CURRENT READ (single source)
  // ==================================================
  float curA_L = getMotorCurrentSafeL();
float curA_R = getMotorCurrentSafeR();

  // ==================================================
  // 5. DETECTION (ไม่แก้ cur)
  // ==================================================
  detectWheelStuck(now);
  detectWheelLock();

  detectSideImbalanceAndSteer(
    finalTargetL,
    finalTargetR,
    curA_L,
    curA_R
  );

  // ==================================================
  // 6. STALL SCALE
  // ==================================================
  float stallScale = computeStallScale(now, curA_L, curA_R);

  finalTargetL *= stallScale;
  finalTargetR *= stallScale;

  // ==================================================
  // 7. POWER LIMIT
  // ==================================================
  float powerScale = getPowerScale();

  finalTargetL *= powerScale;
  finalTargetR *= powerScale;

  // ==================================================
  // 8. DRIVE LIMITS (safety layer)
  // ==================================================
  applyDriveLimits(finalTargetL, finalTargetR, curA_L, curA_R);

  // ==================================================
  // 9. LOAD COMP (low priority)
  // ==================================================
  if (abs(finalTargetL) > 20 || abs(finalTargetR) > 20)
  {
    static float filtL = 0;
    static float filtR = 0;

    filtL = filtL * 0.9f + curA_L * 0.1f;
    filtR = filtR * 0.9f + curA_R * 0.1f;

    constexpr float BASE_LOAD = 5.0f;
    constexpr float K_COMP = 0.18f;

    float compL = constrain((filtL - BASE_LOAD) * K_COMP, -60, 60);
    float compR = constrain((filtR - BASE_LOAD) * K_COMP, -60, 60);

    finalTargetL += compL;
    finalTargetR += compR;
  }

  // ==================================================
  // 🔴 DEADZONE (ต้องมาก่อน current loop)
  // ==================================================
  if (abs(finalTargetL) < 6) finalTargetL = 0;
  if (abs(finalTargetR) < 6) finalTargetR = 0;

  // ==================================================
  // 🔴 HARD LIMIT (ก่อน control loop)
  // ==================================================
  finalTargetL = constrain(finalTargetL, -PWM_TOP, PWM_TOP);
  finalTargetR = constrain(finalTargetR, -PWM_TOP, PWM_TOP);

  // ==================================================
  // 10. CURRENT LOOP (final control)
  // ==================================================
  applyCurrentLoop(finalTargetL, finalTargetR);

  // ==================================================
  // 11. FINAL ARBITRATION
  // ==================================================
  bool forceStop = false;

  if (driveState == DriveState::LOCKED)
    forceStop = true;

  if (isThermalEmergency())
    forceStop = true;

  if (driverState != DriverState::ACTIVE)
    forceStop = true;

  if (forceStop)
  {
    finalTargetL = 0;
    finalTargetR = 0;

    resetCurrentLoop();
    resetDriveRamp();

    curL = 0;
    curR = 0;

    return;
  }

  // ==================================================
  // 12. RAMP (last stage only)
  // ==================================================
  updateDriveRamp(finalTargetL, finalTargetR);

  // ==================================================
  // 🔴 FINAL HARD SAFETY (last defense)
  // ==================================================
  if (systemState != SystemState::ACTIVE)
  {
    finalTargetL = 0;
    finalTargetR = 0;

    curL = 0;
    curR = 0;

    driveSafe();
    return;
  }

  // ==================================================
  // 13. OUTPUT
  // ==================================================
  outputMotorPWM();

  // ==================================================
  // 14. WATCHDOG
  // ==================================================
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

  curL *= 0.85f;
  curR *= 0.85f;

  if (abs(curL) < 5) curL = 0;
  if (abs(curR) < 5) curR = 0;
}

// ============================================================================
// CHECK COMMAND ZERO
// ============================================================================

bool driveCommandZero()
{
  return (abs(targetL) < 10 && abs(targetR) < 10);
}

