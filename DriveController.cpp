// ============================================================================
// DriveController.cpp (CLEAN VERSION)
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

// ======================================================
// MODULES (เฉพาะที่ใช้จริง)
// ======================================================
#include "DriveController.h"
#include "MotorDriver.h"
#include "SafetyManager.h"
#include "ThermalManager.h"

#include "AutoReverse.h"
#include "DriveProtection.h"
#include "CurrentController.h"
#include "DriveRamp.h"

// ============================================================================
// MAIN DRIVE PIPELINE
// ============================================================================

void applyDrive(uint32_t now)
{
  // --------------------------------------------------
  // DRIVER NOT ACTIVE
  // --------------------------------------------------
  if (driverState != DriverState::ACTIVE) {
    setPWM_L(0);
    setPWM_R(0);
    curL = 0;
    curR = 0;
    targetL = 0;
    targetR = 0;
    return;
  }

  // --------------------------------------------------
  // HARD STOP
  // --------------------------------------------------
  if (systemState == SystemState::FAULT ||
      driveState == DriveState::LOCKED) {
    driveSafe();
    curL = 0;
    curR = 0;
    targetL = 0;
    targetR = 0;
    return;
  }

  // --------------------------------------------------
  // DRIVER SETTLING
  // --------------------------------------------------
  if (driverState == DriverState::SETTLING) {
    setPWM_L(0);
    setPWM_R(0);
    curL = 0;
    curR = 0;
    return;
  }

  // --------------------------------------------------
  // SYSTEM NOT ACTIVE
  // --------------------------------------------------
  if (systemState != SystemState::ACTIVE) {
    driveSafe();
    curL = 0;
    curR = 0;
    targetL = 0;
    targetR = 0;
    return;
  }

  // ==================================================
  // TARGET BASE
  // ==================================================
  float finalTargetL = targetL;
  float finalTargetR = targetR;

  // ==================================================
  // AUTO REVERSE
  // ==================================================
  applyAutoReverse(finalTargetL, finalTargetR, now);

  // ==================================================
  // STALL PROTECTION
  // ==================================================
  float stallScale =
    computeStallScale(now, curLeft(), curRight());

  finalTargetL *= stallScale;
  finalTargetR *= stallScale;

  // ==================================================
  // CURRENT LOOP
  // ==================================================
  applyCurrentLoop(finalTargetL, finalTargetR);

  // ==================================================
  // POWER MANAGEMENT
  // ==================================================
  if (isThermalEmergency()) {
    finalTargetL = 0;
    finalTargetR = 0;
  } else {
    float scale = getPowerScale();
    finalTargetL *= scale;
    finalTargetR *= scale;
  }

  // ==================================================
  // FEEDFORWARD LOAD LIMIT
  // ==================================================
  float pwmLoad =
    (abs(finalTargetL) + abs(finalTargetR)) * 0.5f;

  if (pwmLoad > 600) {
    finalTargetL *= 0.8f;
    finalTargetR *= 0.8f;
  }

  // ==================================================
  // DRIVE LIMITS
  // ==================================================
  applyDriveLimits(
    finalTargetL,
    finalTargetR,
    curLeft(),
    curRight());

  // ==================================================
  // CLAMP
  // ==================================================
  finalTargetL =
    constrain(finalTargetL, -PWM_TOP, PWM_TOP);

  finalTargetR =
    constrain(finalTargetR, -PWM_TOP, PWM_TOP);

  // ==================================================
  // RAMP
  // ==================================================
  updateDriveRamp(finalTargetL, finalTargetR);

  // ==================================================
  // DEADZONE
  // ==================================================
  if (abs(curL) < 6) curL = 0;
  if (abs(curR) < 6) curR = 0;

  // ==================================================
  // OUTPUT
  // ==================================================
  outputMotorPWM();
}

// ============================================================================
// FORCE SOFT STOP
// ============================================================================

void forceDriveSoftStop(uint32_t now)
{
  if (driveState != DriveState::SOFT_STOP) {
    driveState = DriveState::SOFT_STOP;
    driveSoftStopStart_ms = now;
  }
}

// ============================================================================
// CHECK COMMAND ZERO
// ============================================================================

bool driveCommandZero()
{
  return (targetL == 0 && targetR == 0);
}