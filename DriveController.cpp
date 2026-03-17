// ============================================================================
// DriveController.cpp (FINAL - FIXED LOAD COMP + CLEAN ARBITRATION + CORRECT FLOW)
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

void applyDrive(uint32_t now) {

  // --------------------------------------------------
  // DRIVER NOT ACTIVE (SETTLING)
  // --------------------------------------------------
  if (driverState == DriverState::SETTLING) {
    curL = 0;
    curR = 0;
    return;
  }

  // --------------------------------------------------
  // HARD STOP
  // --------------------------------------------------
  if (systemState == SystemState::FAULT || driveState == DriveState::LOCKED) {
    curL = 0;
    curR = 0;
    targetL = 0;
    targetR = 0;
    return;
  }

  // --------------------------------------------------
  // SYSTEM NOT ACTIVE
  // --------------------------------------------------
  if (systemState != SystemState::ACTIVE) {
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
  // 🔴 LOAD COMPENSATION (FIXED)
  // ==================================================
  if (abs(finalTargetL) > 20 || abs(finalTargetR) > 20)
  {
    float curL_mA = getMotorCurrentL();
    float curR_mA = getMotorCurrentR();

    static float filtL = 0;
    static float filtR = 0;

    filtL = filtL * 0.8f + curL_mA * 0.2f;
    filtR = filtR * 0.8f + curR_mA * 0.2f;

    constexpr float BASE_LOAD = 5.0f;

    float loadErrL = filtL - BASE_LOAD;
    float loadErrR = filtR - BASE_LOAD;

    constexpr float K_COMP = 0.4f;

    finalTargetL += loadErrL * K_COMP;
    finalTargetR += loadErrR * K_COMP;
  }

  // ==================================================
  // POWER MANAGEMENT
  // ==================================================
  float scale = getPowerScale();
  finalTargetL *= scale;
  finalTargetR *= scale;

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
  finalTargetL = constrain(finalTargetL, -PWM_TOP, PWM_TOP);
  finalTargetR = constrain(finalTargetR, -PWM_TOP, PWM_TOP);

  // ==================================================
  // 🔴 ARBITRATION (CLEAN)
  // ==================================================

  // priority 1: LOCKED ONLY (systemState already handled above)
  if (driveState == DriveState::LOCKED) {
    finalTargetL = 0;
    finalTargetR = 0;
  }

  // priority 2: THERMAL
  else if (isThermalEmergency()) {
    finalTargetL = 0;
    finalTargetR = 0;
  }

  // priority 3: DRIVER NOT READY
  else if (driverState != DriverState::ACTIVE) {
    finalTargetL = 0;
    finalTargetR = 0;
  }

  // ==================================================
  // 🔴 DEADZONE (ต้องอยู่ก่อน RAMP)
  // ==================================================
  if (abs(finalTargetL) < 6) finalTargetL = 0;
  if (abs(finalTargetR) < 6) finalTargetR = 0;

  // ==================================================
  // RAMP (ตัวเดียวของระบบ)
  // ==================================================
  updateDriveRamp(finalTargetL, finalTargetR);

  // ==================================================
  // OUTPUT
  // ==================================================
  outputMotorPWM();
}

// ============================================================================
// FORCE SOFT STOP
// ============================================================================

void forceDriveSoftStop(uint32_t now) {
  if (driveState != DriveState::SOFT_STOP) {
    driveState = DriveState::SOFT_STOP;
    driveSoftStopStart_ms = now;
  }
}

// ============================================================================
// CHECK COMMAND ZERO
// ============================================================================

bool driveCommandZero() {
  return (targetL == 0 && targetR == 0);
}