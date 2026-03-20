// ============================================================================
// DriveController.cpp (TORQUE CONTROL + PID + NO FIGHT)
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

void applyDrive(uint32_t now) {

  // ==================================================
  // RESET EVENT
  // ==================================================
  lastDriveEvent = DriveEvent::NONE;

  // ==================================================
  // SYSTEM GUARD
  // ==================================================
  if (driverState == DriverState::SETTLING) {
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
  if (systemState == SystemState::FAULT || driveState == DriveState::LOCKED) {
    forceDriveSoftStop(now);
    curL = 0;
    curR = 0;
    return;
  }

  // ==================================================
  // BASE TARGET
  // ==================================================
  float finalTargetL = (float)targetL;
  float finalTargetR = (float)targetR;

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
  if (abs(finalTargetL) < 6) finalTargetL = 0;
  if (abs(finalTargetR) < 6) finalTargetR = 0;

  // ==================================================
  // POWER LIMIT
  // ==================================================
  float powerScale = getPowerScale();

  finalTargetL *= powerScale;
  finalTargetR *= powerScale;

  // ==================================================
  // 🔴 SAFETY SCALE (เพิ่ม)
  // ==================================================
  SafetyState s = getDriveSafety();

  if (s == SafetyState::LIMP) {
    finalTargetL *= 0.4f;
    finalTargetR *= 0.4f;
  }
  else if (s == SafetyState::WARN) {
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
  if (abs(finalTargetL) > 20 || abs(finalTargetR) > 20) {
    static float filtL = 0;
    static float filtR = 0;

    filtL = filtL * 0.9f + curA_L * 0.1f;
    filtR = filtR * 0.9f + curA_R * 0.1f;

    constexpr float BASE_LOAD = 5.0f;
    constexpr float K_COMP = 0.10f;

    float compL = constrain((filtL - BASE_LOAD) * K_COMP, -60, 60);
    float compR = constrain((filtR - BASE_LOAD) * K_COMP, -60, 60);

    finalTargetL += compL;
    finalTargetR += compR;
  }

  // ==================================================
  // HARD LIMIT
  // ==================================================
  finalTargetL = constrain(finalTargetL, -PWM_TOP, PWM_TOP);
  finalTargetR = constrain(finalTargetR, -PWM_TOP, PWM_TOP);

  // ==================================================
  // RAMP
  // ==================================================
  updateDriveRamp(finalTargetL, finalTargetR);

  // ==================================================
  // TORQUE TARGET
  // ==================================================
  float targetCurrentL = finalTargetL * 0.02f;
  float targetCurrentR = finalTargetR * 0.02f;

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
  // 🔴 CURRENT LIMIT ADAPTIVE (เพิ่ม)
  // ==================================================
  const float CUR_LIMIT = 30.0f;
  const float CUR_HARD  = 50.0f;

  if (curA_L > CUR_LIMIT) {
    float scale = CUR_LIMIT / curA_L;
    curL *= scale;
  }

  if (curA_R > CUR_LIMIT) {
    float scale = CUR_LIMIT / curA_R;
    curR *= scale;
  }

  if (curA_L > CUR_HARD) curL = 0;
  if (curA_R > CUR_HARD) curR = 0;

  // ==================================================
  // 🔴 EMERGENCY SOFT STOP (เพิ่ม)
  // ==================================================
  if (s == SafetyState::EMERGENCY) {
    forceDriveSoftStop(now);
    return;
  }

  // ==================================================
  // FINAL HARD SAFETY
  // ==================================================
  if (systemState != SystemState::ACTIVE ||
      driveState == DriveState::LOCKED ||
      driverState != DriverState::ACTIVE) {
    curL = 0;
    curR = 0;

    driveSafe();
    return;
  }

  // ==================================================
  // OUTPUT
  // ==================================================
  outputMotorPWM();

  // ==================================================
  // WATCHDOG
  // ==================================================
  wdDrive.lastUpdate_ms = now;
}

// ============================================================================
// FORCE SOFT STOP
// ============================================================================

void forceDriveSoftStop(uint32_t now) {
  if (driveState != DriveState::SOFT_STOP) {
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

bool driveCommandZero() {
  return (abs(targetL) < 10 && abs(targetR) < 10);
}

