// ============================================================================
// DriveStateMachine.cpp (FINAL - CLEAN STATE ONLY)
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

// ======================================================
// CORE STATE + TYPES
// ======================================================
#include "GlobalState.h"
#include "SystemTypes.h"
#include "HardwareConfig.h"

// ======================================================
// MODULES
// ======================================================
#include "SafetyManager.h"
#include "FaultManager.h"
#include "DriveRamp.h"
#include "MotorDriver.h"

// ============================================================================
// MAIN STATE MACHINE
// ============================================================================

void runDrive(uint32_t now)
{
  static DriveState lastDriveState = DriveState::IDLE;
  static uint32_t limpSafeStart_ms = 0;

  // ==================================================
  // MOTOR RUNAWAY DETECTION
  // ==================================================

  constexpr int16_t RUNAWAY_PWM_THRESHOLD = 120;
  constexpr uint32_t RUNAWAY_CONFIRM_MS = 200;

  static uint32_t runawayStart_ms = 0;

  bool commandZero = (targetL == 0 && targetR == 0);

  bool motorMoving =
    (abs(curL) > RUNAWAY_PWM_THRESHOLD ||
     abs(curR) > RUNAWAY_PWM_THRESHOLD);

  if (commandZero && motorMoving)
  {
    if (runawayStart_ms == 0)
      runawayStart_ms = now;

    if (now - runawayStart_ms > RUNAWAY_CONFIRM_MS)
    {
#if DEBUG_SERIAL
      Serial.println(F("[DRIVE] RUNAWAY DETECTED"));
#endif
      latchFault(FaultCode::DRIVE_TIMEOUT);
    }
  }
  else
  {
    runawayStart_ms = 0;
  }

  // ==================================================
  // CACHE SAFETY STATE
  // ==================================================

  SafetyState safety = getDriveSafety();

  // ==================================================
  // DRIVE STATE MACHINE
  // ==================================================

  switch (driveState)
  {
    // --------------------------------------------------
    case DriveState::IDLE:

      targetL = 0;
      targetR = 0;
      curL = 0;
      curR = 0;

      limpSafeStart_ms = 0;
      driveSoftStopStart_ms = 0;

      if (systemState == SystemState::ACTIVE)
      {
        driveState = DriveState::RUN;
      }

      break;

    // --------------------------------------------------
    case DriveState::RUN:

      if (safety == SafetyState::EMERGENCY)
      {
        driveState = DriveState::SOFT_STOP;
      }
      else if (safety == SafetyState::LIMP)
      {
        driveState = DriveState::LIMP;
        limpSafeStart_ms = 0;
      }

      break;

    // --------------------------------------------------
    case DriveState::LIMP:

      {
        float tmpL = targetL;
        float tmpR = targetR;

        updateDriveRamp(tmpL, tmpR);

        targetL = (int16_t)tmpL;
        targetR = (int16_t)tmpR;
      }

      // จำกัดกำลัง
      targetL /= 2;
      targetR /= 2;

      if (safety == SafetyState::EMERGENCY)
      {
        driveState = DriveState::SOFT_STOP;
        break;
      }

      if (safety == SafetyState::SAFE)
      {
        if (limpSafeStart_ms == 0)
          limpSafeStart_ms = now;

        else if (now - limpSafeStart_ms >= LIMP_RECOVER_MS)
        {
#if DEBUG_SERIAL
          Serial.println(F("[DRIVE] LIMP RECOVER -> RUN"));
#endif
          driveState = DriveState::RUN;
          limpSafeStart_ms = 0;
        }
      }
      else
      {
        limpSafeStart_ms = 0;
      }

      break;

    // --------------------------------------------------
    case DriveState::SOFT_STOP:

      targetL = 0;
      targetR = 0;

      if (driveSoftStopStart_ms == 0)
        driveSoftStopStart_ms = now;

      if ((curL == 0 && curR == 0) ||
          (now - driveSoftStopStart_ms >= DRIVE_SOFT_STOP_TIMEOUT_MS))
      {
        driveSafe();
        driveState = DriveState::LOCKED;
      }

      break;

    // --------------------------------------------------
    case DriveState::LOCKED:

      driveSafe();
      break;

    // --------------------------------------------------
    default:

      latchFault(FaultCode::LOGIC_WATCHDOG);

      driveSafe();

      targetL = 0;
      targetR = 0;
      curL = 0;
      curR = 0;

      driveState = DriveState::LOCKED;

      limpSafeStart_ms = 0;
      driveSoftStopStart_ms = 0;

      break;
  }

  // ==================================================
  // STATE CHANGE LOG
  // ==================================================

  if (driveState != lastDriveState)
  {
#if DEBUG_SERIAL
    Serial.print(F("[DRIVE STATE] "));
    Serial.print((uint8_t)lastDriveState);
    Serial.print(F(" -> "));
    Serial.println((uint8_t)driveState);
#endif

    if (driveState == DriveState::SOFT_STOP)
      driveSoftStopStart_ms = now;

    lastDriveState = driveState;
  }
}