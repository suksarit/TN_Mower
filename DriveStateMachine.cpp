// ============================================================================
// DriveStateMachine.cpp (REFACTORED - NO POWER COUPLING)
// ============================================================================

#include "CurrentController.h"

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "SystemTypes.h"
#include "HardwareConfig.h"
#include "SafetyManager.h"
#include "FaultManager.h"
#include "MotorDriver.h"
#include "SystemDegradation.h"

// 🔴 ใช้ killRequest จากระบบหลัก
extern KillType killRequest;

// ============================================================================
// MAIN STATE MACHINE
// ============================================================================
void runDrive(uint32_t now)
{
  static DriveState lastDriveState = DriveState::IDLE;
  static uint32_t limpSafeStart_ms = 0;

  // ==================================================
  // 🔴 PRIORITY: KILL
  // ==================================================
  if (killRequest == KillType::HARD)
  {
    targetL = 0;
    targetR = 0;

    driveSafe();
    driveState = DriveState::LOCKED;

    return;
  }

  if (killRequest == KillType::SOFT)
  {
    if (driveState != DriveState::SOFT_STOP &&
        driveState != DriveState::LOCKED)
    {
      driveState = DriveState::SOFT_STOP;
    }
  }

  // ==================================================
  // RUNAWAY DETECTION
  // ==================================================
  constexpr int16_t RUNAWAY_PWM_THRESHOLD = 120;
  constexpr uint32_t RUNAWAY_CONFIRM_MS = 200;

  static uint32_t runawayStart_ms = 0;

  bool commandZero =
    (fabs(targetL) < 0.05f && fabs(targetR) < 0.05f);

  bool motorMoving =
    (abs(curL) > RUNAWAY_PWM_THRESHOLD ||
     abs(curR) > RUNAWAY_PWM_THRESHOLD);

  if (commandZero && motorMoving)
  {
    if (runawayStart_ms == 0)
      runawayStart_ms = now;

    if (now - runawayStart_ms > RUNAWAY_CONFIRM_MS)
    {
      requestFault(FaultCode::DRIVE_TIMEOUT);
    }
  }
  else
  {
    runawayStart_ms = 0;
  }

  // ==================================================
  // SAFETY
  // ==================================================
  SafetyState safety = getDriveSafety();

  // ==================================================
  // SYSTEM MODE (ใช้แทน powerScale)
  // ==================================================
  SystemMode mode = getSystemMode();

  // ==================================================
  // STATE MACHINE
  // ==================================================
  switch (driveState)
  {
    // --------------------------------------------------
    case DriveState::IDLE:
    {
      targetL = 0;
      targetR = 0;

      limpSafeStart_ms = 0;
      driveSoftStopStart_ms = 0;

      resetCurrentLoop();

      if (systemState == SystemState::ACTIVE &&
          killRequest == KillType::NONE)
      {
        driveState = DriveState::RUN;
      }

      break;
    }

    // --------------------------------------------------
    case DriveState::RUN:
    {
      // 🔴 ใช้ SystemMode แทน scale
      if (mode == SystemMode::DEGRADED_L2 ||
          mode == SystemMode::FAULT)
      {
        driveState = DriveState::LIMP;
        limpSafeStart_ms = 0;
      }

      break;
    }

    // --------------------------------------------------
    case DriveState::LIMP:
    {
      // 🔴 จำกัด target (แต่ไม่คูณ scale ซ้ำ)
      const float limpLimit = 0.5f;

      targetL = constrain(targetL, -limpLimit, limpLimit);
      targetR = constrain(targetR, -limpLimit, limpLimit);

      if (safety == SafetyState::EMERGENCY)
      {
        driveState = DriveState::SOFT_STOP;
        break;
      }

      if (mode == SystemMode::NORMAL &&
          safety == SafetyState::SAFE)
      {
        if (limpSafeStart_ms == 0)
          limpSafeStart_ms = now;

        else if (now - limpSafeStart_ms >= LIMP_RECOVER_MS)
        {
          driveState = DriveState::RUN;
          limpSafeStart_ms = 0;
        }
      }
      else
      {
        limpSafeStart_ms = 0;
      }

      break;
    }

    // --------------------------------------------------
    case DriveState::SOFT_STOP:
    {
      targetL = 0;
      targetR = 0;

      if (driveSoftStopStart_ms == 0)
      {
        driveSoftStopStart_ms = now;
        resetCurrentLoop();
      }

      if ((abs(curL) < 3 && abs(curR) < 3) ||
          (now - driveSoftStopStart_ms >= DRIVE_SOFT_STOP_TIMEOUT_MS))
      {
        driveSafe();
        driveState = DriveState::LOCKED;
      }

      break;
    }

    // --------------------------------------------------
    case DriveState::LOCKED:
    {
      targetL = 0;
      targetR = 0;

      driveSafe();

      if (killRequest == KillType::NONE &&
          systemState == SystemState::ACTIVE &&
          safety == SafetyState::SAFE &&
          mode != SystemMode::FAULT)
      {
        driveState = DriveState::IDLE;
      }

      break;
    }

    // --------------------------------------------------
    default:
    {
      requestFault(FaultCode::LOGIC_WATCHDOG);

      driveSafe();
      targetL = 0;
      targetR = 0;

      driveState = DriveState::LOCKED;

      limpSafeStart_ms = 0;
      driveSoftStopStart_ms = 0;

      break;
    }
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

