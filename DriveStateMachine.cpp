// ============================================================================
// DriveStateMachine.cpp (FINAL - KILL SAFE + INDUSTRIAL)
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
  // 🔴 PRIORITY: KILL (สูงสุด)
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
#if DEBUG_SERIAL
      Serial.println(F("[DRIVE] RUNAWAY DETECTED"));
#endif
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
    }

    // --------------------------------------------------
    case DriveState::LIMP:
    {
      const float limpScale = 0.5f;

      targetL = constrain(targetL, -1.0f, 1.0f) * limpScale;
      targetR = constrain(targetR, -1.0f, 1.0f) * limpScale;

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

      // 🔴 รอ ramp ลงจริง
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

      // 🔴 ปลด lock เฉพาะกรณีปลอดภัยจริง
      if (killRequest == KillType::NONE &&
          systemState == SystemState::ACTIVE &&
          safety == SafetyState::SAFE)
      {
#if DEBUG_SERIAL
        Serial.println(F("[DRIVE] UNLOCK -> IDLE"));
#endif
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

