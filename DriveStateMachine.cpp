// ============================================================================
// DriveStateMachine.cpp 
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "SystemTypes.h"
#include "HardwareConfig.h"

#include "SafetyManager.h"
#include "FaultManager.h"
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
  // SAFETY STATE
  // ==================================================
  SafetyState safety = getDriveSafety();

  // ==================================================
  // STATE MACHINE
  // ==================================================
  switch (driveState)
  {
    // --------------------------------------------------
    case DriveState::IDLE:

      targetL = 0;
      targetR = 0;

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

      // จำกัด target แทนการยุ่ง control
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

      // รอจน ramp + PID หยุดเอง
      if ((abs(curL) < 3 && abs(curR) < 3) ||
          (now - driveSoftStopStart_ms >= DRIVE_SOFT_STOP_TIMEOUT_MS))
      {
        driveSafe();
        driveState = DriveState::LOCKED;
      }

      break;

    // --------------------------------------------------
    case DriveState::LOCKED:

      targetL = 0;
      targetR = 0;

      driveSafe();
      break;

    // --------------------------------------------------
    default:

      latchFault(FaultCode::LOGIC_WATCHDOG);

      driveSafe();

      targetL = 0;
      targetR = 0;

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

