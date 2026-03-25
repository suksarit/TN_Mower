// ============================================================================
// EngineManager.cpp 
// ============================================================================

#include <Arduino.h>
#include <Servo.h>

#include "SystemTypes.h"
#include "GlobalState.h"
#include "HardwareConfig.h"
#include "EngineManager.h"
#include "DriveController.h"
#include "FaultManager.h"
#include "SafetyManager.h"

// ======================================================
// INTERNAL STATE
// ======================================================

static uint8_t startAttempt = 0;
static uint32_t lastStartAttempt_ms = 0;

// ============================================================================
// ENGINE RUN DETECTION
// ============================================================================

void updateEngineState(uint32_t now)
{
  static uint32_t runConfirm_ms = 0;
  static uint32_t stopConfirm_ms = 0;

  float v = engineVolt;

  if (starterActive)
  {
    runConfirm_ms = 0;
    stopConfirm_ms = 0;
    return;
  }

  if (!isfinite(v)) return;
  if (v < 10.0f || v > 40.0f) return;

  if (!engineRunning)
  {
    stopConfirm_ms = 0;

    if (v >= ENGINE_RUNNING_VOLT)
    {
      if (runConfirm_ms == 0)
        runConfirm_ms = now;

      else if (now - runConfirm_ms >= ENGINE_CONFIRM_MS)
      {
        engineRunning = true;
        runConfirm_ms = 0;
        startAttempt = 0;

#if DEBUG_SERIAL
        Serial.println(F("[ENGINE] RUN CONFIRMED"));
#endif
      }
    }
    else
    {
      runConfirm_ms = 0;
    }
  }
  else
  {
    runConfirm_ms = 0;

    if (v <= ENGINE_STOP_VOLT)
    {
      if (stopConfirm_ms == 0)
        stopConfirm_ms = now;

      else if (now - stopConfirm_ms >= ENGINE_CONFIRM_MS)
      {
        engineRunning = false;
        stopConfirm_ms = 0;
        engineStopped_ms = now;

#if DEBUG_SERIAL
        Serial.println(F("[ENGINE] STOP CONFIRMED"));
#endif
      }
    }
    else
    {
      stopConfirm_ms = 0;
    }
  }
}

// ============================================================================
// ENGINE THROTTLE
// ============================================================================

void updateEngineThrottle()
{
  if (systemState == SystemState::FAULT ||
      getDriveSafety() == SafetyState::EMERGENCY ||
      !engineRunning ||
      bladeState != BladeState::RUN)
  {
    bladeServo.writeMicroseconds(1000);
    return;
  }

  uint16_t ch3 = rcEngine;

  if (ch3 < 1000) ch3 = 1000;
  if (ch3 > 2000) ch3 = 2000;

  if (ch3 < 1050)
    ch3 = 1000;

  bladeServo.writeMicroseconds(ch3);
}

// ============================================================================
// STARTER CONTROL
// ============================================================================

void updateStarter(uint32_t now)
{
  uint16_t ch = rcStarter;

  bool requestStart = (ch > 1600);
  bool ignitionOn = ignitionActive;

  if (systemState == SystemState::FAULT ||
      driveState != DriveState::IDLE ||
      rcEngine > 1100 ||
      !neutral(rcThrottle) ||
      !ignitionOn ||
      engineRunning ||
      (engineStopped_ms != 0 &&
       now - engineStopped_ms < ENGINE_RESTART_GUARD_MS))
  {
    starterActive = false;
    digitalWrite(RELAY_STARTER, LOW);
    return;
  }

  if (starterActive)
  {
    if (!requestStart ||
        (now - starterStart_ms > STARTER_MAX_MS))
    {
      starterActive = false;
      digitalWrite(RELAY_STARTER, LOW);

      if (!engineRunning)
      {
        engineStopped_ms = now;

        if (++startAttempt >= 3)
        {
#if DEBUG_SERIAL
          Serial.println(F("[ENGINE] START FAIL"));
#endif
          requestFault(FaultCode::DRIVE_TIMEOUT);
        }
      }
    }
    return;
  }

  if (requestStart)
  {
    if (now - lastStartAttempt_ms < 1000)
      return;

    lastStartAttempt_ms = now;

    starterActive = true;
    starterStart_ms = now;

#if DEBUG_SERIAL
    Serial.println(F("[ENGINE] STARTER ON"));
#endif

    digitalWrite(RELAY_STARTER, HIGH);
  }
}

// ============================================================================
// IGNITION CONTROL
// ============================================================================

void updateIgnition()
{
  static bool ignitionLatched = false;
  static bool lastRaw = false;
  static uint32_t edgeStart_ms = 0;

  constexpr uint32_t DEBOUNCE_MS = 50;

  uint32_t now = millis();

  uint16_t ch = rcIgnition;
  bool raw = (ch > 1600);

  if (faultLatched || ibusCommLost)
  {
    ignitionLatched = false;
    ignitionActive = false;

    digitalWrite(RELAY_IGNITION, LOW);

    lastRaw = raw;
    edgeStart_ms = 0;
    return;
  }

  if (raw != lastRaw)
  {
    edgeStart_ms = now;
    lastRaw = raw;
  }

  if (edgeStart_ms != 0 &&
      (now - edgeStart_ms >= DEBOUNCE_MS))
  {
    ignitionLatched = raw;
    edgeStart_ms = 0;
  }

  ignitionActive = ignitionLatched;

  digitalWrite(RELAY_IGNITION,
               ignitionLatched ? HIGH : LOW);
}

// ============================================================================
// BLADE CONTROL
// ============================================================================

void runBlade(uint32_t now)
{
  switch (bladeState)
  {
    case BladeState::IDLE:

      if (systemState == SystemState::ACTIVE &&
          !faultLatched &&
          engineRunning)
      {
        bladeState = BladeState::RUN;
      }
      break;

    case BladeState::RUN:

      if (faultLatched ||
          getDriveSafety() == SafetyState::EMERGENCY ||
          !engineRunning)
      {
#if DEBUG_SERIAL
        Serial.println(F("[ENGINE] THROTTLE KILL"));
#endif
        bladeState = BladeState::SOFT_STOP;
        bladeSoftStopStart_ms = now;
      }
      break;

    case BladeState::SOFT_STOP:

      bladeServo.writeMicroseconds(1000);

      if (now - bladeSoftStopStart_ms >
          BLADE_SOFT_STOP_TIMEOUT_MS)
      {
        bladeState = BladeState::LOCKED;
      }
      break;

    case BladeState::LOCKED:

      bladeServo.writeMicroseconds(1000);
      break;
  }
}

