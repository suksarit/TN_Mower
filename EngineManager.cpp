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

// ============================================================================
// ENGINE RUN DETECTION
// ============================================================================

void updateEngineState(uint32_t now) {

  static uint32_t runConfirm_ms = 0;
  static uint32_t stopConfirm_ms = 0;

  float v = engineVolt;

  // =================================================
  // LOCK DURING STARTER
  // =================================================

  if (starterActive) {

    runConfirm_ms = 0;
    stopConfirm_ms = 0;

    return;
  }

  // -------------------------------------------------
  // VOLTAGE PLAUSIBILITY
  // -------------------------------------------------

  if (!isfinite(v))
    return;

  if (engineVolt < 10.0f || engineVolt > 40.0f)
    return;

  // =================================================
  // ENGINE OFF → CHECK RUN
  // =================================================

  if (!engineRunning) {

    stopConfirm_ms = 0;

    if (v >= ENGINE_RUNNING_VOLT) {

      if (runConfirm_ms == 0) {

        runConfirm_ms = now;

      } else if (now - runConfirm_ms >= ENGINE_CONFIRM_MS) {

        engineRunning = true;

        runConfirm_ms = 0;

#if DEBUG_SERIAL
        Serial.println(F("[ENGINE] RUN CONFIRMED"));
#endif
      }

    } else {

      runConfirm_ms = 0;
    }
  }

  // =================================================
  // ENGINE ON → CHECK STOP
  // =================================================

  else {

    runConfirm_ms = 0;

    if (v <= ENGINE_STOP_VOLT) {

      if (stopConfirm_ms == 0) {

        stopConfirm_ms = now;

      } else if (now - stopConfirm_ms >= ENGINE_CONFIRM_MS) {

        engineRunning = false;

        stopConfirm_ms = 0;

        engineStopped_ms = now;

#if DEBUG_SERIAL
        Serial.println(F("[ENGINE] STOP CONFIRMED"));
#endif
      }

    } else {

      stopConfirm_ms = 0;
    }
  }
}

// ============================================================================
// ENGINE THROTTLE
// ============================================================================

void updateEngineThrottle() {

  if (systemState == SystemState::FAULT ||
      getDriveSafety() == SafetyState::EMERGENCY ||
      bladeState != BladeState::RUN) {

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

void updateStarter(uint32_t now) {

  uint16_t ch10 = rcStarter;

  bool requestStart = (ch10 > 1600);

  bool ignitionOn = ignitionActive;

  // =====================================================
  // HARD SAFETY GATE
  // =====================================================

  if (systemState == SystemState::FAULT ||
      driveState != DriveState::IDLE ||
      rcEngine > 1100 ||
      !neutral(rcThrottle) ||
      !ignitionOn ||
      engineRunning ||
      (engineStopped_ms != 0 &&
       now - engineStopped_ms < ENGINE_RESTART_GUARD_MS)) {

    starterActive = false;

    digitalWrite(RELAY_STARTER, LOW);

    return;
  }

  // =====================================================
  // STARTER ACTIVE
  // =====================================================

  if (starterActive) {

    if (!requestStart ||
        (now - starterStart_ms > STARTER_MAX_MS)) {

      starterActive = false;

      digitalWrite(RELAY_STARTER, LOW);

      if (!engineRunning)
        engineStopped_ms = now;
    }

    return;
  }

  // =====================================================
  // NEW START REQUEST
  // =====================================================

  if (requestStart) {

    starterActive = true;

    starterStart_ms = now;

    digitalWrite(RELAY_STARTER, HIGH);
  }
}

// ============================================================================
// IGNITION CONTROL
// ============================================================================

void updateIgnition() {

  static bool ignitionLatched = false;
  static bool lastRawRequest = false;
  static uint32_t edgeStart_ms = 0;

  constexpr uint32_t IGNITION_DEBOUNCE_MS = 50;

  uint32_t now = millis();

  uint16_t ch6 = rcIgnition;

  bool rawRequest = (ch6 > 1600);

  // --------------------------------------------------
  // HARD SAFETY
  // --------------------------------------------------

  if (faultLatched || ibusCommLost) {

    ignitionLatched = false;
    ignitionActive = false;

    digitalWrite(RELAY_IGNITION, LOW);

    lastRawRequest = rawRequest;
    edgeStart_ms = 0;

    return;
  }

  // --------------------------------------------------
  // EDGE DETECTION
  // --------------------------------------------------

  if (rawRequest != lastRawRequest) {

    edgeStart_ms = now;

    lastRawRequest = rawRequest;
  }

  // --------------------------------------------------
  // DEBOUNCE
  // --------------------------------------------------

  if (edgeStart_ms != 0 &&
      (now - edgeStart_ms >= IGNITION_DEBOUNCE_MS)) {

    ignitionLatched = rawRequest;

    edgeStart_ms = 0;
  }

  ignitionActive = ignitionLatched;

  digitalWrite(RELAY_IGNITION,
               ignitionLatched ? HIGH : LOW);
}

// ============================================================================
// BLADE CONTROL
// ============================================================================

void runBlade(uint32_t now) {

  switch (bladeState) {

    case BladeState::IDLE:

      if (systemState == SystemState::ACTIVE &&
          !faultLatched) {

        bladeState = BladeState::RUN;
      }

      break;

    case BladeState::RUN:

      if (faultLatched ||
          getDriveSafety() == SafetyState::EMERGENCY) {

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
          BLADE_SOFT_STOP_TIMEOUT_MS) {

        bladeState = BladeState::LOCKED;
      }

      break;

    case BladeState::LOCKED:

      bladeServo.writeMicroseconds(1000);

      break;
  }
}