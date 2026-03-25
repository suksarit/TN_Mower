// ============================================================================
// FaultManager.cpp (ECU STYLE - CONTROLLED)
// ============================================================================

#include <Arduino.h>
#include "FaultManager.h"
#include "GlobalState.h"
#include "SafetyManager.h"

extern KillType killRequest;

// ======================================================
// PRIORITY
// ======================================================

static uint8_t getFaultPriority(FaultCode code)
{
  switch (code)
  {
    case FaultCode::LOW_VOLTAGE_CRITICAL: return 1;
    case FaultCode::IBUS_LOST:            return 2;
    case FaultCode::DRIVE_TIMEOUT:        return 3;
    case FaultCode::BLADE_TIMEOUT:        return 3;
    case FaultCode::ENGINE_START_FAIL:    return 3;
    case FaultCode::SENSOR_TIMEOUT:       return 4;
    case FaultCode::RC_INVALID:           return 4;
    case FaultCode::LOGIC_WATCHDOG:       return 5;
    default: return 0;
  }
}

// ======================================================
// FAULT REQUEST (ENTRY POINT เดียว)
// ======================================================

void requestFault(FaultCode code)
{
  if (code == FaultCode::NONE)
    return;

  // 🔴 ถ้ามี fault อยู่แล้ว → ใช้ priority
  if (faultLatched)
  {
    if (getFaultPriority(code) >= getFaultPriority(activeFault))
      return;
  }

  activeFault = code;
  faultLatched = true;

  // ==================================================
  // ACTION
  // ==================================================
  killRequest = KillType::HARD;
  forceSafetyState(SafetyState::EMERGENCY);

#if DEBUG_SERIAL
  Serial.print(F("[FAULT] "));
  Serial.println((uint8_t)code);
#endif
}

// ======================================================
// CLEAR FAULT
// ======================================================

void clearFault(void)
{
  if (!faultLatched)
    return;

  // 🔴 ต้องปลอดภัยก่อน
  if (!neutral(rcThrottle) || !neutral(rcSteer))
    return;

  activeFault = FaultCode::NONE;
  faultLatched = false;

  killRequest = KillType::NONE;
  forceSafetyState(SafetyState::SAFE);

#if DEBUG_SERIAL
  Serial.println(F("[FAULT] CLEARED"));
#endif
}

// ======================================================
// QUERY
// ======================================================

bool isFaultActive(void)
{
  return faultLatched;
}

FaultCode getActiveFault(void)
{
  return activeFault;
}

bool canSystemRun(void)
{
  return !faultLatched;
}

bool isEmergency(void)
{
  return faultLatched;
}

// ======================================================
// WATCHDOG
// ======================================================

void monitorSubsystemWatchdogs(uint32_t now)
{
  constexpr uint16_t WD_GRACE = 120;

  if (now - wdComms.lastUpdate_ms > wdComms.timeout_ms + WD_GRACE)
    requestFault(FaultCode::IBUS_LOST);

  if (now - wdDrive.lastUpdate_ms > wdDrive.timeout_ms + WD_GRACE)
    requestFault(FaultCode::DRIVE_TIMEOUT);

  if (now - wdBlade.lastUpdate_ms > wdBlade.timeout_ms + WD_GRACE)
    requestFault(FaultCode::BLADE_TIMEOUT);

  if (now - wdSensor.lastUpdate_ms > wdSensor.timeout_ms + WD_GRACE)
    requestFault(FaultCode::SENSOR_TIMEOUT);
}

// ======================================================
// RESET FLOW (IGNITION BASED)
// ======================================================

void processFaultReset(uint32_t now)
{
  static bool ignitionWasOn = false;

  bool ign = ignitionActive;

  if (!ignitionWasOn && ign)
  {
    clearFault();
  }

  ignitionWasOn = ign;
}

// ======================================================
// BACKGROUND
// ======================================================

void backgroundFaultEEPROMTask(uint32_t now)
{
  (void)now;
}


