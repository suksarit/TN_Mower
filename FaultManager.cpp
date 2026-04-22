// ============================================================================
// FaultManager.cpp (ECU STYLE + HISTORY + COOLDOWN)
// ============================================================================

#include <Arduino.h>

#include "FaultManager.h"
#include "GlobalState.h"
#include "SafetyManager.h"
#include "SystemDegradation.h"

extern KillType killRequest;

// ======================================================
// CONFIG
// ======================================================
constexpr uint8_t  FAULT_LOG_SIZE = 5;
constexpr uint32_t FAULT_COOLDOWN_MS = 1000;

// ======================================================
// INTERNAL
// ======================================================
static FaultRecord faultLog[FAULT_LOG_SIZE];
static uint8_t logIndex = 0;
static uint8_t logCount = 0;

static uint32_t lastFaultTime = 0;

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
    default: return 10;
  }
}

// ======================================================
// LOG STORE
// ======================================================
static void pushFaultLog(FaultCode code, uint32_t now)
{
  faultLog[logIndex].faultCode = (uint8_t)code;
  faultLog[logIndex].timestamp = now;

  logIndex = (logIndex + 1) % FAULT_LOG_SIZE;

  if (logCount < FAULT_LOG_SIZE)
    logCount++;
}

// ======================================================
// REQUEST FAULT
// ======================================================
void requestFault(FaultCode code)
{
  if (code == FaultCode::NONE)
    return;

  uint32_t now = millis();

  // 🔴 cooldown กัน spam
  if (now - lastFaultTime < FAULT_COOLDOWN_MS)
    return;

  // 🔴 priority
  if (faultLatched)
  {
    if (getFaultPriority(code) >= getFaultPriority(activeFault))
      return;
  }

  activeFault = code;
  faultLatched = true;
  lastFaultTime = now;

  // 🔴 log
  pushFaultLog(code, now);

  // 🔴 action
  switch (code)
{
  case FaultCode::SENSOR_TIMEOUT:
    setSystemMode(SystemMode::DEGRADED_L2);
    killRequest = KillType::SOFT;
    break;

  case FaultCode::LOW_VOLTAGE_CRITICAL:
    setSystemMode(SystemMode::DEGRADED_L2);
    killRequest = KillType::SOFT;
    break;

  case FaultCode::OVER_CURRENT:
    setSystemMode(SystemMode::FAULT);
    killRequest = KillType::HARD;
    break;

  default:
    setSystemMode(SystemMode::FAULT);
    killRequest = KillType::HARD;
}

  forceSafetyState(SafetyState::EMERGENCY);

#if DEBUG_SERIAL
  Serial.print(F("[FAULT SET] "));
  Serial.println(faultCodeToString(code));
#endif
}

// ======================================================
// CLEAR
// ======================================================
void clearFault(void)
{
  if (!faultLatched)
    return;

  if (!neutral(rcThrottle) || !neutral(rcSteer))
    return;

  activeFault = FaultCode::NONE;
  faultLatched = false;

  killRequest = KillType::NONE;
  forceSafetyState(SafetyState::SAFE);

#if DEBUG_SERIAL
  Serial.println(F("[FAULT CLEARED]"));
#endif
}

// ======================================================
// QUERY
// ======================================================
bool isFaultActive(void) { return faultLatched; }
FaultCode getActiveFault(void) { return activeFault; }
bool canSystemRun(void) { return !faultLatched; }
bool isEmergency(void) { return faultLatched; }

// ======================================================
// WATCHDOG (เหมือนเดิม)
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
// RESET
// ======================================================
void processFaultReset(uint32_t now)
{
  static bool ignitionWasOn = false;
  bool ign = ignitionActive;

  if (!ignitionWasOn && ign)
    clearFault();

  ignitionWasOn = ign;
}

// ======================================================
// LOG API
// ======================================================
uint8_t getFaultLogCount()
{
  return logCount;
}

bool getFaultLog(uint8_t index, FaultRecord& out)
{
  if (index >= logCount)
    return false;

  out = faultLog[index];
  return true;
}

// ======================================================
// BACKGROUND
// ======================================================
void backgroundFaultEEPROMTask(uint32_t now)
{
  (void)now;
}

