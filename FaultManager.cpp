// ============================================================================
// FaultManager.cpp (FINAL - DEBOUNCE + PRIORITY + SAFE CLEAR + RECOVERY)
// ============================================================================

#include <Arduino.h>
#include <EEPROM.h>

#include "SystemTypes.h"
#include "GlobalState.h"
#include "HardwareConfig.h"
#include "FaultManager.h"
#include "SafetyManager.h"
#include "CommsManager.h"

// ======================================================
// LOCAL: FAULT PRIORITY
// ======================================================
static uint8_t getFaultPriority(FaultCode code)
{
  switch (code)
  {
    case FaultCode::LOW_VOLTAGE_CRITICAL: return 1;
    case FaultCode::IBUS_LOST:            return 2;
    case FaultCode::DRIVE_TIMEOUT:        return 3;
    case FaultCode::BLADE_TIMEOUT:        return 3;
    case FaultCode::LOGIC_WATCHDOG:       return 5;
    default: return 0;
  }
}

// ======================================================
// LOCAL: DEBOUNCE STORAGE
// ======================================================
static uint32_t faultStartTime[10] = {0}; // รองรับ fault สูงสุด 10 code

// ======================================================
// FUNCTION: latchFault (มี debounce)
// ======================================================
void latchFault(FaultCode code)
{
  if (code == FaultCode::NONE)
    return;

  uint8_t idx = (uint8_t)code;

  // --------------------------------------------------
  // DEBOUNCE (ต้องเกิดต่อเนื่องก่อน latch)
  // --------------------------------------------------
  if (faultStartTime[idx] == 0)
  {
    faultStartTime[idx] = millis();
    return;
  }

  if (millis() - faultStartTime[idx] < 300) // 300ms debounce
    return;

  // --------------------------------------------------
  // IGNORE SAME
  // --------------------------------------------------
  if (faultLatched && activeFault == code)
    return;

  // --------------------------------------------------
  // PRIORITY
  // --------------------------------------------------
  if (faultLatched)
  {
    if (getFaultPriority(code) <= getFaultPriority(activeFault))
      return;
  }

  activeFault = code;
  faultLatched = true;

  faultToStore = code;
  faultWritePending = true;

#if DEBUG_SERIAL
  Serial.print(F("[FAULT] LATCH: "));
  Serial.println((uint8_t)code);
#endif
}

// ======================================================
// EEPROM BACKGROUND TASK
// ======================================================
void backgroundFaultEEPROMTask(uint32_t now)
{
  if (!faultWritePending)
    return;

  if (faultToStore == FaultCode::NONE)
  {
    faultWritePending = false;
    return;
  }

  if (faultWriteCount >= MAX_FAULT_WRITES_PER_BOOT)
    return;

  if (now - lastFaultWriteMs < FAULT_EEPROM_COOLDOWN_MS)
    return;

  uint8_t raw;
  EEPROM.get(100, raw);

  FaultCode lastStored =
    isValidEnum<FaultCode>(raw) ?
    static_cast<FaultCode>(raw) :
    FaultCode::NONE;

  uint8_t newRaw = static_cast<uint8_t>(faultToStore);

  if (!isValidEnum<FaultCode>(newRaw))
  {
    faultWritePending = false;
    return;
  }

  if (lastStored != faultToStore)
  {
    EEPROM.put(100, newRaw);

    faultWriteCount++;
    lastFaultWriteMs = now;

#if DEBUG_SERIAL
    Serial.print(F("[EEPROM] STORED: "));
    Serial.println(newRaw);
#endif
  }

  faultWritePending = false;
}

// ======================================================
// WATCHDOG MONITOR (มี recovery)
// ======================================================
constexpr uint16_t WD_GRACE_MS = 40;

void monitorSubsystemWatchdogs(uint32_t now)
{
  // SENSOR
  if (i2cState == I2CRecoverState::IDLE)
  {
    if (now - wdSensor.lastUpdate_ms > wdSensor.timeout_ms + WD_GRACE_MS)
    {
      latchFault(FaultCode::LOGIC_WATCHDOG);
      wdSensor.faulted = true;
    }
    else
    {
      wdSensor.faulted = false; // recover ได้
    }
  }

  // COMMS
  if (now - wdComms.lastUpdate_ms > wdComms.timeout_ms + WD_GRACE_MS)
  {
    latchFault(FaultCode::IBUS_LOST);
    wdComms.faulted = true;
  }
  else
  {
    wdComms.faulted = false;
  }

  // DRIVE
  if (now - wdDrive.lastUpdate_ms > wdDrive.timeout_ms + WD_GRACE_MS)
  {
    latchFault(FaultCode::DRIVE_TIMEOUT);
    wdDrive.faulted = true;
  }
  else
  {
    wdDrive.faulted = false;
  }

  // BLADE
  if (now - wdBlade.lastUpdate_ms > wdBlade.timeout_ms + WD_GRACE_MS)
  {
    latchFault(FaultCode::BLADE_TIMEOUT);
    wdBlade.faulted = true;
  }
  else
  {
    wdBlade.faulted = false;
  }
}

// ======================================================
// RESET VIA IGNITION
// ======================================================
void processFaultReset(uint32_t now)
{
  static bool ignitionWasOn = false;
  static uint32_t ignitionOffStart_ms = 0;
  static bool ignitionOffQualified = false;

  if (!faultLatched)
    return;

  bool ignitionNow = ignitionActive;

  if (!ignitionNow)
  {
    if (ignitionWasOn)
      ignitionOffStart_ms = now;

    if (ignitionOffStart_ms != 0 &&
        (now - ignitionOffStart_ms >= 3000))
      ignitionOffQualified = true;
  }

  bool allSafe =
    neutral(rcThrottle) &&
    neutral(rcSteer) &&
    (rcEngine < 1100) &&
    (engineVolt > V_WARN_LOW);

  if (!ignitionWasOn && ignitionNow)
  {
    if (ignitionOffQualified && allSafe)
    {
      clearFault();
    }

    ignitionOffQualified = false;
    ignitionOffStart_ms = 0;
  }

  ignitionWasOn = ignitionNow;
}

// ======================================================
// CLEAR FAULT
// ======================================================
void clearFault()
{
  if (!faultLatched)
    return;

  if (getFaultPriority(activeFault) >= 4)
    return;

  faultLatched = false;
  activeFault = FaultCode::NONE;

  systemState = SystemState::INIT;
  driveState  = DriveState::IDLE;
  bladeState  = BladeState::IDLE;

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

bool shouldStopSystem(void)
{
  return faultLatched;
}