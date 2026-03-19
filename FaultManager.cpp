// ============================================================================
// FaultManager.cpp (FULL SYSTEM - FIXED COMPLETE VERSION)
// ============================================================================

#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>   // ✅ FIX: สำหรับ memset

#include "SystemTypes.h"
#include "GlobalState.h"
#include "HardwareConfig.h"
#include "FaultManager.h"
#include "SafetyManager.h"

// ======================================================
// CONFIG
// ======================================================

#define MAX_FAULT_CODES 10
#define FAULT_LOG_SIZE  20

#define EEPROM_BASE_A   100
#define EEPROM_BASE_B   400

// ======================================================
// STRUCT
// ======================================================

struct FaultBlock
{
  FaultRecord log[FAULT_LOG_SIZE];
  uint8_t head;
  uint8_t count;
  uint8_t crc;
};

// ======================================================
// CRC8
// ======================================================

static uint8_t crc8(const uint8_t* data, uint16_t len)
{
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}

// ======================================================
// STORAGE STATE
// ======================================================

static FaultBlock currentBlock;
static bool useSlotA = true;
static uint16_t faultCounter = 0;

// ======================================================
// LOAD FROM EEPROM
// ======================================================

static bool loadBlock(FaultBlock& out)
{
  FaultBlock a, b;

  EEPROM.get(EEPROM_BASE_A, a);
  EEPROM.get(EEPROM_BASE_B, b);

  bool va = (crc8((uint8_t*)&a, sizeof(FaultBlock) - 1) == a.crc);
  bool vb = (crc8((uint8_t*)&b, sizeof(FaultBlock) - 1) == b.crc);

  if (va && vb)
  {
    out = (a.head >= b.head) ? a : b;
    return true;
  }
  else if (va)
  {
    out = a;
    return true;
  }
  else if (vb)
  {
    out = b;
    return true;
  }

  memset(&out, 0, sizeof(FaultBlock));
  return false;
}

// ======================================================
// SAVE TO EEPROM
// ======================================================

static void saveBlock(FaultBlock& b)
{
  b.crc = crc8((uint8_t*)&b, sizeof(FaultBlock) - 1);

  if (useSlotA)
    EEPROM.put(EEPROM_BASE_A, b);
  else
    EEPROM.put(EEPROM_BASE_B, b);

  useSlotA = !useSlotA;
}

// ======================================================
// INIT (เรียกใน setup)
// ======================================================

void initFaultSystem()
{
  loadBlock(currentBlock);
}

// ======================================================
// ADD LOG (RING BUFFER)
// ======================================================

static void pushFaultLog(FaultCode code)
{
  FaultRecord& rec = currentBlock.log[currentBlock.head];

  rec.faultCode  = (uint8_t)code;
  rec.prevFault  = (uint8_t)activeFault;
  rec.engineVolt = (uint16_t)engineVolt;
  rec.throttle   = (int16_t)rcThrottle;
  rec.timestamp  = millis();
  rec.counter    = faultCounter++;

  currentBlock.head = (currentBlock.head + 1) % FAULT_LOG_SIZE;

  if (currentBlock.count < FAULT_LOG_SIZE)
    currentBlock.count++;

  saveBlock(currentBlock);
}

// ======================================================
// DEBOUNCE
// ======================================================

static uint32_t faultStartTime[MAX_FAULT_CODES] = {0};

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
    case FaultCode::LOGIC_WATCHDOG:       return 5;
    default: return 0;
  }
}

// ======================================================
// LATCH FAULT
// ======================================================

void latchFault(FaultCode code)
{
  if (code == FaultCode::NONE)
    return;

  uint8_t idx = (uint8_t)code;
  if (idx >= MAX_FAULT_CODES)
    return;

  if (faultStartTime[idx] == 0)
  {
    faultStartTime[idx] = millis();
    return;
  }

  if (millis() - faultStartTime[idx] < 300)
    return;

  faultStartTime[idx] = 0;

  if (faultLatched && activeFault == code)
    return;

  if (faultLatched)
  {
    if (getFaultPriority(code) <= getFaultPriority(activeFault))
      return;
  }

  pushFaultLog(code);

  activeFault = code;
  faultLatched = true;

#if DEBUG_SERIAL
  Serial.print(F("[FAULT] LATCH: "));
  Serial.println((uint8_t)code);
#endif
}

// ======================================================
// WATCHDOG
// ======================================================

constexpr uint16_t WD_GRACE_MS = 80;

void monitorSubsystemWatchdogs(uint32_t now)
{
  if (now - wdComms.lastUpdate_ms > wdComms.timeout_ms + WD_GRACE_MS)
    latchFault(FaultCode::IBUS_LOST);

  if (now - wdDrive.lastUpdate_ms > wdDrive.timeout_ms + WD_GRACE_MS)
    latchFault(FaultCode::DRIVE_TIMEOUT);

  if (now - wdBlade.lastUpdate_ms > wdBlade.timeout_ms + WD_GRACE_MS)
    latchFault(FaultCode::BLADE_TIMEOUT);
}

// ======================================================
// RESET VIA IGNITION (FIX LINKER ERROR)
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
// BACKGROUND TASK (FIX LINKER ERROR)
// ======================================================

void backgroundFaultEEPROMTask(uint32_t now)
{
  // เวอร์ชันนี้เขียน EEPROM ตอนเกิด fault แล้ว
  // ไม่ต้องทำอะไรใน background

  (void)now;
}

// ======================================================
// CLEAR
// ======================================================

void clearFault()
{
  if (!faultLatched)
    return;

  if (getFaultPriority(activeFault) >= 4)
  {
#if DEBUG_SERIAL
    Serial.println(F("[FAULT] CLEAR BLOCKED (CRITICAL)"));
#endif
    return;
  }

  faultLatched = false;
  activeFault = FaultCode::NONE;

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

