// ============================================================================
// FaultManager.cpp (FINAL - ROBUST + RATE LIMIT + SENSOR SAFE)
// ============================================================================

#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>

#include "SystemTypes.h"
#include "GlobalState.h"
#include "HardwareConfig.h"
#include "FaultManager.h"
#include "SafetyManager.h"

extern KillType killRequest;

// ======================================================
// CONFIG
// ======================================================

#define MAX_FAULT_CODES 10
#define FAULT_LOG_SIZE  20

#define EEPROM_BASE_A   100
#define EEPROM_BASE_B   400

#define EEPROM_SAVE_DIVIDER 5

constexpr uint16_t WD_GRACE_MS = 120;
constexpr uint8_t  WD_CONFIRM_CNT = 3;
constexpr uint16_t FAULT_RATE_LIMIT_MS = 200;

// ======================================================
// FAULT SEVERITY
// ======================================================

enum class FaultSeverity : uint8_t
{
  NONE = 0,
  LIMP,
  FAULT
};

static FaultSeverity getFaultSeverity(FaultCode code)
{
  switch (code)
  {
    case FaultCode::LOW_VOLTAGE_CRITICAL:
    case FaultCode::IBUS_LOST:
    case FaultCode::DRIVE_TIMEOUT:
    case FaultCode::BLADE_TIMEOUT:
      return FaultSeverity::FAULT;

    case FaultCode::SENSOR_TIMEOUT:
      return FaultSeverity::LIMP;

    default:
      return FaultSeverity::NONE;
  }
}

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
// STORAGE
// ======================================================

static FaultBlock currentBlock;
static bool useSlotA = true;
static uint16_t faultCounter = 0;

// ======================================================
// EEPROM LOAD
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
  else if (va) return (out = a, true);
  else if (vb) return (out = b, true);

  memset(&out, 0, sizeof(FaultBlock));
  return false;
}

// ======================================================
// EEPROM SAVE
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
// INIT
// ======================================================

void initFaultSystem()
{
  loadBlock(currentBlock);
}

// ======================================================
// LOG
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

  static uint8_t saveDivider = 0;

  if (++saveDivider >= EEPROM_SAVE_DIVIDER)
  {
    saveBlock(currentBlock);
    saveDivider = 0;
  }
}

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
    case FaultCode::SENSOR_TIMEOUT:       return 4;
    case FaultCode::LOGIC_WATCHDOG:       return 5;
    default: return 0;
  }
}

// ======================================================
// DEBOUNCE
// ======================================================

static uint32_t faultStartTime[MAX_FAULT_CODES] = {0};

// ======================================================
// LATCH FAULT
// ======================================================

void latchFault(FaultCode code)
{
  if (code == FaultCode::NONE) return;

  uint8_t idx = (uint8_t)code;
  if (idx >= MAX_FAULT_CODES) return;

  uint32_t now = millis();

  static uint32_t lastFaultTime = 0;
  if (now - lastFaultTime < FAULT_RATE_LIMIT_MS) return;
  lastFaultTime = now;

  if (faultStartTime[idx] == 0)
  {
    faultStartTime[idx] = now;
    return;
  }

  uint32_t dt = now - faultStartTime[idx];

  if (dt < 300) return;

  if (dt > 1500)
  {
    faultStartTime[idx] = 0;
    return;
  }

  faultStartTime[idx] = 0;

  if (faultLatched)
  {
    if (activeFault == code) return;
    if (getFaultPriority(code) <= getFaultPriority(activeFault)) return;
  }

  pushFaultLog(code);

  activeFault = code;
  faultLatched = true;

  FaultSeverity sev = getFaultSeverity(code);

  if (sev == FaultSeverity::FAULT)
  {
    killRequest = KillType::HARD;
    forceSafetyState(SafetyState::EMERGENCY);
  }
  else if (sev == FaultSeverity::LIMP)
  {
    if (killRequest != KillType::HARD)
      killRequest = KillType::SOFT;

    forceSafetyState(SafetyState::LIMP);
  }
  else
  {
    if (killRequest != KillType::HARD)
      killRequest = KillType::SOFT;
  }
}

// ======================================================
// WATCHDOG
// ======================================================

void monitorSubsystemWatchdogs(uint32_t now)
{
  static uint8_t commsCnt = 0;
  static uint8_t driveCnt = 0;
  static uint8_t bladeCnt = 0;
  static uint8_t sensorCnt = 0;

  if (now - wdComms.lastUpdate_ms > wdComms.timeout_ms + WD_GRACE_MS)
  {
    if (++commsCnt >= WD_CONFIRM_CNT)
      latchFault(FaultCode::IBUS_LOST);
  } else commsCnt = 0;

  if (now - wdDrive.lastUpdate_ms > wdDrive.timeout_ms + WD_GRACE_MS)
  {
    if (++driveCnt >= WD_CONFIRM_CNT)
      latchFault(FaultCode::DRIVE_TIMEOUT);
  } else driveCnt = 0;

  if (now - wdBlade.lastUpdate_ms > wdBlade.timeout_ms + WD_GRACE_MS)
  {
    if (++bladeCnt >= WD_CONFIRM_CNT)
      latchFault(FaultCode::BLADE_TIMEOUT);
  } else bladeCnt = 0;

  if (now - wdSensor.lastUpdate_ms > wdSensor.timeout_ms + WD_GRACE_MS)
  {
    if (++sensorCnt >= WD_CONFIRM_CNT)
      latchFault(FaultCode::SENSOR_TIMEOUT);
  } else sensorCnt = 0;
}

// ======================================================
// 🔴 FIX ปัญหา: ฟังก์ชันที่หาย
// ======================================================

void processFaultReset(uint32_t now)
{
  static bool ignitionWasOn = false;
  static uint32_t ignitionOffStart = 0;
  static bool readyToReset = false;

  if (!faultLatched)
    return;

  bool ign = ignitionActive;

  if (!ign)
  {
    if (ignitionWasOn)
      ignitionOffStart = now;

    if (ignitionOffStart &&
        (now - ignitionOffStart > 3000))
    {
      readyToReset = true;
    }
  }

  bool safe =
    neutral(rcThrottle) &&
    neutral(rcSteer) &&
    (rcEngine < 1100);

  if (!ignitionWasOn && ign)
  {
    if (readyToReset && safe)
    {
      clearFault();
    }

    readyToReset = false;
    ignitionOffStart = 0;
  }

  ignitionWasOn = ign;
}

// ======================================================
// CLEAR
// ======================================================

void clearFault()
{
  if (!faultLatched) return;

  if (getFaultPriority(activeFault) >= 5)
    return;

  faultLatched = false;
  activeFault = FaultCode::NONE;

  killRequest = KillType::NONE;
  forceSafetyState(SafetyState::SAFE);
}

// ======================================================
// QUERY
// ======================================================

bool isFaultActive(void) { return faultLatched; }
FaultCode getActiveFault(void) { return activeFault; }
bool shouldStopSystem(void) { return faultLatched; }

// ======================================================
// BACKGROUND EEPROM TASK
// ======================================================

void backgroundFaultEEPROMTask(uint32_t now)
{
  (void)now;
}

