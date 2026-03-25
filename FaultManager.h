// ============================================================================
// FaultManager.h (ECU STYLE + HISTORY)
// ============================================================================

#pragma once

#include <Arduino.h>
#include "SystemTypes.h"

// ======================================================
// CORE
// ======================================================
void initFaultSystem();
void requestFault(FaultCode code);  
void clearFault(void);
bool isFaultActive(void);
FaultCode getActiveFault(void);

// ======================================================
// CONTROL
// ======================================================
bool canSystemRun(void);     
bool isEmergency(void);

// ======================================================
// WATCHDOG
// ======================================================
void monitorSubsystemWatchdogs(uint32_t now);

// ======================================================
// BACKGROUND
// ======================================================
void processFaultReset(uint32_t now);
void backgroundFaultEEPROMTask(uint32_t now);

// ======================================================
// LOG (ใหม่)
// ======================================================
uint8_t getFaultLogCount();

struct FaultRecord
{
  uint8_t  faultCode;
  uint32_t timestamp;
};

bool getFaultLog(uint8_t index, FaultRecord& out);

