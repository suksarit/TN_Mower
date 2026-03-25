// ============================================================================
// FaultManager.h 
// ============================================================================

#pragma once

#include <Arduino.h>
#include "SystemTypes.h"

// ================= CORE =================
void initFaultSystem();
void latchFault(FaultCode code);
void clearFault(void);
void backgroundFaultEEPROMTask(uint32_t now);

// ================= WATCHDOG =================
void monitorSubsystemWatchdogs(uint32_t now);

// ================= QUERY =================
bool isFaultActive(void);
FaultCode getActiveFault(void);
bool shouldStopSystem(void);

// ================= LOG =================
uint8_t getFaultLogCount();

struct FaultRecord
{
  uint8_t  faultCode;
  uint8_t  prevFault;
  uint16_t engineVolt;
  int16_t  throttle;
  uint32_t timestamp;
  uint16_t counter;
};

bool getFaultLog(uint8_t index, FaultRecord& out); 


