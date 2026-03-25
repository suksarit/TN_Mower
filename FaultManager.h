// ============================================================================
// FaultManager.h (ECU STYLE)
// ============================================================================

#pragma once

#include <Arduino.h>
#include "SystemTypes.h"

// ======================================================
// CORE API (ห้าม bypass)
// ======================================================
void initFaultSystem();
void latchFault(FaultCode code);
void requestFault(FaultCode code);  
void clearFault(void);
bool isFaultActive(void);
FaultCode getActiveFault(void);

// ======================================================
// CONTROL POLICY
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
// LOG
// ======================================================
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

