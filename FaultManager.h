// ============================================================================
// FaultManager.h (FINAL - COMPLETE INTERFACE)
// ============================================================================

#pragma once

#include <Arduino.h>
#include "SystemTypes.h"

// ======================================================
// CORE FAULT CONTROL
// ======================================================

// ยิง fault (มี priority + latch)
void latchFault(FaultCode code);

// clear fault (มีเงื่อนไข ปลอดภัยเท่านั้น)
void clearFault(void);

// ======================================================
// BACKGROUND TASK
// ======================================================

// เขียน EEPROM แบบ background (เรียกใน loop หลัก)
void backgroundFaultEEPROMTask(uint32_t now);

// ======================================================
// WATCHDOG MONITOR
// ======================================================

// ตรวจ watchdog ทุก subsystem
void monitorSubsystemWatchdogs(uint32_t now);

// ======================================================
// FAULT RESET LOGIC
// ======================================================

// reset fault ผ่าน ignition logic
void processFaultReset(uint32_t now);

// ======================================================
// QUERY (ห้ามอ่าน global ตรง)
// ======================================================

// มี fault อยู่หรือไม่
bool isFaultActive(void);

// fault ปัจจุบันคืออะไร
FaultCode getActiveFault(void);

// ======================================================
// UTILITY (ใช้ใน SystemGate)
// ======================================================

// ควรหยุดระบบหรือไม่ (ใช้ใน main loop)
bool shouldStopSystem(void);