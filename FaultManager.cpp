// FaultManager.cpp

#include <Arduino.h>
#include <EEPROM.h>

#include "SystemTypes.h"
#include "GlobalState.h"
#include "HardwareConfig.h"
#include "FaultManager.h"
#include "SafetyManager.h"
#include "CommsManager.h"

void latchFault(FaultCode code) {

  if (faultLatched)
    return;

  activeFault = code;
  faultLatched = true;

  // --------------------------------------------------
  // Queue EEPROM write (background task will handle)
  // --------------------------------------------------
  faultToStore = code;
  faultWritePending = true;

#if DEBUG_SERIAL
  Serial.println(F("========== FAULT SNAPSHOT =========="));
  Serial.print(F("FaultCode="));
  Serial.println((uint8_t)code);
  Serial.print(F("SystemState="));
  Serial.println((uint8_t)systemState);
  Serial.print(F("DriveState="));
  Serial.println((uint8_t)driveState);
  Serial.print(F("BladeState="));
  Serial.println((uint8_t)bladeState);
  Serial.print(F("Safety="));
  Serial.println((uint8_t)getDriveSafety());

  Serial.print(F("CurA: "));
  for (int i = 0; i < 4; i++) {
    Serial.print(curA[i]);
    Serial.print(F(" "));
  }
  Serial.println();

  Serial.print(F("TempDriverL="));
  Serial.print(tempDriverL);
  Serial.print(F(" TempDriverR="));
  Serial.println(tempDriverR);

  Serial.print(F("PWM L/R="));
  Serial.print(curL);
  Serial.print(F("/"));
  Serial.println(curR);
  Serial.println(F("===================================="));
#endif
}





void backgroundFaultEEPROMTask(uint32_t now) {

  // --------------------------------------------------
  // NOTHING TO DO
  // --------------------------------------------------
  if (!faultWritePending)
    return;

  if (faultToStore == FaultCode::NONE) {
    faultWritePending = false;
    return;
  }

  // --------------------------------------------------
  // ANTI-WEAR LIMITS
  // --------------------------------------------------
  if (faultWriteCount >= MAX_FAULT_WRITES_PER_BOOT)
    return;

  if (now - lastFaultWriteMs < FAULT_EEPROM_COOLDOWN_MS)
    return;

  // --------------------------------------------------
  // READ LAST STORED VALUE (SAFE RAW READ)
  // --------------------------------------------------
  uint8_t raw;
  EEPROM.get(100, raw);

  FaultCode lastStored;

  if (!isValidEnum<FaultCode>(raw)) {
    // EEPROM corruption → treat as NONE
    lastStored = FaultCode::NONE;
  } else {
    lastStored = static_cast<FaultCode>(raw);
  }

  // --------------------------------------------------
  // VALIDATE CURRENT FAULT BEFORE WRITE
  // --------------------------------------------------
  uint8_t newRaw = static_cast<uint8_t>(faultToStore);

  if (!isValidEnum<FaultCode>(newRaw)) {
    // Corrupted RAM value → do NOT write garbage
    faultWritePending = false;
    return;
  }

  // --------------------------------------------------
  // WRITE ONLY IF CHANGED
  // --------------------------------------------------
  if (lastStored != faultToStore) {

    EEPROM.put(100, newRaw);

    faultWriteCount++;
    lastFaultWriteMs = now;

#if DEBUG_SERIAL
    Serial.print(F("[EEPROM] FAULT STORED: "));
    Serial.println(newRaw);
#endif
  }

  faultWritePending = false;
}




constexpr uint16_t WD_GRACE_MS = 40;

void monitorSubsystemWatchdogs(uint32_t now) {

  // --------------------------------------------------
  // SENSOR WATCHDOG (I2C RECOVERY SAFE)
  // --------------------------------------------------

  if (i2cState == I2CRecoverState::IDLE) {

    if (now - wdSensor.lastUpdate_ms > wdSensor.timeout_ms + WD_GRACE_MS) {

#if DEBUG_SERIAL
      Serial.println(F("[WD] SENSOR TIMEOUT"));
#endif

      latchFault(FaultCode::LOGIC_WATCHDOG);
      wdSensor.faulted = true;
    }

  } else {

    // I2C recovery running
    // pause sensor watchdog
  }

  // --------------------------------------------------
  // COMMS WATCHDOG
  // --------------------------------------------------

  if (now - wdComms.lastUpdate_ms > wdComms.timeout_ms + WD_GRACE_MS) {

#if DEBUG_SERIAL
    Serial.println(F("[WD] COMMS TIMEOUT"));
#endif

    latchFault(FaultCode::IBUS_LOST);
    wdComms.faulted = true;
  }

  // --------------------------------------------------
  // DRIVE WATCHDOG
  // --------------------------------------------------

  if (now - wdDrive.lastUpdate_ms > wdDrive.timeout_ms + WD_GRACE_MS) {

#if DEBUG_SERIAL
    Serial.println(F("[WD] DRIVE TIMEOUT"));
#endif

    latchFault(FaultCode::DRIVE_TIMEOUT);
    wdDrive.faulted = true;
  }

  // --------------------------------------------------
  // BLADE WATCHDOG
  // --------------------------------------------------

  if (now - wdBlade.lastUpdate_ms > wdBlade.timeout_ms + WD_GRACE_MS) {

#if DEBUG_SERIAL
    Serial.println(F("[WD] BLADE TIMEOUT"));
#endif

    latchFault(FaultCode::BLADE_TIMEOUT);
    wdBlade.faulted = true;
  }
}




void processFaultReset(uint32_t now) {
  static bool ignitionWasOn = false;
  static uint32_t ignitionOffStart_ms = 0;
  static bool ignitionOffQualified = false;

  if (!faultLatched)
    return;

  bool ignitionNow = ignitionActive;

  // --------------------------------------------------
  // DETECT IGNITION OFF PERIOD
  // --------------------------------------------------
  if (!ignitionNow) {
    if (ignitionWasOn)
      ignitionOffStart_ms = now;

    if (ignitionOffStart_ms != 0 && (now - ignitionOffStart_ms >= 3000)) {
      ignitionOffQualified = true;
    }
  }

  // --------------------------------------------------
  // SAFE CONDITIONS (USE RC CACHE)
  // --------------------------------------------------
  bool throttleNeutral = neutral(rcThrottle);
  bool steerNeutral = neutral(rcSteer);
  bool engineIdle = (rcEngine < 1100);

  bool driveIdle =
    (driveState == DriveState::IDLE || driveState == DriveState::LOCKED);

  bool bladeSafe =
    (bladeState == BladeState::IDLE || bladeState == BladeState::LOCKED);

  bool currentSafe = true;
  for (uint8_t i = 0; i < 4; i++)
    if (curA[i] > CUR_WARN_A)
      currentSafe = false;

  bool tempSafe =
    (tempDriverL < TEMP_WARN_C && tempDriverR < TEMP_WARN_C);

  bool voltSafe = (engineVolt > V_WARN_LOW);

  bool safetyClear =
    (getDriveSafety() != SafetyState::EMERGENCY);

  bool allSafe =
    throttleNeutral && steerNeutral && engineIdle && driveIdle && bladeSafe && currentSafe && tempSafe && voltSafe && safetyClear;

  // --------------------------------------------------
  // EXECUTE RESET ON RISING EDGE OF IGNITION
  // --------------------------------------------------
  if (!ignitionWasOn && ignitionNow) {
    if (ignitionOffQualified && allSafe) {
#if DEBUG_SERIAL
      Serial.println(F("[FAULT] RESET VIA IGNITION TOGGLE"));
#endif

      faultLatched = false;
      activeFault = FaultCode::NONE;

      systemState = SystemState::INIT;
      driveState = DriveState::IDLE;
      bladeState = BladeState::IDLE;
      forceSafetyState(SafetyState::SAFE);
    }

    ignitionOffQualified = false;
    ignitionOffStart_ms = 0;
  }

  ignitionWasOn = ignitionNow;
}



