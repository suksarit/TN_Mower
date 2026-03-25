// ============================================================================
// TelemetryManager.cpp 
// ============================================================================

#include "SystemTypes.h"
#include "GlobalState.h"
#include "HardwareConfig.h"
#include "TelemetryManager.h"
#include "SafetyManager.h"   // 🔴 เพิ่ม

// ============================================================================
// FALLBACK CONFIG
// ============================================================================

#ifndef BUDGET_LOOP_MS
#define BUDGET_LOOP_MS 10
#endif

// ============================================================================
// CRC8
// ============================================================================

static uint8_t crc8_update(uint8_t crc, uint8_t data) {

  crc ^= data;

  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x01)
      crc = (crc >> 1) ^ 0x8C;
    else
      crc >>= 1;
  }

  return crc;
}

// ============================================================================
// freeRam
// ============================================================================

static int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

// ============================================================================
// BINARY STRUCT
// ============================================================================

#pragma pack(push, 1)
struct TelemetryPacket {

  uint8_t  seq;
  uint8_t  type;

  uint32_t time_ms;

  int16_t tempL_c;
  int16_t tempR_c;

  int16_t volt_x10;
  int16_t curMax_x10;

  int16_t pwmL;
  int16_t pwmR;

  int16_t cpuLoad_x10;
  int16_t cpuMargin_x10;

  uint16_t ibusAge_ms;

  uint8_t flags;

  uint8_t activeFault;

  uint16_t freeRam;

  uint16_t autoReverseCount;

  uint8_t safetyState;

};
#pragma pack(pop)


// ============================================================================
// CSV TELEMETRY
// ============================================================================

void telemetryCSV(uint32_t now, uint32_t loopStart_us) {

#if TELEMETRY_CSV

  static uint32_t lastTx = 0;
  if (now - lastTx < TELEMETRY_PERIOD_MS) return;
  lastTx = now;

  static char data[256];
  uint8_t crc = 0;

  uint32_t loopTime_us = micros() - loopStart_us;
  uint32_t loopBudget_us = BUDGET_LOOP_MS * 1000UL;

  int cpuLoad_x10 = (int)((loopTime_us * 1000.0f) / loopBudget_us);
  if (cpuLoad_x10 > 9990) cpuLoad_x10 = 9990;
  if (cpuLoad_x10 < 0) cpuLoad_x10 = 0;

  float curMax =
    max(max(curA[0], curA[1]),
        max(curA[2], curA[3]));

  int curMax_x10 = (int)(curMax * 10.0f);
  int volt_x10 = (int)(engineVolt * 10.0f);

  // ==================================================
  // 🔴 SAFETY (REAL)
  // ==================================================

  SafetyInput sin;

  for (int i = 0; i < 4; i++)
    sin.curA[i] = curA[i];

  sin.tempDriverL = tempDriverL;
  sin.tempDriverR = tempDriverR;
  sin.faultLatched = faultLatched;
  sin.driveEvent = lastDriveEvent;

  SafetyThresholds sth = {
    CUR_WARN_A,
    CUR_LIMP_A,
    TEMP_WARN_C,
    TEMP_LIMP_C
  };

  SafetyState raw =
    evaluateSafetyRaw(sin, sth);

  updateSafetyStability(
    raw,
    now,
    autoReverseCount,
    autoReverseActive,
    lastDriveEvent);

  uint8_t rawSafety =
    (uint8_t)getDriveSafety();

  // ==================================================

  int len = snprintf(
    data,
    sizeof(data),
    "%lu,%d,%d,%d,%d,%u",
    now,
    tempDriverL,
    tempDriverR,
    volt_x10,
    curMax_x10,
    rawSafety);

  if (len <= 0 || len >= (int)sizeof(data)) return;

  for (int i = 0; i < len; i++)
    crc = crc8_update(crc, data[i]);

  Serial.write("$TN,");
  Serial.print(len);
  Serial.write(',');
  Serial.write(data, len);
  Serial.write(',');
  Serial.println(crc);

#endif
}


// ============================================================================
// BINARY TELEMETRY
// ============================================================================

void telemetryBinary(uint32_t now, uint32_t loopStart_us) {

#if TELEMETRY_BINARY

  static uint32_t lastTx = 0;
  static uint8_t seq = 0;

  if (now - lastTx < TELEMETRY_PERIOD_MS) return;
  lastTx = now;

  TelemetryPacket pkt;

  pkt.seq = seq++;
  pkt.type = 0;
  pkt.time_ms = now;

  pkt.tempL_c = tempDriverL;
  pkt.tempR_c = tempDriverR;

  pkt.volt_x10 = (int16_t)(engineVolt * 10.0f);

  float curMax =
    max(max(curA[0], curA[1]),
        max(curA[2], curA[3]));

  pkt.curMax_x10 = (int16_t)(curMax * 10.0f);

  pkt.pwmL = curL;
  pkt.pwmR = curR;

  uint32_t loopTime_us = micros() - loopStart_us;
  uint32_t loopBudget_us = BUDGET_LOOP_MS * 1000UL;

  int cpu = (int)((loopTime_us * 1000.0f) / loopBudget_us);
  if (cpu > 9990) cpu = 9990;
  if (cpu < 0) cpu = 0;

  pkt.cpuLoad_x10 = cpu;
  pkt.cpuMargin_x10 = 1000 - cpu;

  pkt.ibusAge_ms = now - lastIbusByte_ms;

  pkt.flags = 0;
  pkt.activeFault = (uint8_t)activeFault;
  pkt.freeRam = freeRam();
  pkt.autoReverseCount = autoReverseCount;

  // ==================================================
  // 🔴 SAFETY (REAL)
  // ==================================================

  SafetyInput sin;

  for (int i = 0; i < 4; i++)
    sin.curA[i] = curA[i];

  sin.tempDriverL = tempDriverL;
  sin.tempDriverR = tempDriverR;
  sin.faultLatched = faultLatched;
  sin.driveEvent = lastDriveEvent;

  SafetyThresholds sth = {
    CUR_WARN_A,
    CUR_LIMP_A,
    TEMP_WARN_C,
    TEMP_LIMP_C
  };

  SafetyState raw =
    evaluateSafetyRaw(sin, sth);

  updateSafetyStability(
    raw,
    now,
    autoReverseCount,
    autoReverseActive,
    lastDriveEvent);

  pkt.safetyState =
    (uint8_t)getDriveSafety();

  // ==================================================

  uint8_t crc = 0;
  uint8_t len = sizeof(TelemetryPacket);

  const uint8_t* p = (const uint8_t*)&pkt;

  Serial.write(0xAA);
  Serial.write(0x55);

  Serial.write(len);
  crc = crc8_update(crc, len);

  for (uint8_t i = 0; i < len; i++) {
    Serial.write(p[i]);
    crc = crc8_update(crc, p[i]);
  }

  Serial.write(crc);

#endif
}


// ============================================================================
// DEBUG TELEMETRY
// ============================================================================

void debugTelemetry(uint32_t now) {

#if DEBUG_SERIAL

  Serial.print(F("[TEL] "));
  Serial.print(now);
  Serial.print(F(" V="));
  Serial.print(engineVolt, 1);
  Serial.print(F(" PWM="));
  Serial.print(curL);
  Serial.print(F("/"));
  Serial.print(curR);

  Serial.print(F(" SAF="));
  Serial.println((uint8_t)getDriveSafety());

#endif
}


// ============================================================================
// TEST MODE
// ============================================================================

void debugTestMode(uint32_t now) {

#if TEST_MODE && DEBUG_SERIAL

  Serial.print(F("[TEST] "));
  Serial.print(now);
  Serial.print(F(" PWM="));
  Serial.print(curL);
  Serial.print(F("/"));
  Serial.println(curR);

#endif
}


// ============================================================================
// IBUS DEBUG
// ============================================================================

void debugIBus(uint32_t now) {

#if DEBUG_SERIAL

  Serial.print(F("[IBUS] "));
  Serial.print(F("THR="));
  Serial.print(rcThrottle);
  Serial.print(F(" STR="));
  Serial.print(rcSteer);
  Serial.print(F(" ENG="));
  Serial.println(rcEngine);

#endif
}

