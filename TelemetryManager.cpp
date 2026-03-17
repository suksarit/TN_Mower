// ============================================================================
// TelemetryManager.cpp
// ============================================================================

#include <Arduino.h>
#include <stdio.h>

#include "SystemTypes.h"
#include "GlobalState.h"
#include "TelemetryManager.h"

void telemetryCSV(uint32_t now, uint32_t loopStart_us) {

#if TELEMETRY_CSV

  static uint32_t lastTx = 0;

  if (now - lastTx < TELEMETRY_PERIOD_MS)
    return;

  lastTx = now;

  static char data[200];

  uint8_t crc = 0;

  // ==================================================
  // LOOP TIME
  // ==================================================

  uint32_t loopTime_us =
    micros() - loopStart_us;

  uint32_t loopBudget_us =
    BUDGET_LOOP_MS * 1000UL;

  int cpuLoad_x10 =
    (loopTime_us * 1000UL) / loopBudget_us;

  if (cpuLoad_x10 > 9990)
    cpuLoad_x10 = 9990;

  int cpuMargin_x10 =
    1000 - cpuLoad_x10;

  // ==================================================
  // CURRENT
  // ==================================================

  float curMax =
    max(max(curA[0], curA[1]),
        max(curA[2], curA[3]));

  int curMax_x10 =
    (int)(curMax * 10.0f);

  int volt_x10 =
    (int)(engineVolt * 10.0f);

  // ==================================================
  // WATCHDOG
  // ==================================================

  char wdS =
    wdSensor.faulted ? 'X' : 'O';

  char wdC =
    wdComms.faulted ? 'X' : 'O';

  char wdD =
    wdDrive.faulted ? 'X' : 'O';

  char wdB =
    wdBlade.faulted ? 'X' : 'O';

  char adsCur =
    adsCurPresent ? '1' : '0';

  char adsVolt =
    adsVoltPresent ? '1' : '0';

  char gimbalOn =
    (systemState == SystemState::ACTIVE &&
     !faultLatched &&
     !requireIbusConfirm) ? '1' : '0';

  // ==================================================
  // SAFETY
  // ==================================================

  SafetyInput sin;

  sin.curA[0] = curA[0];
  sin.curA[1] = curA[1];
  sin.curA[2] = curA[2];
  sin.curA[3] = curA[3];

  sin.tempDriverL = tempDriverL;
  sin.tempDriverR = tempDriverR;

  sin.faultLatched = faultLatched;
  sin.driveEvent = lastDriveEvent;

  SafetyThresholds sth =
  {
    CUR_WARN_A,
    CUR_LIMP_A,
    TEMP_WARN_C,
    TEMP_LIMP_C
  };

  SafetyState rawSafety =
    evaluateSafetyRaw(sin, sth);

  // ==================================================
  // BUILD CSV
  // ==================================================

  int len = snprintf(
    data,
    sizeof(data),
    "%lu,%d,%d,%d.%d,%d.%d,%d,%d,%d.%d,%d.%d,%lu,%c,%c,%c,%c,%u,%d,%u,%u,%c,%c,%c",
    now,
    tempDriverL,
    tempDriverR,
    volt_x10 / 10, abs(volt_x10 % 10),
    curMax_x10 / 10, abs(curMax_x10 % 10),
    curL,
    curR,
    cpuLoad_x10 / 10, abs(cpuLoad_x10 % 10),
    cpuMargin_x10 / 10, abs(cpuMargin_x10 % 10),
    now - lastIbusByte_ms,
    wdS,
    wdC,
    wdD,
    wdB,
    (uint8_t)activeFault,
    freeRam(),
    autoReverseCount,
    (uint8_t)rawSafety,
    adsCur,
    adsVolt,
    gimbalOn);

  // ==================================================
  // CRC
  // ==================================================

  char lenStr[6];

  itoa(len, lenStr, 10);

  for (uint8_t i = 0; lenStr[i]; i++)
    crc = crc8_update(crc, lenStr[i]);

  crc = crc8_update(crc, ',');

  for (int i = 0; i < len; i++)
    crc = crc8_update(crc, data[i]);

  // ==================================================
  // OUTPUT
  // ==================================================

  Serial.write("$TN,");
  Serial.print(len);
  Serial.write(',');
  Serial.write(data, len);
  Serial.write(',');
  Serial.println(crc);

#endif
}



// ============================================================================
// DEBUG TELEMETRY
// ============================================================================

void debugTelemetry(uint32_t now) {

#if DEBUG_SERIAL

  static uint32_t lastPrint = 0;

  if (now - lastPrint < 200)
    return;

  lastPrint = now;

  Serial.print(F("[TEL] "));
  Serial.print(F("S="));
  Serial.print((uint8_t)systemState);

  Serial.print(F(" D="));
  Serial.print((uint8_t)driveState);

  Serial.print(F(" B="));
  Serial.print((uint8_t)bladeState);

  Serial.print(F(" | IbusAge="));
  Serial.print(now - lastIbusByte_ms);

  Serial.print(F(" | Cur="));

  for (int i = 0; i < 4; i++) {

    Serial.print(curA[i]);
    Serial.print(F(" "));
  }

  Serial.print(F(" | TdL="));
  Serial.print(tempDriverL);

  Serial.print(F(" TdR="));
  Serial.print(tempDriverR);

  Serial.print(F(" | PWM="));
  Serial.print(curL);
  Serial.print(F("/"));
  Serial.println(curR);

  Serial.print(F(" | V24="));
  Serial.print(engineVolt, 1);

  Serial.print(F(" ER="));
  Serial.print(engineRunning ? 1 : 0);

#endif
}



// ============================================================================
// TEST MODE
// ============================================================================

void debugTestMode(uint32_t now) {

#if TEST_MODE && DEBUG_SERIAL

  static uint32_t lastPrint = 0;

  if (now - lastPrint < 500)
    return;

  lastPrint = now;

  Serial.print(F("[TEST MODE] "));

  Serial.print(F("Drive="));
  Serial.print((uint8_t)driveState);

  Serial.print(F(" Blade="));
  Serial.print((uint8_t)bladeState);

  Serial.print(F(" | PWM L/R="));
  Serial.print(curL);
  Serial.print(F("/"));
  Serial.print(curR);

  Serial.print(F(" | Cur(A)="));

  for (uint8_t i = 0; i < 4; i++) {

    Serial.print(curA[i], 1);
    Serial.print(F(" "));
  }

  Serial.print(tempDriverL);
  Serial.print(F("/"));
  Serial.print(tempDriverR);

  Serial.println();

#endif
}



// ============================================================================
// IBUS DEBUG
// ============================================================================

void debugIBus(uint32_t now) {

#if DEBUG_SERIAL

  static uint32_t lastPrint = 0;

  if (now - lastPrint < 500)
    return;

  lastPrint = now;

  Serial.print(F("[IBUS] "));

  Serial.print(F("THR="));
  Serial.print(rcThrottle);

  Serial.print(F(" STR="));
  Serial.print(rcSteer);

  Serial.print(F(" ENG="));
  Serial.print(rcEngine);

#endif
}


