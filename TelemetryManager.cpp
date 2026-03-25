// ============================================================================
// TelemetryManager.cpp (BT READY + NON-SPAM + ECU SAFE)
// ============================================================================

#include "SystemTypes.h"
#include "GlobalState.h"
#include "HardwareConfig.h"
#include "TelemetryManager.h"
#include "SafetyManager.h"
#include "FaultManager.h"

// 🔴 ใช้ Bluetooth (HC-05)
#define TEL_SERIAL Serial2

#ifndef BUDGET_LOOP_MS
#define BUDGET_LOOP_MS 10
#endif

// ======================================================
// CRC8
// ======================================================

static uint8_t crc8_update(uint8_t crc, uint8_t data)
{
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc & 1) ? (crc >> 1) ^ 0x8C : (crc >> 1);
  return crc;
}

// ======================================================
// freeRam
// ======================================================

static int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

// ======================================================
// PACKET
// ======================================================

#pragma pack(push, 1)
struct TelemetryPacket
{
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

  uint8_t activeFault;
  uint16_t freeRam;
};
#pragma pack(pop)

// ======================================================
// CSV (BLUETOOTH)
// ======================================================

void telemetryCSV(uint32_t now, uint32_t loopStart_us)
{
#if TELEMETRY_CSV

  static uint32_t lastTx = 0;
  if (now - lastTx < TELEMETRY_PERIOD_MS) return;
  lastTx = now;

  TEL_SERIAL.print(now); TEL_SERIAL.print(",");

  TEL_SERIAL.print(curA[0], 2); TEL_SERIAL.print(",");
  TEL_SERIAL.print(curA[1], 2); TEL_SERIAL.print(",");

  TEL_SERIAL.print(engineVolt, 2); TEL_SERIAL.print(",");

  TEL_SERIAL.print(curL); TEL_SERIAL.print(",");
  TEL_SERIAL.print(curR); TEL_SERIAL.print(",");

  TEL_SERIAL.println((uint8_t)getActiveFault());

#endif
}

// ======================================================
// BINARY (BLUETOOTH)
// ======================================================

void telemetryBinary(uint32_t now, uint32_t loopStart_us)
{
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

  uint32_t loopTime = micros() - loopStart_us;
  int cpu = (int)((loopTime * 1000.0f) / (BUDGET_LOOP_MS * 1000UL));
  if (cpu > 9990) cpu = 9990;
  if (cpu < 0) cpu = 0;

  pkt.cpuLoad_x10 = cpu;

  pkt.activeFault = (uint8_t)getActiveFault();
  pkt.freeRam = freeRam();

  uint8_t crc = 0;
  uint8_t len = sizeof(pkt);

  const uint8_t* p = (const uint8_t*)&pkt;

  TEL_SERIAL.write(0xAA);
  TEL_SERIAL.write(0x55);

  TEL_SERIAL.write(len);
  crc = crc8_update(crc, len);

  for (uint8_t i = 0; i < len; i++)
  {
    TEL_SERIAL.write(p[i]);
    crc = crc8_update(crc, p[i]);
  }

  TEL_SERIAL.write(crc);

#endif
}

// ======================================================
// DEBUG (USB SERIAL ONLY)
// ======================================================

void debugTelemetry(uint32_t now)
{
#if DEBUG_SERIAL

  static uint32_t last = 0;
  if (now - last < 200) return;
  last = now;

  Serial.print(F("[TEL] V="));
  Serial.print(engineVolt, 1);
  Serial.print(F(" PWM="));
  Serial.print(curL);
  Serial.print(F("/"));
  Serial.print(curR);
  Serial.print(F(" F="));
  Serial.println((uint8_t)getActiveFault());

#endif
}

// ======================================================
// TEST
// ======================================================

void debugTestMode(uint32_t now)
{
#if TEST_MODE && DEBUG_SERIAL

  static uint32_t last = 0;
  if (now - last < 200) return;
  last = now;

  Serial.print(F("[TEST] PWM="));
  Serial.print(curL);
  Serial.print(F("/"));
  Serial.println(curR);

#endif
}

// ======================================================
// IBUS DEBUG
// ======================================================

void debugIBus(uint32_t now)
{
#if DEBUG_SERIAL

  static uint32_t last = 0;
  if (now - last < 200) return;
  last = now;

  Serial.print(F("[IBUS] "));
  Serial.print(rcThrottle);
  Serial.print(F(","));
  Serial.print(rcSteer);
  Serial.print(F(","));
  Serial.println(rcEngine);

#endif
}

