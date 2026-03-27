// ============================================================================
// BluetoothTelemetry.cpp (SEND DATA ONLY - FIXED)
// ============================================================================

#include "BluetoothTelemetry.h"
#include "GlobalState.h"
#include "FaultManager.h"
#include "SensorManager.h"

#define BT Serial2

// ==============================
static uint32_t lastSend = 0;
static const uint32_t TELEMETRY_PERIOD = 200; // ms

// ==============================
static String calcCRC(String data);

// ==============================
void btTelemetryInit() {
  // ไม่ต้อง begin ซ้ำ
}

// ==============================
// ==============================
void btTelemetryUpdate(uint32_t now) {

  if (now - lastSend < TELEMETRY_PERIOD)
    return;

  lastSend = now;

  uint8_t packet[24];
  uint8_t idx = 0;

  // ==================================================
  // HEADER
  // ==================================================
  packet[idx++] = 0xAA;

  // reserve LEN
  uint8_t lenIndex = idx++;
  
  // ==================================================
  // DATA
  // ==================================================

  // Fault
  packet[idx++] = (uint8_t)getActiveFault();

  // System state
  packet[idx++] = (uint8_t)systemState;

  // Voltage (x100)
  int16_t v = (int16_t)(engineVolt * 100);
  packet[idx++] = highByte(v);
  packet[idx++] = lowByte(v);

 // ===============================
// Current M1 (x100)
int16_t i1 = (int16_t)(curA[0] * 100);
packet[idx++] = highByte(i1);
packet[idx++] = lowByte(i1);

// Current M2 (x100)
int16_t i2 = (int16_t)(curA[1] * 100);
packet[idx++] = highByte(i2);
packet[idx++] = lowByte(i2);

// Current M3 (x100)
int16_t i3 = (int16_t)(curA[2] * 100);
packet[idx++] = highByte(i3);
packet[idx++] = lowByte(i3);

// Current M4 (x100)
int16_t i4 = (int16_t)(curA[3] * 100);
packet[idx++] = highByte(i4);
packet[idx++] = lowByte(i4);

  // Temp L
  int16_t tL = tempDriverL;
  packet[idx++] = highByte(tL);
  packet[idx++] = lowByte(tL);

  // Temp R
  int16_t tR = tempDriverR;
  packet[idx++] = highByte(tR);
  packet[idx++] = lowByte(tR);

  // ==================================================
  // LEN
  // ==================================================
  packet[lenIndex] = idx - 2; // DATA length

  // ==================================================
  // CRC (XOR)
  // ==================================================
  uint8_t crc = 0;
  for (uint8_t i = 0; i < idx; i++) {
    crc ^= packet[i];
  }

  packet[idx++] = crc;

  // ==================================================
  // SEND
  // ==================================================
  BT.write(packet, idx);
}

// ==============================
static String calcCRC(String data) {

  uint8_t crc = 0;

  for (size_t i = 0; i < data.length(); i++) {
    crc ^= (uint8_t)data[i];
  }

  char hex[3];
  sprintf(hex, "%02X", crc);

  return String(hex);
}

