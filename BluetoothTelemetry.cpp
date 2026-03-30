// ============================================================================
// BluetoothTelemetry.cpp (FULL PROTOCOL READY - CRC16 + SEQ + TYPE)
// ============================================================================

#include "BluetoothTelemetry.h"
#include "GlobalState.h"
#include "FaultManager.h"
#include "SensorManager.h"

#define BT Serial2

// ==============================
// 🔴 Packet Sequence
// ==============================
static uint8_t seq = 0;

// ==============================
static uint32_t lastSend = 0;
static const uint32_t TELEMETRY_PERIOD = 200; // ms

// ==============================
// 🔴 CRC16 (MODBUS)
// ==============================
static uint16_t crc16(uint8_t *data, uint8_t len) {

  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < len; i++) {

    crc ^= data[i];

    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }

  return crc;
}

// ==============================
void btTelemetryInit() {
  // ไม่ต้อง begin ซ้ำ
}

// ==============================
// 🔴 TELEMETRY SEND
// ==============================
void btTelemetryUpdate(uint32_t now) {

  if (now - lastSend < TELEMETRY_PERIOD)
    return;

  lastSend = now;

  uint8_t packet[32];
  uint8_t idx = 0;

  // ==================================================
  // HEADER
  // ==================================================
  packet[idx++] = 0xAA;

  // reserve LEN
  uint8_t lenIndex = idx++;

  // ==================================================
  // TYPE + SEQ (FULL PROTOCOL)
  // ==================================================
  packet[idx++] = 0x01;     // TYPE = TELEMETRY
  packet[idx++] = seq++;    // SEQ

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
  // Current M1–M4
  for (int i = 0; i < 4; i++) {
    int16_t c = (int16_t)(curA[i] * 100);
    packet[idx++] = highByte(c);
    packet[idx++] = lowByte(c);
  }

  // Temp L
  int16_t tL = tempDriverL;
  packet[idx++] = highByte(tL);
  packet[idx++] = lowByte(tL);

  // Temp R
  int16_t tR = tempDriverR;
  packet[idx++] = highByte(tR);
  packet[idx++] = lowByte(tR);

  // ==================================================
  // LEN (DATA LENGTH)
  // ==================================================
  packet[lenIndex] = idx - 2;

  // ==================================================
  // CRC16
  // ==================================================
  uint16_t crc = crc16(packet, idx);

  // LOW → HIGH (สำคัญ)
  packet[idx++] = crc & 0xFF;
  packet[idx++] = (crc >> 8) & 0xFF;

  // ==================================================
  // SEND
  // ==================================================
  BT.write(packet, idx);
}

