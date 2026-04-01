// ============================================================================
// BluetoothTelemetry.cpp (SAFE + SYNC ANDROID 100%)
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
// 🔴 SAFE CLAMP (กันค่าพัง)
// ==============================
static float safeFloat(float v, float minV, float maxV) {

  if (isnan(v) || isinf(v)) return minV;

  if (v < minV) return minV;
  if (v > maxV) return maxV;

  return v;
}

// ==============================
// 🔴 TELEMETRY SEND
// ==============================
void btTelemetryUpdate(uint32_t now) {

  if (now - lastSend < TELEMETRY_PERIOD)
    return;

  lastSend = now;

  // ==================================================
  // 🔴 SANITY CHECK SENSOR (กันค่าพังทั้ง packet)
  // ==================================================
  if (isnan(engineVolt)) return;

  uint8_t packet[32];
  uint8_t idx = 0;

  // ==================================================
  // HEADER
  // ==================================================
  packet[idx++] = 0xAA;

  // reserve LEN
  uint8_t lenIndex = idx++;

  // ==================================================
  // TYPE + SEQ
  // ==================================================
  packet[idx++] = 0x01;     // TYPE = TELEMETRY
  packet[idx++] = seq++;    // SEQ

  // ==================================================
  // 🔴 FLAGS (bit mask)
  // ==================================================
  uint8_t flags = 0;

  if (systemState != SystemState::ACTIVE)
    flags |= 0x01;

  if (ibusCommLost)
    flags |= 0x02;

  if (engineRunning)
    flags |= 0x04;

  packet[idx++] = flags;

  // ==================================================
  // 🔴 ERROR CODE
  // ==================================================
  packet[idx++] = (uint8_t)getActiveFault();

  // ==================================================
  // 🔴 VOLTAGE (x100 + clamp)
  // ==================================================
  float voltSafe = safeFloat(engineVolt, 0.0f, 60.0f);
  int16_t v = (int16_t)(voltSafe * 100.0f);
  v = constrain(v, -32768, 32767);

  packet[idx++] = highByte(v);
  packet[idx++] = lowByte(v);

  // ==================================================
  // 🔴 CURRENT M1–M4 (x100 + clamp)
  // ==================================================
  for (int i = 0; i < 4; i++) {

    float curSafe = safeFloat(curA[i], 0.0f, 150.0f);

    int16_t c = (int16_t)(curSafe * 100.0f);
    c = constrain(c, -32768, 32767);

    packet[idx++] = highByte(c);
    packet[idx++] = lowByte(c);
  }

  // ==================================================
  // 🔴 TEMP LEFT (clamp)
  // ==================================================
  float tLSafe = safeFloat(tempDriverL, -40.0f, 150.0f);
  int16_t tL = (int16_t)tLSafe;

  packet[idx++] = highByte(tL);
  packet[idx++] = lowByte(tL);

  // ==================================================
  // 🔴 TEMP RIGHT (clamp)
  // ==================================================
  float tRSafe = safeFloat(tempDriverR, -40.0f, 150.0f);
  int16_t tR = (int16_t)tRSafe;

  packet[idx++] = highByte(tR);
  packet[idx++] = lowByte(tR);

  // ==================================================
  // LEN
  // ==================================================
  packet[lenIndex] = idx - 2;

  // ==================================================
  // CRC16
  // ==================================================
  uint16_t crc = crc16(packet, idx);

  // LOW → HIGH (ตรง Android)
  packet[idx++] = crc & 0xFF;
  packet[idx++] = (crc >> 8) & 0xFF;

  // ==================================================
  // SEND
  // ==================================================
  BT.write(packet, idx);
}

