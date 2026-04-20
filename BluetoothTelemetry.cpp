// ============================================================================
// BluetoothTelemetry.cpp (FINAL - CLEAN + MATCH ANDROID 100%)
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
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
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
  // 🔴 SANITY CHECK SENSOR (ห้ามหยุดส่ง packet)
  // ==================================================
  float voltLocal = engineVolt;

  if (isnan(voltLocal) || isinf(voltLocal)) {
    voltLocal = 0.0f;   // 🔴 ยังส่ง packetต่อ
  }

  // 🔴 เผื่อ buffer
  uint8_t packet[40];
  uint8_t idx = 0;

  // ==================================================
  // HEADER
  // ==================================================
  packet[idx++] = 0xAA;
  packet[idx++] = 0x55;

  uint8_t lenIndex = idx++;

  // ==================================================
  // 🔴 SEQ (ไม่มี TYPE แล้ว)
  // ==================================================
  packet[idx++] = seq++;

  // ==================================================
  // FLAGS
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
  // ERROR
  // ==================================================
  packet[idx++] = (uint8_t)getActiveFault();

  // ==================================================
  // VOLTAGE (x100)
  // ==================================================
  float voltSafe = safeFloat(voltLocal, 0.0f, 60.0f);
  int16_t v = (int16_t)(voltSafe * 100.0f);
  v = constrain(v, -32768, 32767);

  packet[idx++] = highByte(v);
  packet[idx++] = lowByte(v);

  // ==================================================
  // CURRENT M1–M4 (x100)
  // ==================================================
  for (int i = 0; i < 4; i++) {

    float curSafe = safeFloat(curA[i], 0.0f, 150.0f);

    int16_t c = (int16_t)(curSafe * 100.0f);
    c = constrain(c, -32768, 32767);

    packet[idx++] = highByte(c);
    packet[idx++] = lowByte(c);
  }

  // ==================================================
  // TEMP LEFT
  // ==================================================
  float tLSafe = safeFloat(tempDriverL, -40.0f, 150.0f);
  int16_t tL = (int16_t)tLSafe;

  packet[idx++] = highByte(tL);
  packet[idx++] = lowByte(tL);

  // ==================================================
  // TEMP RIGHT
  // ==================================================
  float tRSafe = safeFloat(tempDriverR, -40.0f, 150.0f);
  int16_t tR = (int16_t)tRSafe;

  packet[idx++] = highByte(tR);
  packet[idx++] = lowByte(tR);

  // ==================================================
  // LEN (SEQ → DATA)
  // ==================================================
  packet[lenIndex] = idx - 3;

  // ==================================================
  // CRC (payload only)
  // ==================================================
  uint16_t crc = crc16(&packet[3], idx - 3);

  packet[idx++] = crc & 0xFF;
  packet[idx++] = (crc >> 8) & 0xFF;

  // ==================================================
  // SEND
  // ==================================================
  BT.write(packet, idx);
}

