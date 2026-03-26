// BluetoothTelemetry.cpp

// ============================================================================
// BluetoothTelemetry.cpp (SEND DATA ONLY)
// ============================================================================

#include "BluetoothTelemetry.h"
#include "GlobalState.h"
#include "FaultManager.h"

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
void btTelemetryUpdate(uint32_t now) {

  if (now - lastSend < TELEMETRY_PERIOD)
    return;

  lastSend = now;

  // ==================================================
  // BUILD DATA
  // ==================================================
  String msg = "";

  // 🔴 Fault
  msg += "FLT:";
  msg += (int)getActiveFault();

  // 🔴 System state
  msg += ",SYS:";
  msg += (int)systemState;

  // 🔴 ตัวอย่างเพิ่มเองได้
  msg += ",V:";
  msg += engineVolt;

  // ==================================================
  // SEND
  // ==================================================
  String packet = msg + "|" + calcCRC(msg);
  BT.println(packet);
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

