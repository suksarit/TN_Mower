// ============================================================================
// BluetoothProtocol.cpp (FINAL - INDUSTRIAL SAFE / ROBUST)
// ============================================================================

#include "BluetoothProtocol.h"
#include "SystemTypes.h"
#include "FaultManager.h"

extern KillType killRequest;

// ==============================
#define BT Serial2

// ==============================
// HEARTBEAT
static unsigned long lastHB = 0;
static const unsigned long TIMEOUT = 3000;
static const unsigned long HB_GRACE = 500;   // 🔴 กัน jitter

// ==============================
// BUFFER
static String rx = "";

// 🔴 latch กันยิงซ้ำ
static bool timeoutLatched = false;

// 🔴 STOP debounce
static unsigned long lastStop_ms = 0;
static const unsigned long STOP_DEBOUNCE_MS = 200;

// ==================================================
static void processPacket(String packet);
static String calcCRC(const String &data);
static bool checkCRC(const String &packet);
static String removeCRC(const String &packet);
static void sendWithCRC(const String &msg);

static void emergencyStopSoft(const char* reason);
static void emergencyStopHard(const char* reason);

// ==============================
void btProtocolInit() {
  lastHB = millis();  // 🔴 init กัน timeout ตอน boot
}

// ==============================
void btProtocolUpdate() {

  // ==================================================
  // READ SERIAL (robust)
  // ==================================================
  while (BT.available()) {

    char c = BT.read();

    // 🔴 filter char แปลก
    if (c == '\r') continue;

    if (c == '\n') {

      processPacket(rx);
      rx = "";

    } else {

      // 🔴 จำกัด buffer
      if (rx.length() < 120) {
        rx += c;
      } else {
        rx = "";  // flush ทิ้ง
      }
    }
  }

  // ==================================================
  // TIMEOUT → HARD STOP
  // ==================================================
  unsigned long now = millis();

  if ((now - lastHB) > (TIMEOUT + HB_GRACE)) {

    if (!timeoutLatched) {
      emergencyStopHard("TIMEOUT");
      timeoutLatched = true;
    }

  } else {
    timeoutLatched = false;
  }
}

// ==============================
// PROCESS PACKET
// ==============================
static void processPacket(String packet) {

  packet.trim();

  // 🔴 packet สั้นเกิน = ignore
  if (packet.length() < 4) return;

  // ==================================================
  // CRC CHECK
  // ==================================================
  if (!checkCRC(packet)) {

    // 🔴 ไม่ kill ทันที → กัน false noise
#if DEBUG_SERIAL
    Serial.println(F("[BT] CRC FAIL IGNORE"));
#endif
    return;
  }

  String data = removeCRC(packet);

  // ==================================================
  // 🔴 PRIORITY STOP (SOFT)
  // ==================================================
  if (data == "CMD:STOP") {

    unsigned long now = millis();

    // 🔴 debounce กัน spam
    if (now - lastStop_ms > STOP_DEBOUNCE_MS) {

      emergencyStopSoft("REMOTE_STOP");
      lastStop_ms = now;
    }

    // 🔴 refresh HB
    lastHB = now;
    timeoutLatched = false;

    return;
  }

  // ==================================================
  // HEARTBEAT
  // ==================================================
  if (data == "HB") {

    lastHB = millis();
    timeoutLatched = false;

    sendWithCRC("STAT:OK");
    return;
  }

  // ==================================================
  // RESET FAULT
  // ==================================================
  if (data == "CMD:RESET") {

    clearFault();
    sendWithCRC("STAT:RESET");
    return;
  }

  // ==================================================
  // BLOCK DRIVE COMMAND (สำคัญ)
  // ==================================================
  if (data.startsWith("CMD:FWD") ||
      data.startsWith("CMD:BACK") ||
      data.startsWith("CMD:TURN")) {

    sendWithCRC("ERR:DENY");
    return;
  }

  // ==================================================
  // UNKNOWN
  // ==================================================
  sendWithCRC("ERR:UNKNOWN");
}

// ==============================
// SOFT STOP (ผ่าน DriveRamp)
// ==============================
static void emergencyStopSoft(const char* reason) {

  killRequest = KillType::SOFT;

#if DEBUG_SERIAL
  Serial.print(F("[BT SOFT STOP] "));
  Serial.println(reason);
#endif

  sendWithCRC("STAT:STOP");
}

// ==============================
// HARD STOP (ฉุกเฉินจริง)
// ==============================
static void emergencyStopHard(const char* reason) {

  killRequest = KillType::HARD;

#if DEBUG_SERIAL
  Serial.print(F("[BT HARD STOP] "));
  Serial.println(reason);
#endif

  sendWithCRC(String("ERR:") + reason);
}

// ==============================
// SEND WITH CRC
// ==============================
static void sendWithCRC(const String &msg) {

  String packet = msg + "|" + calcCRC(msg);
  BT.println(packet);
}

// ==============================
// CRC (XOR)
// ==============================
static String calcCRC(const String &data) {

  uint8_t crc = 0;

  for (size_t i = 0; i < data.length(); i++) {
    crc ^= (uint8_t)data[i];
  }

  char hex[3];
  sprintf(hex, "%02X", crc);

  return String(hex);
}

// ==============================
static bool checkCRC(const String &packet) {

  int sep = packet.indexOf('|');
  if (sep == -1) return false;

  String data = packet.substring(0, sep);
  String crc  = packet.substring(sep + 1);

  return calcCRC(data) == crc;
}

// ==============================
static String removeCRC(const String &packet) {

  int sep = packet.indexOf('|');
  if (sep == -1) return "";

  return packet.substring(0, sep);
}

