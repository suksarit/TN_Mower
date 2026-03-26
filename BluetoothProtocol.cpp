// ============================================================================
// BluetoothProtocol.cpp (CRC + SAFETY + NO DRIVE CONTROL)
// ============================================================================

#include "BluetoothProtocol.h"
#include "SystemTypes.h"
#include "FaultManager.h"

extern KillType killRequest;

// ==============================
#define BT Serial2

// ==============================
static unsigned long lastHB = 0;
static const unsigned long TIMEOUT = 3000;

static String rx = "";

// 🔴 กันยิง STOP ซ้ำ
static bool timeoutLatched = false;

// ==================================================
static void processPacket(String packet);
static String calcCRC(String data);
static bool checkCRC(String packet);
static String removeCRC(String packet);
static void sendWithCRC(const String &msg);

// 🔴 แยก STOP
static void emergencyStopSoft(const char* reason);
static void emergencyStopHard(const char* reason);

// ==============================
void btProtocolInit() {}

// ==============================
void btProtocolUpdate() {

  // ==================================================
  // READ SERIAL
  // ==================================================
  while (BT.available()) {

    char c = BT.read();

    if (c == '\n') {

      processPacket(rx);
      rx = "";

    } else {

      // 🔴 ป้องกัน fragmentation
      if (rx.length() < 120)
        rx += c;
      else
        rx = "";
    }
  }

  // ==================================================
  // TIMEOUT → HARD STOP
  // ==================================================
  if (millis() - lastHB > TIMEOUT) {

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
  if (packet.length() == 0) return;

  // 🔴 CRC CHECK
  if (!checkCRC(packet)) {
    emergencyStopHard("CRC_FAIL");
    return;
  }

  String data = removeCRC(packet);

  // ==================================================
  // 🔴 STOP → SOFT (สำคัญสุด)
  // ==================================================
  if (data == "CMD:STOP") {

    lastHB = millis();        // 🔴 กัน timeout ทับ
    timeoutLatched = false;

    emergencyStopSoft("REMOTE_STOP");
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
  // BLOCK DRIVE COMMAND
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

  sendWithCRC(String("STAT:STOP"));
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
static String calcCRC(String data) {

  uint8_t crc = 0;

  for (size_t i = 0; i < data.length(); i++) {
    crc ^= (uint8_t)data[i];
  }

  char hex[3];
  sprintf(hex, "%02X", crc);

  return String(hex);
}

// ==============================
static bool checkCRC(String packet) {

  int sep = packet.indexOf('|');
  if (sep == -1) return false;

  String data = packet.substring(0, sep);
  String crc  = packet.substring(sep + 1);

  return calcCRC(data) == crc;
}

// ==============================
static String removeCRC(String packet) {

  int sep = packet.indexOf('|');
  if (sep == -1) return "";

  return packet.substring(0, sep);
}

