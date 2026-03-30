// ============================================================================
// BluetoothCommand.cpp (CLEAN + MATCH ANDROID)
// ============================================================================

#include "BluetoothCommand.h"
#include "GlobalState.h"

#define BT Serial2

#define CMD_TIMEOUT 2000  // ms

static uint8_t lastCmdSeq = 0;
static uint32_t lastCmdTime = 0;

// ==================================================
// CRC16
// ==================================================
static uint16_t crc16(uint8_t *data, uint8_t len) {

  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];

    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }

  return crc;
}

// ==================================================
// RECEIVE COMMAND
// ==================================================
void btReceiveCommand() {

  if (!BT.available()) return;

  if (BT.read() != 0xAA) return;

  uint8_t len = BT.read();
  if (len < 2 || len > 20) return;

  uint8_t buffer[32];
  BT.readBytes(buffer, len);

  uint8_t crcLow = BT.read();
  uint8_t crcHigh = BT.read();
  uint16_t crcRx = (crcHigh << 8) | crcLow;

  uint8_t temp[40];
  temp[0] = 0xAA;
  temp[1] = len;
  memcpy(&temp[2], buffer, len);

  if (crc16(temp, len + 2) != crcRx) return;

  uint8_t idx = 0;

  // 🔴 FORMAT: SEQ + CMD
  uint8_t seq = buffer[idx++];
  uint8_t cmd = buffer[idx++];

  // กันซ้ำ
  if (seq == lastCmdSeq) return;
  lastCmdSeq = seq;

  lastCmdTime = millis();

  // ==================================================
  // COMMAND
  // ==================================================
  switch (cmd) {

    case 0x10:  // STOP

      killRequest = KillType::HARD;
      killLatched = true;
      killISRFlag = true;

      break;

    case 0x11:
      break;

    case 0x12:
      break;

    default:
      break;
  }
}

// ==================================================
// FAILSAFE
// ==================================================
void btSafetyCheck() {

  if (lastCmdTime == 0) return;

  if (millis() - lastCmdTime > CMD_TIMEOUT) {

    killRequest = KillType::HARD;
    killLatched = true;
    killISRFlag = true;
  }
}

