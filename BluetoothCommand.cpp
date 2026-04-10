// ============================================================================
// BluetoothCommand.cpp (CLEAN + MATCH ANDROID)
// ============================================================================

#include "BluetoothCommand.h"
#include "GlobalState.h"
#include "FaultManager.h"

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
// SEND ACK
// ==================================================
static void sendAck(uint8_t seq, uint8_t status) {

  uint8_t pkt[6];
  uint8_t idx = 0;

  pkt[idx++] = 0xAB;   // ACK header
  pkt[idx++] = 2;      // LEN
  pkt[idx++] = seq;
  pkt[idx++] = status;

  uint16_t crc = crc16(pkt, idx);

  pkt[idx++] = crc & 0xFF;
  pkt[idx++] = (crc >> 8) & 0xFF;

  BT.write(pkt, idx);
}

// ==================================================
// RECEIVE COMMAND (NON-BLOCKING + ACK + SAFE)
// ==================================================
void btReceiveCommand() {

  static uint8_t state = 0;
  static uint8_t len = 0;
  static uint8_t idx = 0;
  static uint8_t buffer[32];
  static uint8_t crcLow = 0;

  while (BT.available()) {

    uint8_t b = BT.read();

    switch (state) {

      // =========================
      // WAIT HEADER
      // =========================
      case 0:
        if (b == 0xAA) {
          state = 1;
        }
        break;

      // =========================
      // READ LEN
      // =========================
      case 1:
        if (b < 2 || b > 20) {
          state = 0;  // invalid
        } else {
          len = b;
          idx = 0;
          state = 2;
        }
        break;

      // =========================
      // READ PAYLOAD
      // =========================
      case 2:
        buffer[idx++] = b;

        if (idx >= len) {
          state = 3;
        }
        break;

      // =========================
      // CRC LOW
      // =========================
      case 3:
        crcLow = b;
        state = 4;
        break;

      // =========================
      // CRC HIGH + PROCESS
      // =========================
      case 4: {

        uint16_t crcRx = (b << 8) | crcLow;

        uint8_t temp[40];
        temp[0] = 0xAA;
        temp[1] = len;
        memcpy(&temp[2], buffer, len);

        uint16_t crcCalc = crc16(temp, len + 2);

        if (crcCalc == crcRx) {

          uint8_t i = 0;
          uint8_t seq = buffer[i++];
          uint8_t cmd = buffer[i++];

          bool isDuplicate = (seq == lastCmdSeq);

          if (!isDuplicate) {

            lastCmdSeq = seq;
            lastCmdTime = millis();

            switch (cmd) {

              // ======================================
              // 🔴 EMERGENCY STOP (CRITICAL)
              // ======================================
              case 0x10:
                killRequest = KillType::HARD;
                killLatched = true;
                killISRFlag = true;

                sendAck(seq, 0x00);
                break;

              // ======================================
              // START
              // ======================================
              case 0x11:
                systemState = SystemState::ACTIVE;

                sendAck(seq, 0x00);
                break;

              // ======================================
              // RESET FAULT
              // ======================================
              case 0x12:
                clearFault();   // ต้อง include FaultManager.h

                sendAck(seq, 0x00);
                break;

              // ======================================
              // UNKNOWN
              // ======================================
              default:
                sendAck(seq, 0x01);
                break;
            }

          } else {
            // 🔴 duplicate → ACK ซ้ำให้ Android หยุด retry
            sendAck(seq, 0x00);
          }
        }

        // reset state machine
        state = 0;
        break;
      }

      default:
        state = 0;
        break;
    }
  }
}

void btSafetyCheck() {

  if (killLatched) return;  // 🔴 กันยิงซ้ำ

  if (lastCmdTime == 0) return;

  if (millis() - lastCmdTime > CMD_TIMEOUT) {

    killRequest = KillType::HARD;
    killLatched = true;
    killISRFlag = true;
  }
}

