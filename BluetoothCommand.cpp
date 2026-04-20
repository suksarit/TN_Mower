// ============================================================================
// BluetoothCommand.cpp (FINAL + FIX LINK ERROR + MATCH PROTOCOL)
// ============================================================================

#include "BluetoothCommand.h"
#include "GlobalState.h"
#include "FaultManager.h"

#define BT Serial2

#define CMD_TIMEOUT 2000  // ms

// ==================================================
// 🔴 STATE (GLOBAL STATIC)
// ==================================================
static uint8_t lastCmdSeq = 0;
static uint32_t lastCmdTime = 0;

// ==================================================
// 🔴 CRC16 (MODBUS - ตรง Android)
// ==================================================
static uint16_t crc16(uint8_t *data, uint8_t len) {

  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];

    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }

  return crc;
}

// ==================================================
// 🔴 SEND ACK (PROTOCOL เดียวกับ Telemetry)
// ==================================================
static void sendAck(uint8_t seq, uint8_t status) {

  uint8_t pkt[10];
  uint8_t idx = 0;

  // HEADER
  pkt[idx++] = 0xAA;
  pkt[idx++] = 0x55;

  uint8_t lenIndex = idx++;

  // PAYLOAD
  pkt[idx++] = seq;
  pkt[idx++] = status;

  // LEN (payload only)
  pkt[lenIndex] = idx - 3;

  // CRC (payload only)
  uint16_t crc = crc16(&pkt[3], idx - 3);

  pkt[idx++] = crc & 0xFF;
  pkt[idx++] = (crc >> 8) & 0xFF;

  BT.write(pkt, idx);
}

// ==================================================
// 🔴 RECEIVE COMMAND (NON-BLOCKING + CRC + DUPLICATE SAFE)
// ==================================================
void btReceiveCommand(void) {

  static uint8_t state = 0;
  static uint8_t len = 0;
  static uint8_t idx = 0;
  static uint8_t buffer[32];

  while (BT.available()) {

    uint8_t b = BT.read();

    switch (state) {

      // -----------------------------
      // HEADER 1
      // -----------------------------
      case 0:
        if (b == 0xAA) state = 1;
        break;

      // -----------------------------
      // HEADER 2
      // -----------------------------
      case 1:
        if (b == 0x55) state = 2;
        else state = 0;
        break;

      // -----------------------------
      // LEN
      // -----------------------------
      case 2:
        if (b < 2 || b > 28) {
          state = 0;
        } else {
          len = b;
          idx = 0;
          state = 3;
        }
        break;

      // -----------------------------
      // PAYLOAD + CRC
      // -----------------------------
      case 3:
        buffer[idx++] = b;

        if (idx >= len + 2) {
          state = 4;
        }
        break;

      // -----------------------------
      // PROCESS PACKET
      // -----------------------------
      case 4: {

        uint16_t crcCalc = crc16(buffer, len);

        uint16_t crcRecv =
          buffer[len] |
          (buffer[len + 1] << 8);

        if (crcCalc == crcRecv) {

          uint8_t i = 0;
          uint8_t seq = buffer[i++];
          uint8_t cmd = buffer[i++];

          bool isDuplicate = (seq == lastCmdSeq);

          if (!isDuplicate) {

            lastCmdSeq = seq;
            lastCmdTime = millis();

            switch (cmd) {

              // ==============================
              // 🔴 EMERGENCY STOP (CRITICAL)
              // ==============================
              case 0x10:
                killRequest = KillType::HARD;
                killLatched = true;
                killISRFlag = true;
                sendAck(seq, 0x00);
                break;

              // ==============================
              // START SYSTEM
              // ==============================
              case 0x11:
                systemState = SystemState::ACTIVE;
                sendAck(seq, 0x00);
                break;

              // ==============================
              // RESET FAULT
              // ==============================
              case 0x12:
                clearFault();
                sendAck(seq, 0x00);
                break;

              // ==============================
              // UNKNOWN CMD
              // ==============================
              default:
                sendAck(seq, 0x01);
                break;
            }

          } else {
            // 🔴 duplicate → ACK ซ้ำ
            sendAck(seq, 0x00);
          }
        }

        state = 0;
        break;
      }

      default:
        state = 0;
        break;
    }
  }
}

// ==================================================
// 🔴 SAFETY CHECK (FIX LINK ERROR ตรงนี้แหละ)
// ==================================================
void btSafetyCheck(void) {

  // 🔴 ถ้าล็อกแล้ว ไม่ต้องทำอะไร
  if (killLatched) return;

  // 🔴 ยังไม่เคยรับคำสั่ง
  if (lastCmdTime == 0) return;

  // 🔴 timeout → HARD KILL
  if (millis() - lastCmdTime > CMD_TIMEOUT) {

    killRequest = KillType::HARD;
    killLatched = true;
    killISRFlag = true;
  }
}

