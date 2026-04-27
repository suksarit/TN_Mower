// ============================================================================
// BluetoothCommand.cpp
// FINAL SAFE VERSION
// ลด kill สุ่ม + ยังปลอดภัย + protocol เดิม
// ============================================================================

#include "BluetoothCommand.h"
#include "GlobalState.h"
#include "FaultManager.h"

#define BT Serial2

// ============================================================================
// CONFIG
// ============================================================================

// ไม่มี packet ใหม่เกินนี้ = soft stop
#define CMD_WARN_TIMEOUT   2000UL

// ไม่มี packet ใหม่เกินนี้ = hard kill
#define CMD_KILL_TIMEOUT   4000UL

// parser ค้างเกินนี้ reset state
#define PARSER_TIMEOUT_MS   150UL

// ============================================================================
// STATE
// ============================================================================

static uint8_t  lastCmdSeq  = 0xFF;
static uint32_t lastCmdTime = 0;
static bool     btLinked    = false;

// parser state
static uint8_t state = 0;
static uint8_t len   = 0;
static uint8_t idx   = 0;
static uint8_t buffer[32];

static uint32_t lastByteTime = 0;

// ============================================================================
// CRC16 MODBUS
// ============================================================================

static uint16_t crc16(uint8_t *data, uint8_t len)
{
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

// ============================================================================
// RESET PARSER
// ============================================================================

static void resetParser(void)
{
  state = 0;
  len   = 0;
  idx   = 0;
}

// ============================================================================
// SEND ACK
// protocol เดิม
// ============================================================================

static void sendAck(uint8_t seq, uint8_t status)
{
  uint8_t pkt[10];
  uint8_t p = 0;

  pkt[p++] = 0xAA;
  pkt[p++] = 0x55;

  uint8_t lenIndex = p++;

  pkt[p++] = seq;
  pkt[p++] = status;

  pkt[lenIndex] = p - 3;

  uint16_t crc = crc16(&pkt[3], p - 3);

  pkt[p++] = crc & 0xFF;
  pkt[p++] = (crc >> 8) & 0xFF;

  BT.write(pkt, p);
}

// ============================================================================
// PROCESS COMMAND
// ============================================================================

static void processCommand(uint8_t seq, uint8_t cmd)
{
  bool isDuplicate = (seq == lastCmdSeq);

  // duplicate ก็ถือว่ายังมีลิงก์อยู่
  lastCmdTime = millis();
  btLinked = true;

  if (isDuplicate) {
    sendAck(seq, 0x00);
    return;
  }

  lastCmdSeq = seq;

  switch (cmd) {

    // ==================================================
    // EMERGENCY STOP
    // ==================================================
    case 0x10:

      killRequest = KillType::HARD;
      killLatched = true;
      killISRFlag = true;

      sendAck(seq, 0x00);
      break;

    // ==================================================
    // START SYSTEM
    // เดิม set ACTIVE ตรง ๆ อันตรายเกินไป
    // เปลี่ยนเป็นกลับ INIT ให้ state machine หลักจัดการเอง
    // ==================================================
    case 0x11:

      if (!faultLatched) {
        systemState = SystemState::INIT;
        sendAck(seq, 0x00);
      } else {
        sendAck(seq, 0x02);
      }

      break;

    // ==================================================
    // RESET FAULT
    // ==================================================
    case 0x12:

      clearFault();
      sendAck(seq, 0x00);
      break;

    // ==================================================
    // HEARTBEAT
    // ==================================================
    case 0x01:

      sendAck(seq, 0x00);
      break;

    // ==================================================
    // UNKNOWN
    // ==================================================
    default:

      sendAck(seq, 0x01);
      break;
  }
}

// ============================================================================
// RECEIVE COMMAND
// ============================================================================

void btReceiveCommand(void)
{
  uint32_t now = millis();

  // parser stall protection
  if (state != 0) {
    if (now - lastByteTime > PARSER_TIMEOUT_MS) {
      resetParser();
    }
  }

  while (BT.available()) {

    uint8_t b = BT.read();
    lastByteTime = now;

    switch (state) {

      // ------------------------------------------------
      // HEADER 1
      // ------------------------------------------------
      case 0:

        if (b == 0xAA)
          state = 1;

        break;

      // ------------------------------------------------
      // HEADER 2
      // ------------------------------------------------
      case 1:

        if (b == 0x55)
          state = 2;
        else
          resetParser();

        break;

      // ------------------------------------------------
      // LEN
      // ------------------------------------------------
      case 2:

        if (b < 2 || b > 28) {
          resetParser();
        } else {
          len = b;
          idx = 0;
          state = 3;
        }

        break;

      // ------------------------------------------------
      // PAYLOAD + CRC
      // ------------------------------------------------
      case 3:

        if (idx < sizeof(buffer)) {
          buffer[idx++] = b;
        } else {
          resetParser();
          break;
        }

        if (idx >= (uint8_t)(len + 2)) {

          uint16_t crcCalc = crc16(buffer, len);

          uint16_t crcRecv =
            buffer[len] |
            (buffer[len + 1] << 8);

          if (crcCalc == crcRecv) {

            uint8_t seq = buffer[0];
            uint8_t cmd = buffer[1];

            processCommand(seq, cmd);
          }

          resetParser();
        }

        break;

      default:

        resetParser();
        break;
    }
  }
}

// ============================================================================
// SAFETY CHECK
// ============================================================================

void btSafetyCheck(void)
{
  if (killLatched)
    return;

  if (!btLinked)
    return;

  uint32_t dt = millis() - lastCmdTime;

  // --------------------------------------------------
  // stage 1 : command หาย -> สั่งหยุดนุ่มนวล
  // --------------------------------------------------
  if (dt > CMD_WARN_TIMEOUT &&
      dt <= CMD_KILL_TIMEOUT) {

    targetL = 0.0f;
    targetR = 0.0f;
  }

  // --------------------------------------------------
  // stage 2 : หายนานจริง -> hard kill
  // --------------------------------------------------
  if (dt > CMD_KILL_TIMEOUT) {

    killRequest = KillType::HARD;
    killLatched = true;
    killISRFlag = true;
  }
}

