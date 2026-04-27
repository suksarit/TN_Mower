#ifndef STORM32_CONTROLLER_H
#define STORM32_CONTROLLER_H

#include <Arduino.h>
#include <IBusBM.h>
#include <EEPROM.h>

// ======================================================
// EEPROM
// ======================================================
#define EEPROM_STORM32_BASE 200
#define STORM32_MAGIC       0x32B3

// ======================================================
// SERIAL PROTOCOL
// ======================================================
#define STORM32_START_BYTE  0xFA
#define STORM32_CMD_CONTROL 0x19

// ======================================================
// RC CHANNEL
// ======================================================
#define STORM32_CH_PITCH 3
#define STORM32_CH_YAW   9

// ======================================================
// STATE (ลดให้เหลือเท่าที่จำเป็น)
// ======================================================
enum class Storm32State : uint8_t
{
  INIT,        // ยังไม่พร้อม
  OK,          // ทำงานปกติ
  LOST,        // ไม่ตอบสนอง
  EMERGENCY    // ถูกล็อก
};

// ======================================================
// CONFIG (ลดให้เหลือพื้นฐาน)
// ======================================================
struct Storm32Config
{
  bool invertPitch;
  bool invertYaw;

  float pitchLimitDeg;
  float yawLimitDeg;

  float slewRateDegPerSec;

  uint16_t ackTimeoutMs;

  uint16_t magic;
};

// ======================================================
// CLASS
// ======================================================
class Storm32Controller
{
public:

  Storm32Controller(HardwareSerial& serial,
                    IBusBM& ibus);

  // เริ่มระบบ
  void begin();

  // เรียกใน loop()
  void update(uint32_t now);

  // ปิดฉุกเฉิน
  void forceOff();

  // ล็อก/ปลดล็อก
  void hardDisable(bool enable);

  // เปิด/ปิดตามระบบหลัก
  void setSystemEnabled(bool enable);

  // ล้าง emergency
  void clearEmergency();

  // อ่านสถานะ
  Storm32State getState() const;

  // ถูกล็อกหรือไม่
  bool isLocked() const;

private:

  // ====================================================
  // REFERENCE
  // ====================================================
  HardwareSerial& stormSerial;
  IBusBM& ibus;

  // ====================================================
  // CONFIG
  // ====================================================
  Storm32Config cfg;

  // ====================================================
  // STATE
  // ====================================================
  Storm32State state;

  bool hardDisabled;
  bool systemEnabled;

  // ====================================================
  // TIME
  // ====================================================
  uint32_t lastAckMs;
  uint32_t lastTxMs;
  uint32_t lastUpdateMs;

  // ====================================================
  // TARGET / CURRENT
  // ====================================================
  float targetPitch;
  float targetYaw;

  float currentPitch;
  float currentYaw;

  // ====================================================
  // CONST
  // ====================================================
  static constexpr uint16_t TX_PERIOD_MS = 50; // 20Hz

  // ====================================================
  // INTERNAL
  // ====================================================
  void updateTargetFromIBUS();
  void applyMotion(float dt);
  void updateWDT(uint32_t now);

  float mapRCtoDeg(uint16_t rc,
                   float limit,
                   bool invert);

  float stepToward(float cur,
                   float tgt,
                   float maxStep);

  void processSerial(uint32_t now);

  void sendBinaryControl(int16_t pitch,
                         int16_t yaw);

  void sendDriveOff();

  uint16_t crc16_ccitt(const uint8_t* data,
                       uint8_t len);

  void loadConfig();
  void saveConfig();
  void setDefaultConfig();
};

// ======================================================
// SINGLETON
// ======================================================
Storm32Controller& getGimbal();

#endif
