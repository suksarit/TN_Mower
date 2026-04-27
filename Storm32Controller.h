#ifndef STORM32_CONTROLLER_H
#define STORM32_CONTROLLER_H

#include <Arduino.h>
#include <IBusBM.h>
#include <EEPROM.h>

// ======================================================
// EEPROM
// ======================================================
#define EEPROM_STORM32_BASE 200
#define STORM32_MAGIC 0x32B3

// ======================================================
// SERIAL PROTOCOL
// ======================================================
#define STORM32_START_BYTE   0xFA
#define STORM32_CMD_CONTROL  0x19

// ======================================================
// RC CHANNEL
// ======================================================
#define STORM32_CH_PITCH 3
#define STORM32_CH_YAW   9

// ======================================================
// PWM OUTPUT PIN (Mega2560)
// ======================================================
#define GIMBAL_PWM_PIN_PITCH 10
#define GIMBAL_PWM_PIN_YAW   11

// ======================================================
// FRAME
// ======================================================
#define STORM32_MAX_FRAME 32

// ======================================================
// STATE
// ======================================================
enum class Storm32State : uint8_t
{
  INIT,
  OK,
  DEGRADED,
  LOST,
  EMERGENCY
};

// ======================================================
// OUTPUT MODE
// ======================================================
enum class GimbalOutputMode : uint8_t
{
  MODE_SERIAL = 0,
  MODE_PWM    = 1,
  MODE_AUTO   = 2
};

// ======================================================
// CONFIG
// ======================================================
struct Storm32Config
{
  bool invertPitch;
  bool invertYaw;

  float pitchLimitDeg;
  float yawLimitDeg;

  float slewNormal;
  float slewDegraded;

  uint16_t ackTimeout1;
  uint16_t ackTimeout2;
  uint16_t ackTimeout3;

  uint8_t outputMode;

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

  void begin();
  void update(uint32_t now);

  void forceOff();
  void hardDisable(bool enable);
  void setSystemEnabled(bool enable);
  void clearEmergency();

  void setOutputMode(GimbalOutputMode mode);
  GimbalOutputMode getOutputMode() const;

  Storm32State getState() const;
  bool isLocked() const;

private:

  // ----------------------------------------
  // REF
  // ----------------------------------------
  HardwareSerial& stormSerial;
  IBusBM& ibus;

  // ----------------------------------------
  // CONFIG
  // ----------------------------------------
  Storm32Config cfg;
  Storm32State state;
  GimbalOutputMode mode;

  bool hardDisabled;
  bool systemEnabled;

  // ----------------------------------------
  // TIME
  // ----------------------------------------
  uint32_t lastAckMs;
  uint32_t lastTxMs;
  uint32_t lastUpdateMs;

  uint32_t lastPwmFrameMs;
  uint32_t autoModeStartMs;

  // ----------------------------------------
  // TARGET
  // ----------------------------------------
  float targetPitch;
  float targetYaw;

  float currentPitch;
  float currentYaw;

  // ----------------------------------------
  // PARSER
  // ----------------------------------------
  uint8_t frameBuf[STORM32_MAX_FRAME];
  uint8_t frameIndex;
  bool frameActive;

  // ----------------------------------------
  // CONST
  // ----------------------------------------
  static constexpr uint16_t TX_PERIOD_MS = 100;
  static constexpr uint16_t PWM_FRAME_MS = 20;

  // ----------------------------------------
  // INTERNAL
  // ----------------------------------------
  void updateTargetFromIBUS();
  void applyMotion(float dt);
  void updateWDT(uint32_t now);

  float mapRCtoDeg(uint16_t rc,
                   float limit,
                   bool invert);

  float stepToward(float cur,
                   float tgt,
                   float maxStep);

  // SERIAL
  void processSerial(uint32_t now);
  void parseByte(uint8_t b, uint32_t now);
  void sendBinaryControl(int16_t pitch,
                         int16_t yaw);

  // PWM
  void sendPwmControl(uint32_t now);
  uint16_t degToUs(float deg,
                   float limit);

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

