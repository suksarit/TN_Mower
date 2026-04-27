#include "Storm32Controller.h"
#include "GlobalState.h"

// ======================================================
// SINGLETON
// ======================================================
Storm32Controller& getGimbal()
{
  static Storm32Controller gimbal(Serial3, ibus);
  return gimbal;
}

// ======================================================
// CONSTRUCTOR
// ======================================================
Storm32Controller::Storm32Controller(HardwareSerial& serial,
                                     IBusBM& ib)
  : stormSerial(serial),
    ibus(ib)
{
}

// ======================================================
// BEGIN
// ======================================================
void Storm32Controller::begin()
{
  stormSerial.begin(115200);

  loadConfig();

  uint32_t now = millis();

  state = Storm32State::INIT;

  hardDisabled  = false;
  systemEnabled = false;

  lastAckMs    = now;
  lastTxMs     = now;
  lastUpdateMs = now;

  targetPitch  = 0.0f;
  targetYaw    = 0.0f;

  currentPitch = 0.0f;
  currentYaw   = 0.0f;
}

// ======================================================
// UPDATE
// ======================================================
void Storm32Controller::update(uint32_t now)
{
  float dt = (now - lastUpdateMs) * 0.001f;
  lastUpdateMs = now;

  if (dt <= 0.0f) dt = 0.001f;
  if (dt > 0.1f)  dt = 0.1f;

  bool locked =
      hardDisabled ||
      !systemEnabled ||
      state == Storm32State::INIT ||
      state == Storm32State::EMERGENCY;

  if (locked)
  {
    sendDriveOff();
    return;
  }

  // อ่านข้อมูลตอบกลับ (ถ้ามี)
  processSerial(now);

  // ตรวจ timeout
  updateWDT(now);

  // ถ้าหลุด ให้หยุดส่งคำสั่ง
  if (state == Storm32State::LOST)
  {
    sendDriveOff();
    return;
  }

  // อ่านรีโมท
  updateTargetFromIBUS();

  // เคลื่อนนุ่มนวล
  applyMotion(dt);

  // ส่งคำสั่งทุกช่วงเวลา
  if (now - lastTxMs >= TX_PERIOD_MS)
  {
    sendBinaryControl(
      (int16_t)(currentPitch * 100.0f),
      (int16_t)(currentYaw   * 100.0f));

    lastTxMs = now;
  }
}

// ======================================================
// FORCE OFF
// ======================================================
void Storm32Controller::forceOff()
{
  hardDisabled = true;
  state = Storm32State::EMERGENCY;

  sendDriveOff();
}

// ======================================================
// HARD DISABLE
// ======================================================
void Storm32Controller::hardDisable(bool enable)
{
  hardDisabled = enable;

  if (hardDisabled)
  {
    state = Storm32State::EMERGENCY;
    sendDriveOff();
  }
}

// ======================================================
// SYSTEM ENABLE
// ======================================================
void Storm32Controller::setSystemEnabled(bool enable)
{
  systemEnabled = enable;

  if (!systemEnabled && !hardDisabled)
  {
    state = Storm32State::INIT;
    sendDriveOff();
  }

  if (systemEnabled && !hardDisabled)
  {
    state = Storm32State::OK;

    uint32_t now = millis();

    lastAckMs = now;
    lastTxMs  = now;
  }
}

// ======================================================
// CLEAR EMERGENCY
// ======================================================
void Storm32Controller::clearEmergency()
{
  if (hardDisabled)
    return;

  state = Storm32State::OK;

  uint32_t now = millis();

  lastAckMs = now;
  lastTxMs  = now;
}

// ======================================================
// TARGET FROM IBUS
// ======================================================
void Storm32Controller::updateTargetFromIBUS()
{
  uint16_t rcPitch = ibus.readChannel(STORM32_CH_PITCH);
  uint16_t rcYaw   = ibus.readChannel(STORM32_CH_YAW);

  rcPitch = constrain(rcPitch, 1000, 2000);
  rcYaw   = constrain(rcYaw,   1000, 2000);

  targetPitch =
    mapRCtoDeg(rcPitch,
               cfg.pitchLimitDeg,
               cfg.invertPitch);

  targetYaw =
    mapRCtoDeg(rcYaw,
               cfg.yawLimitDeg,
               cfg.invertYaw);
}

// ======================================================
// APPLY MOTION
// ======================================================
void Storm32Controller::applyMotion(float dt)
{
  float step = cfg.slewRateDegPerSec * dt;

  currentPitch =
    stepToward(currentPitch, targetPitch, step);

  currentYaw =
    stepToward(currentYaw, targetYaw, step);
}

// ======================================================
// STEP
// ======================================================
float Storm32Controller::stepToward(float cur,
                                    float tgt,
                                    float maxStep)
{
  float d = tgt - cur;

  if (fabs(d) <= maxStep)
    return tgt;

  if (d > 0.0f)
    return cur + maxStep;

  return cur - maxStep;
}

// ======================================================
// MAP RC TO DEG
// ======================================================
float Storm32Controller::mapRCtoDeg(uint16_t rc,
                                    float limit,
                                    bool invert)
{
  float out =
    ((float)rc - 1500.0f) / 500.0f * limit;

  if (invert)
    out = -out;

  return out;
}

// ======================================================
// SERIAL RX
// ======================================================
void Storm32Controller::processSerial(uint32_t now)
{
  while (stormSerial.available() > 0)
  {
    stormSerial.read();

    // ถ้ามีข้อมูลกลับมา ถือว่ายัง online
    lastAckMs = now;

    if (state == Storm32State::LOST)
    {
      state = Storm32State::OK;
    }
  }
}

// ======================================================
// WATCHDOG
// ======================================================
void Storm32Controller::updateWDT(uint32_t now)
{
  uint32_t dt = now - lastAckMs;

  if (state == Storm32State::OK &&
      dt > cfg.ackTimeoutMs)
  {
    state = Storm32State::LOST;
  }
}

// ======================================================
// SERIAL TX
// ======================================================
void Storm32Controller::sendBinaryControl(int16_t pitch,
                                          int16_t yaw)
{
  uint8_t pkt[9];

  pkt[0] = STORM32_START_BYTE;
  pkt[1] = 4;
  pkt[2] = STORM32_CMD_CONTROL;

  pkt[3] = pitch & 0xFF;
  pkt[4] = (pitch >> 8) & 0xFF;

  pkt[5] = yaw & 0xFF;
  pkt[6] = (yaw >> 8) & 0xFF;

  uint16_t crc = crc16_ccitt(pkt, 7);

  pkt[7] = crc & 0xFF;
  pkt[8] = (crc >> 8) & 0xFF;

  stormSerial.write(pkt, 9);
}

// ======================================================
// DRIVE OFF
// ======================================================
void Storm32Controller::sendDriveOff()
{
  sendBinaryControl(0, 0);
}

// ======================================================
// CRC16
// ======================================================
uint16_t Storm32Controller::crc16_ccitt(
  const uint8_t* data,
  uint8_t len)
{
  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= ((uint16_t)data[i] << 8);

    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x8000)
      {
        crc = (crc << 1) ^ 0x1021;
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

// ======================================================
// EEPROM LOAD
// ======================================================
void Storm32Controller::loadConfig()
{
  EEPROM.get(EEPROM_STORM32_BASE, cfg);

  if (cfg.magic != STORM32_MAGIC)
  {
    setDefaultConfig();
    saveConfig();
  }
}

// ======================================================
// EEPROM SAVE
// ======================================================
void Storm32Controller::saveConfig()
{
  cfg.magic = STORM32_MAGIC;

  EEPROM.put(EEPROM_STORM32_BASE, cfg);
}

// ======================================================
// DEFAULT CONFIG
// ======================================================
void Storm32Controller::setDefaultConfig()
{
  cfg.invertPitch = false;
  cfg.invertYaw   = false;

  cfg.pitchLimitDeg = 30.0f;
  cfg.yawLimitDeg   = 45.0f;

  cfg.slewRateDegPerSec = 60.0f;

  cfg.ackTimeoutMs = 1500;

  cfg.magic = STORM32_MAGIC;
}

// ======================================================
// STATUS
// ======================================================
Storm32State Storm32Controller::getState() const
{
  return state;
}

// ======================================================
// LOCK CHECK
// ======================================================
bool Storm32Controller::isLocked() const
{
  return hardDisabled ||
         !systemEnabled ||
         state == Storm32State::INIT ||
         state == Storm32State::EMERGENCY;
}

