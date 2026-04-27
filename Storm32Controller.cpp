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
  pinMode(GIMBAL_PWM_PIN_PITCH, OUTPUT);
  pinMode(GIMBAL_PWM_PIN_YAW, OUTPUT);

  digitalWrite(GIMBAL_PWM_PIN_PITCH, LOW);
  digitalWrite(GIMBAL_PWM_PIN_YAW, LOW);

  stormSerial.begin(115200);

  loadConfig();

  mode = (GimbalOutputMode)cfg.outputMode;

  uint32_t now = millis();

  state = Storm32State::INIT;

  hardDisabled = false;
  systemEnabled = false;

  lastAckMs = now;
  lastTxMs = now;
  lastUpdateMs = now;
  lastPwmFrameMs = now;
  autoModeStartMs = now;

  targetPitch = 0.0f;
  targetYaw   = 0.0f;

  currentPitch = 0.0f;
  currentYaw   = 0.0f;

  frameIndex = 0;
  frameActive = false;
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

  updateTargetFromIBUS();
  applyMotion(dt);

  processSerial(now);
  updateWDT(now);

  // ------------------------------------------
  // AUTO MODE
  // ------------------------------------------
  if (mode == GimbalOutputMode::MODE_AUTO)
  {
    if ((now - autoModeStartMs) < 3000UL)
    {
      sendBinaryControl(
        (int16_t)(currentPitch * 100.0f),
        (int16_t)(currentYaw   * 100.0f));
    }
    else
    {
      if (now - lastAckMs > 1000UL)
      {
        sendPwmControl(now);
      }
      else
      {
        sendBinaryControl(
          (int16_t)(currentPitch * 100.0f),
          (int16_t)(currentYaw   * 100.0f));
      }
    }

    return;
  }

  // ------------------------------------------
  // SERIAL MODE
  // ------------------------------------------
  if (mode == GimbalOutputMode::MODE_SERIAL)
  {
    if (now - lastTxMs >= TX_PERIOD_MS)
    {
      sendBinaryControl(
        (int16_t)(currentPitch * 100.0f),
        (int16_t)(currentYaw   * 100.0f));

      lastTxMs = now;
    }

    return;
  }

  // ------------------------------------------
  // PWM MODE
  // ------------------------------------------
  sendPwmControl(now);
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
  lastTxMs = now;
  autoModeStartMs = now;
}

// ======================================================
// MODE
// ======================================================
void Storm32Controller::setOutputMode(GimbalOutputMode m)
{
  mode = m;
  cfg.outputMode = (uint8_t)m;
  saveConfig();
}

GimbalOutputMode Storm32Controller::getOutputMode() const
{
  return mode;
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
  float slew =
    (state == Storm32State::DEGRADED)
      ? cfg.slewDegraded
      : cfg.slewNormal;

  float step = slew * dt;

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

  return cur + ((d > 0) ? maxStep : -maxStep);
}

// ======================================================
// MAP RC
// ======================================================
float Storm32Controller::mapRCtoDeg(uint16_t rc,
                                    float limit,
                                    bool invert)
{
  float v =
    ((float)rc - 1500.0f) / 500.0f * limit;

  if (invert)
    v = -v;

  return v;
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
  pkt[4] = pitch >> 8;

  pkt[5] = yaw & 0xFF;
  pkt[6] = yaw >> 8;

  uint16_t crc = crc16_ccitt(pkt, 7);

  pkt[7] = crc & 0xFF;
  pkt[8] = crc >> 8;

  stormSerial.write(pkt, 9);
}

// ======================================================
// PWM TX
// ======================================================
void Storm32Controller::sendPwmControl(uint32_t now)
{
  if (now - lastPwmFrameMs < PWM_FRAME_MS)
    return;

  lastPwmFrameMs = now;

  uint16_t usPitch =
    degToUs(currentPitch, cfg.pitchLimitDeg);

  uint16_t usYaw =
    degToUs(currentYaw, cfg.yawLimitDeg);

  digitalWrite(GIMBAL_PWM_PIN_PITCH, HIGH);
  delayMicroseconds(usPitch);
  digitalWrite(GIMBAL_PWM_PIN_PITCH, LOW);

  digitalWrite(GIMBAL_PWM_PIN_YAW, HIGH);
  delayMicroseconds(usYaw);
  digitalWrite(GIMBAL_PWM_PIN_YAW, LOW);
}

// ======================================================
// DEG TO PWM
// ======================================================
uint16_t Storm32Controller::degToUs(float deg,
                                    float limit)
{
  if (limit < 1.0f)
    limit = 1.0f;

  deg = constrain(deg, -limit, limit);

  long us = map((long)(deg * 100),
                (long)(-limit * 100),
                (long)( limit * 100),
                1000,
                2000);

  return (uint16_t)us;
}

// ======================================================
// SERIAL RX
// ======================================================
void Storm32Controller::processSerial(uint32_t now)
{
  while (stormSerial.available())
  {
    parseByte(stormSerial.read(), now);
  }
}

void Storm32Controller::parseByte(uint8_t b,
                                  uint32_t now)
{
  lastAckMs = now;

  if (state == Storm32State::DEGRADED ||
      state == Storm32State::LOST)
  {
    state = Storm32State::OK;
  }

  if (!frameActive)
  {
    if (b == STORM32_START_BYTE)
    {
      frameActive = true;
      frameIndex = 0;
      frameBuf[frameIndex++] = b;
    }

    return;
  }

  frameBuf[frameIndex++] = b;

  if (frameIndex >= STORM32_MAX_FRAME)
  {
    frameActive = false;
    frameIndex = 0;
  }
}

// ======================================================
// WATCHDOG
// ======================================================
void Storm32Controller::updateWDT(uint32_t now)
{
  uint32_t dt = now - lastAckMs;

  if (state == Storm32State::OK &&
      dt > cfg.ackTimeout1)
  {
    state = Storm32State::DEGRADED;
  }

  if (state == Storm32State::DEGRADED &&
      dt > cfg.ackTimeout2)
  {
    state = Storm32State::LOST;
  }

  if (state == Storm32State::LOST &&
      dt > cfg.ackTimeout3)
  {
    state = Storm32State::EMERGENCY;
  }
}

// ======================================================
// OFF
// ======================================================
void Storm32Controller::sendDriveOff()
{
  if (mode == GimbalOutputMode::MODE_PWM)
  {
    digitalWrite(GIMBAL_PWM_PIN_PITCH, LOW);
    digitalWrite(GIMBAL_PWM_PIN_YAW, LOW);
    return;
  }

  sendBinaryControl(0, 0);
}

// ======================================================
// CRC
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
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }

  return crc;
}

// ======================================================
// EEPROM
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

void Storm32Controller::saveConfig()
{
  cfg.magic = STORM32_MAGIC;
  EEPROM.put(EEPROM_STORM32_BASE, cfg);
}

void Storm32Controller::setDefaultConfig()
{
  cfg.invertPitch = false;
  cfg.invertYaw   = false;

  cfg.pitchLimitDeg = 30.0f;
  cfg.yawLimitDeg   = 45.0f;

  cfg.slewNormal   = 60.0f;
  cfg.slewDegraded = 20.0f;

  cfg.ackTimeout1 = 400;
  cfg.ackTimeout2 = 900;
  cfg.ackTimeout3 = 2000;

  cfg.outputMode =
    (uint8_t)GimbalOutputMode::MODE_AUTO;

  cfg.magic = STORM32_MAGIC;
}

// ======================================================
// STATUS
// ======================================================
Storm32State Storm32Controller::getState() const
{
  return state;
}

bool Storm32Controller::isLocked() const
{
  return hardDisabled ||
         !systemEnabled ||
         state == Storm32State::INIT ||
         state == Storm32State::EMERGENCY;
}

