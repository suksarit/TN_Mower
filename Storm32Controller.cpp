#include "Storm32Controller.h"
#include "GlobalState.h"

// =================================================
// SINGLETON ACCESSOR
// =================================================
Storm32Controller& getGimbal()
{
  static Storm32Controller gimbal(Serial3, ibus);
  return gimbal;
}

// =================================================
// CONSTRUCTOR
// =================================================
Storm32Controller::Storm32Controller(HardwareSerial& serial,
                                     IBusBM& ib)
  : stormSerial(serial),
    ibus(ib)
{
}

// =================================================
// BEGIN
// =================================================
void Storm32Controller::begin()
{
  stormSerial.begin(115200);

  loadConfig();

  uint32_t now = millis();

  state = Storm32State::INIT;

  lastAckMs = now;
  lastTxMs = now;
  lastUpdateMs = now;
  lastPhaseSwitchMs = now;

  targetPitch = 0.0f;
  targetYaw   = 0.0f;

  currentPitch = 0.0f;
  currentYaw   = 0.0f;

  hardDisabled = false;
  systemEnabled = false;

  frameIndex = 0;
  frameLength = 0;
  frameActive = false;
}

// =================================================
// FORCE OFF
// =================================================
void Storm32Controller::forceOff()
{
  hardDisabled = true;
  state = Storm32State::EMERGENCY;
  sendDriveOff();
}

// =================================================
// HARD DISABLE
// =================================================
void Storm32Controller::hardDisable(bool enable)
{
  hardDisabled = enable;

  if (hardDisabled)
  {
    state = Storm32State::EMERGENCY;
    sendDriveOff();
  }
}

// =================================================
// SYSTEM ENABLE
// =================================================
void Storm32Controller::setSystemEnabled(bool enable)
{
  systemEnabled = enable;

  if (!systemEnabled)
  {
    if (!hardDisabled)
      state = Storm32State::INIT;

    sendDriveOff();
  }
}

// =================================================
// CLEAR EMERGENCY
// =================================================
void Storm32Controller::clearEmergency()
{
  if (hardDisabled)
    return;

  if (state == Storm32State::EMERGENCY ||
      state == Storm32State::INIT ||
      state == Storm32State::LOST)
  {
    uint32_t now = millis();

    state = Storm32State::OK;

    lastAckMs = now;
    lastTxMs = now;
    lastUpdateMs = now;

    targetPitch = 0.0f;
    targetYaw   = 0.0f;

    currentPitch = 0.0f;
    currentYaw   = 0.0f;
  }
}

// =================================================
// UPDATE
// =================================================
void Storm32Controller::update(uint32_t now)
{
  uint32_t dtMs = now - lastUpdateMs;
  lastUpdateMs = now;

  float dt = dtMs * 0.001f;

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

  // ---------------------------------------------
  // Scheduler phase
  // ---------------------------------------------
  if (now - lastPhaseSwitchMs >=
      (CONTROL_WINDOW_MS + COMM_WINDOW_MS))
  {
    lastPhaseSwitchMs = now;
  }

  uint32_t phaseTime = now - lastPhaseSwitchMs;
  bool controlPhase = (phaseTime < CONTROL_WINDOW_MS);

  if (controlPhase)
  {
    updateTargetFromIBUS();
    applyMotion(dt);
  }
  else
  {
    processSerial(now);
  }

  updateWDT(now);

  if (state == Storm32State::EMERGENCY)
  {
    sendDriveOff();
    return;
  }

  // ---------------------------------------------
  // TX command
  // ---------------------------------------------
  if (now - lastTxMs >= TX_PERIOD_MS)
  {
    int16_t p = (int16_t)(currentPitch * 100.0f);
    int16_t y = (int16_t)(currentYaw   * 100.0f);

    sendBinaryControl(p, y);
    lastTxMs = now;
  }
}

// =================================================
// PROCESS SERIAL
// =================================================
void Storm32Controller::processSerial(uint32_t now)
{
  uint32_t startUs = micros();
  uint8_t count = 0;

  while (stormSerial.available())
  {
    if (++count > 24)
      break;

    if (micros() - startUs > SERIAL_BUDGET_US)
      break;

    uint8_t b = stormSerial.read();
    parseByte(b, now);
  }
}

// =================================================
// UPDATE TARGET FROM IBUS
// =================================================
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

// =================================================
// APPLY MOTION
// =================================================
void Storm32Controller::applyMotion(float dt)
{
  float slew =
    (state == Storm32State::DEGRADED)
      ? cfg.slewDegraded
      : cfg.slewNormal;

  float maxStep = slew * dt;

  currentPitch =
    stepToward(currentPitch,
               targetPitch,
               maxStep);

  currentYaw =
    stepToward(currentYaw,
               targetYaw,
               maxStep);

  currentPitch =
    constrain(currentPitch,
              -cfg.pitchLimitDeg,
               cfg.pitchLimitDeg);

  currentYaw =
    constrain(currentYaw,
              -cfg.yawLimitDeg,
               cfg.yawLimitDeg);
}

// =================================================
// STEP TOWARD
// =================================================
float Storm32Controller::stepToward(float cur,
                                    float tgt,
                                    float maxStep)
{
  float diff = tgt - cur;

  if (fabs(diff) < 0.02f)
    return cur;

  if (fabs(diff) <= maxStep)
    return tgt;

  return cur + ((diff > 0.0f) ? maxStep : -maxStep);
}

// =================================================
// MAP RC TO DEG
// =================================================
float Storm32Controller::mapRCtoDeg(uint16_t rc,
                                    float limit,
                                    bool invert)
{
  rc = constrain(rc, 1000, 2000);

  float v =
    ((float)rc - 1500.0f) / 500.0f * limit;

  if (invert)
    v = -v;

  return v;
}

// =================================================
// WATCHDOG
// =================================================
void Storm32Controller::updateWDT(uint32_t now)
{
  uint32_t dt = now - lastAckMs;

  switch (state)
  {
    case Storm32State::OK:
      if (dt > cfg.ackTimeout1)
        state = Storm32State::DEGRADED;
      break;

    case Storm32State::DEGRADED:
      if (dt > cfg.ackTimeout2)
        state = Storm32State::LOST;
      break;

    case Storm32State::LOST:
      if (dt > cfg.ackTimeout3)
        state = Storm32State::EMERGENCY;
      break;

    default:
      break;
  }
}

// =================================================
// SEND CONTROL
// =================================================
void Storm32Controller::sendBinaryControl(int16_t pitch,
                                          int16_t yaw)
{
  int16_t maxP =
    (int16_t)(cfg.pitchLimitDeg * 100.0f);

  int16_t maxY =
    (int16_t)(cfg.yawLimitDeg * 100.0f);

  pitch = constrain(pitch, -maxP, maxP);
  yaw   = constrain(yaw,   -maxY, maxY);

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

// =================================================
// DRIVE OFF
// =================================================
void Storm32Controller::sendDriveOff()
{
  sendBinaryControl(0, 0);
}

// =================================================
// PARSER (SAFE STUB)
// =================================================
void Storm32Controller::parseByte(uint8_t b,
                                  uint32_t now)
{
  // มี byte กลับมา ถือว่าสื่อสารได้ระดับหนึ่ง
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

// =================================================
// CRC16
// =================================================
uint16_t Storm32Controller::crc16_ccitt(const uint8_t* data,
                                        uint8_t len)
{
  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;

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

// =================================================
// EEPROM
// =================================================
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

  cfg.magic = STORM32_MAGIC;
}

// =================================================
// STATUS
// =================================================
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

