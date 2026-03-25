// ============================================================================
// CommsManager.cpp 
// ============================================================================

#include <Arduino.h>
#include <IBusBM.h>

#include "SystemTypes.h"
#include "GlobalState.h"
#include "HardwareConfig.h"

#include "FaultManager.h"
#include "CommsManager.h"

// ============================================================================
// UPDATE COMMUNICATION STATE
// ============================================================================
void updateComms(uint32_t now)
{
  static uint8_t ibusLostCnt = 0;
  static uint8_t frameLostCnt = 0;
  static bool frameInitialized = false;

  constexpr uint16_t RC_FRAME_MAX_INTERVAL = 60;
  constexpr uint8_t PARSE_BUDGET = 24;

  uint8_t parseBudget = PARSE_BUDGET;

  bool frameValid = false;

  uint16_t thr = 1500;
  uint16_t str = 1500;

  // ==================================================
  // PARSE
  // ==================================================
  while (Serial1.available() && parseBudget--)
  {
    ibus.loop();
    lastIbusByte_ms = now;

    thr = ibus.readChannel(CH_THROTTLE);
    str = ibus.readChannel(CH_STEER);

    // 🔴 validate multi-channel
    if (thr >= 900 && thr <= 2100 &&
        str >= 900 && str <= 2100)
    {
      frameValid = true;
    }
  }

  // ==================================================
  // HEARTBEAT
  // ==================================================
  if (frameValid)
  {
    rcLastFrame_ms = now;
    frameInitialized = true;
  }

  // ==================================================
  // FRAME LOST
  // ==================================================
  if (frameInitialized &&
      (now - rcLastFrame_ms > RC_FRAME_MAX_INTERVAL))
  {
    if (++frameLostCnt >= 3)
    {
#if DEBUG_SERIAL
      Serial.println(F("[RC] TIMEOUT"));
#endif
      requestFault(FaultCode::COMMS_TIMEOUT);
      return;
    }
  }
  else
  {
    frameLostCnt = 0;
  }

  // ==================================================
  // SOFT LOST
  // ==================================================
  if (now - lastIbusByte_ms > 120)
  {
    if (++ibusLostCnt >= 3)
    {
#if DEBUG_SERIAL
      Serial.println(F("[IBUS] SOFT LOST"));
#endif
      ibusCommLost = true;
      requireIbusConfirm = true;
    }
  }
  else
  {
    ibusLostCnt = 0;
  }

  // ==================================================
  // HARD LOST
  // ==================================================
  if (now - lastIbusByte_ms > IBUS_TIMEOUT_MS)
  {
#if DEBUG_SERIAL
    Serial.println(F("[IBUS] HARD LOST"));
#endif
    requestFault(FaultCode::IBUS_LOST);
    return;
  }

  // ==================================================
  // RECOVERY
  // ==================================================
  if (frameValid)
  {
    ibusCommLost = false;
    wdComms.lastUpdate_ms = now;
  }
}

// ============================================================================
// UPDATE RC CACHE
// ============================================================================
void updateRcCache()
{
  uint32_t now = millis();

  if (ibusCommLost)
    return;

  constexpr uint32_t FREEZE_TIMEOUT = 150;

  if (now - rcLastFrame_ms > FREEZE_TIMEOUT)
  {
#if DEBUG_SERIAL
    Serial.println(F("[RC] FREEZE"));
#endif
    requestFault(FaultCode::COMMS_TIMEOUT);
    return;
  }

  uint16_t thr = ibus.readChannel(CH_THROTTLE);
  uint16_t str = ibus.readChannel(CH_STEER);
  uint16_t eng = ibus.readChannel(CH_ENGINE);
  uint16_t ign = ibus.readChannel(CH_IGNITION);
  uint16_t sta = ibus.readChannel(CH_STARTER);

  // ==================================================
  // VALIDATION
  // ==================================================
  bool valid =
    (thr >= 900 && thr <= 2100) &&
    (str >= 900 && str <= 2100) &&
    (eng >= 900 && eng <= 2100) &&
    (ign >= 900 && ign <= 2100) &&
    (sta >= 900 && sta <= 2100);

  if (!valid)
  {
#if DEBUG_SERIAL
    Serial.println(F("[RC] INVALID"));
#endif
    requestFault(FaultCode::COMMS_TIMEOUT);
    return;
  }

  // ==================================================
  // RATE LIMIT
  // ==================================================
  static uint16_t lastThr = 1500;
  static uint16_t lastStr = 1500;

  constexpr int MAX_STEP = 120;

  int dThr = (int)thr - (int)lastThr;
  int dStr = (int)str - (int)lastStr;

  if (dThr > MAX_STEP) dThr = MAX_STEP;
  if (dThr < -MAX_STEP) dThr = -MAX_STEP;

  if (dStr > MAX_STEP) dStr = MAX_STEP;
  if (dStr < -MAX_STEP) dStr = -MAX_STEP;

  thr = lastThr + dThr;
  str = lastStr + dStr;

  // ==================================================
  // APPLY
  // ==================================================
  rcThrottle = thr;
  rcSteer    = str;
  rcEngine   = eng;
  rcIgnition = ign;
  rcStarter  = sta;

  lastThr = thr;
  lastStr = str;
}

