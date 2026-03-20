// ============================================================================
// CommsManager.cpp (HARDENED - REAL FRAME + NO FALSE FAULT)
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
  static uint32_t lastFrame_ms = 0;

  constexpr uint16_t RC_FRAME_MAX_INTERVAL = 60;

  // ==================================================
  // PARSE IBUS (adaptive budget)
  // ==================================================

  uint8_t available = Serial1.available();
  uint8_t parseBudget = constrain(available, 8, 64);

  bool frameValid = false;

  while (Serial1.available() && parseBudget--)
  {
    ibus.loop();

    lastIbusByte_ms = now;

    // 🔴 ใช้ channel validity เป็น frame indicator
    uint16_t test = ibus.readChannel(CH_THROTTLE);

    if (test >= 900 && test <= 2100)
      frameValid = true;
  }

  // ==================================================
  // FRAME HEARTBEAT (VALID ONLY)
  // ==================================================

  if (frameValid)
  {
    lastFrame_ms = now;
  }

  // ==================================================
  // HEARTBEAT TIMEOUT
  // ==================================================

  if (now - lastFrame_ms > RC_FRAME_MAX_INTERVAL)
  {
#if DEBUG_SERIAL
    Serial.println(F("[RC] HEARTBEAT LOST"));
#endif
    latchFault(FaultCode::COMMS_TIMEOUT);
    return;
  }

  // ==================================================
  // RECOVERY
  // ==================================================

  if (frameValid)
  {
    if (ibusCommLost)
    {
#if DEBUG_SERIAL
      Serial.println(F("[IBUS] RECOVERED"));
#endif
      requireIbusConfirm = true;
      ibusRecoverStart_ms = now;
    }

    ibusLostCnt = 0;
    ibusCommLost = false;

    wdComms.lastUpdate_ms = now;
  }

  // ==================================================
  // SOFT LOST
  // ==================================================

  if (now - lastIbusByte_ms > 120)
  {
    if (++ibusLostCnt >= 3)
    {
      if (!ibusCommLost)
      {
#if DEBUG_SERIAL
        Serial.println(F("[IBUS] SOFT LOST"));
#endif
        ibusCommLost = true;
        requireIbusConfirm = true;
      }
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
    Serial.println(F("[IBUS] HARD LOST -> FAULT"));
#endif
    latchFault(FaultCode::IBUS_LOST);
    return;
  }

  // ==================================================
  // HEALTHY
  // ==================================================

  if (!ibusCommLost)
  {
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

  uint16_t thr = ibus.readChannel(CH_THROTTLE);
  uint16_t str = ibus.readChannel(CH_STEER);
  uint16_t eng = ibus.readChannel(CH_ENGINE);
  uint16_t ign = ibus.readChannel(CH_IGNITION);
  uint16_t sta = ibus.readChannel(CH_STARTER);

  // ==================================================
  // PLAUSIBILITY
  // ==================================================

  if (thr < 900 || thr > 2100) return;
  if (str < 900 || str > 2100) return;
  if (eng < 900 || eng > 2100) return;
  if (ign < 900 || ign > 2100) return;
  if (sta < 900 || sta > 2100) return;

  // ==================================================
  // SPIKE FILTER
  // ==================================================

  static uint16_t lastThr = 1500;
  static uint16_t lastStr = 1500;

  constexpr uint16_t SPIKE = 300;

  if (abs((int)thr - (int)lastThr) > SPIKE)
    thr = lastThr;

  if (abs((int)str - (int)lastStr) > SPIKE)
    str = lastStr;

  // ==================================================
  // UPDATE CACHE
  // ==================================================

  rcThrottle = thr;
  rcSteer    = str;
  rcEngine   = eng;
  rcIgnition = ign;
  rcStarter  = sta;

  // ==================================================
  // 🔴 FREEZE DETECTION (IMPROVED)
  // ==================================================

  static uint32_t lastChange_ms = 0;

  constexpr uint16_t DB = 4;
  constexpr uint32_t TIMEOUT = 1500;

  bool changed =
    abs((int)thr - (int)lastThr) > DB ||
    abs((int)str - (int)lastStr) > DB ||
    abs((int)eng - 1500) > 200 ||
    abs((int)ign - 1500) > 200;

  if (changed)
    lastChange_ms = now;

  lastThr = thr;
  lastStr = str;

  if (now - lastChange_ms > TIMEOUT)
  {
#if DEBUG_SERIAL
    Serial.println(F("[RC] FREEZE DETECTED"));
#endif
    latchFault(FaultCode::COMMS_TIMEOUT);
  }
}

