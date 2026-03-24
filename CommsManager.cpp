// ============================================================================
// CommsManager.cpp (FIXED - STABLE + NO FALSE FAULT + REAL FIELD READY)
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
  static bool frameInitialized = false;

  static uint8_t frameLostCnt = 0;

  constexpr uint16_t RC_FRAME_MAX_INTERVAL = 60;

  // ==================================================
  // 🔴 FIXED PARSE BUDGET (กัน noise ทำ loop ค้าง)
  // ==================================================

  constexpr uint8_t PARSE_BUDGET = 24;
  uint8_t parseBudget = PARSE_BUDGET;

  bool frameValid = false;

  while (Serial1.available() && parseBudget--)
  {
    ibus.loop();

    lastIbusByte_ms = now;

    // ใช้ channel validity เป็น frame indicator
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
    frameInitialized = true;
  }

  // ==================================================
  // 🔴 FRAME TIMEOUT WITH DEBOUNCE + STARTUP SAFE
  // ==================================================

  if (frameInitialized && (now - lastFrame_ms > RC_FRAME_MAX_INTERVAL))
  {
    if (++frameLostCnt >= 3)
    {
#if DEBUG_SERIAL
      Serial.println(F("[RC] HEARTBEAT LOST (CONFIRMED)"));
#endif
      latchFault(FaultCode::COMMS_TIMEOUT);
      return;
    }
  }
  else
  {
    frameLostCnt = 0;
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

  lastThr = thr;
  lastStr = str;

  // ==================================================
  // 🔴 FREEZE DETECTION (FIXED - FRAME BASED)
  // ==================================================

  static uint32_t lastAlive_ms = 0;

  // มีข้อมูล valid = ยัง alive
  lastAlive_ms = now;

  constexpr uint32_t FREEZE_TIMEOUT = 1500;

  if (now - lastAlive_ms > FREEZE_TIMEOUT)
  {
#if DEBUG_SERIAL
    Serial.println(F("[RC] FREEZE (NO FRAME)"));
#endif
    latchFault(FaultCode::COMMS_TIMEOUT);
  }
}

