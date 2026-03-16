// ============================================================================
// CommsManager.cpp
// จัดการระบบสื่อสาร iBUS และ RC watchdog
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
  // ==================================================
  // IBUS LOST COUNTER
  // ==================================================

  static uint8_t ibusLostCnt = 0;

  // ==================================================
  // RC HEARTBEAT WATCHDOG
  // ==================================================

  static uint32_t lastFrame_ms = 0;

  constexpr uint16_t RC_FRAME_MAX_INTERVAL = 50;

  // ==================================================
  // IBUS BYTE PARSING
  // ==================================================

  bool gotByte = false;

  // จำกัดจำนวน byte ต่อ loop เพื่อไม่ให้กิน CPU
  uint8_t parseBudget = 32;

  while (Serial1.available() && parseBudget--)
  {
    ibus.loop();

    lastIbusByte_ms = now;
    gotByte = true;
  }

  // ==================================================
  // FRAME HEARTBEAT UPDATE
  // ==================================================

  if (gotByte)
  {
    lastFrame_ms = now;
  }

  // ==================================================
  // RC HEARTBEAT TIMEOUT
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
  // RECOVERY : BYTE RETURNS
  // ==================================================

  if (gotByte)
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
  // SOFT TIMEOUT
  // ==================================================

  if (now - lastIbusByte_ms > 100)
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
  // HARD TIMEOUT
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
  // COMMS HEALTHY
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

  // ==================================================
  // IBUS LOST GUARD
  // ==================================================

  if (ibusCommLost)
    return;

  // ==================================================
  // READ CHANNELS
  // ==================================================

  uint16_t thr = ibus.readChannel(CH_THROTTLE);
  uint16_t str = ibus.readChannel(CH_STEER);
  uint16_t eng = ibus.readChannel(CH_ENGINE);
  uint16_t ign = ibus.readChannel(CH_IGNITION);
  uint16_t sta = ibus.readChannel(CH_STARTER);

  // ==================================================
  // PLAUSIBILITY GUARD
  // ==================================================

  if (thr < 900 || thr > 2100) return;
  if (str < 900 || str > 2100) return;
  if (eng < 900 || eng > 2100) return;
  if (ign < 900 || ign > 2100) return;
  if (sta < 900 || sta > 2100) return;

  // ==================================================
  // RC SPIKE FILTER
  // ป้องกันค่า RC กระโดดผิดปกติ
  // ==================================================

  static uint16_t lastThr = 1500;
  static uint16_t lastStr = 1500;

  constexpr uint16_t RC_SPIKE_LIMIT = 300;

  if (abs((int)thr - (int)lastThr) > RC_SPIKE_LIMIT)
    thr = lastThr;

  if (abs((int)str - (int)lastStr) > RC_SPIKE_LIMIT)
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
  // RC FREEZE DETECTION
  // ==================================================

  static uint32_t lastChange_ms = 0;

  constexpr uint16_t RC_FREEZE_DEADBAND = 4;
  constexpr uint32_t RC_FREEZE_TIMEOUT_MS = 1500;

  bool changed = false;

  if (abs((int)thr - (int)lastThr) > RC_FREEZE_DEADBAND)
    changed = true;

  if (abs((int)str - (int)lastStr) > RC_FREEZE_DEADBAND)
    changed = true;

  if (changed)
    lastChange_ms = now;

  // update history
  lastThr = thr;
  lastStr = str;

  // ==================================================
  // FREEZE TIMEOUT
  // ==================================================

  if (now - lastChange_ms > RC_FREEZE_TIMEOUT_MS)
  {

#if DEBUG_SERIAL
    Serial.println(F("[RC] FREEZE DETECTED"));
#endif

    latchFault(FaultCode::COMMS_TIMEOUT);
  }
}