// ============================================================================
// CurrentCalibration.cpp (FINAL - ROBUST + FAULT SAFE + ANTI-DRIFT)
// ============================================================================

#include <Arduino.h>
#include "CurrentCalibration.h"
#include "GlobalState.h"
#include "FaultManager.h"   // 🔴 เพิ่ม

// ======================================================
// CONFIG
// ======================================================

constexpr uint8_t CAL_SAMPLE_N = 32;
constexpr uint16_t CAL_TIMEOUT_MS = 400;

constexpr float CAL_STABLE_THRESHOLD_A = 1.0f;

// 🔴 sanity limit (กัน offset เพี้ยน)
constexpr float OFFSET_MAX_ABS = 5.0f;

// 🔴 sensor sanity
constexpr float CURRENT_MIN_VALID = -100.0f;
constexpr float CURRENT_MAX_VALID = 100.0f;

// 🔴 clamp กัน spike
constexpr float CURRENT_CLAMP = 50.0f;

// 🔴 auto re-zero
constexpr float REZERO_THRESHOLD_A = 0.8f;
constexpr float REZERO_ALPHA = 0.003f;

// ======================================================
// INTERNAL STATE
// ======================================================

static uint8_t calCount = 0;
static float calAccum[4] = {0,0,0,0};
static uint32_t calStart_ms = 0;
static bool calDone = false;

// ======================================================
// RESET
// ======================================================

void resetCurrentCalibration()
{
  calCount = 0;
  calStart_ms = 0;
  calDone = false;

  for (uint8_t i = 0; i < 4; i++)
  {
    calAccum[i] = 0;
    currentOffset[i] = 0;
  }
}

// ======================================================
// NON-BLOCKING CALIBRATION (ROBUST)
// ======================================================

bool calibrateCurrentOffsetNonBlocking(uint32_t now)
{
  if (calDone) return true;

  if (calStart_ms == 0)
    calStart_ms = now;

  // ==================================================
  // 🔴 SENSOR SANITY CHECK
  // ==================================================
  for (uint8_t i = 0; i < 4; i++)
  {
    if (curA[i] < CURRENT_MIN_VALID || curA[i] > CURRENT_MAX_VALID)
    {
#if DEBUG_SERIAL
      Serial.println(F("[CAL] CURRENT SENSOR FAULT"));
#endif
      latchFault(FaultCode::SENSOR_TIMEOUT);
      calDone = true;
      return true;
    }
  }

  // ==================================================
  // 🔴 TIMEOUT (fail safe)
  // ==================================================
  if (now - calStart_ms > CAL_TIMEOUT_MS)
  {
#if DEBUG_SERIAL
    Serial.println(F("[CAL] TIMEOUT -> SKIP"));
#endif
    calDone = true;
    return true;
  }

  // ==================================================
  // 🔴 ต้องนิ่งจริง
  // ==================================================
  for (uint8_t i = 0; i < 4; i++)
  {
    if (fabs(curA[i]) > CAL_STABLE_THRESHOLD_A)
    {
      calCount = 0;

      for (uint8_t j = 0; j < 4; j++)
        calAccum[j] = 0;

      return false;
    }
  }

  // ==================================================
  // 🔴 SAMPLE + CLAMP
  // ==================================================
  for (uint8_t i = 0; i < 4; i++)
  {
    float v = curA[i];

    if (v > CURRENT_CLAMP) v = CURRENT_CLAMP;
    if (v < -CURRENT_CLAMP) v = -CURRENT_CLAMP;

    calAccum[i] += v;
  }

  calCount++;

  // ==================================================
  // COMPLETE
  // ==================================================
  if (calCount >= CAL_SAMPLE_N)
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      float offset = calAccum[i] / CAL_SAMPLE_N;

      // 🔴 sanity check offset
      if (fabs(offset) < OFFSET_MAX_ABS)
      {
        currentOffset[i] = offset;
      }
      else
      {
        currentOffset[i] = 0;

#if DEBUG_SERIAL
        Serial.println(F("[CAL] OFFSET REJECTED"));
#endif
      }
    }

    calDone = true;

#if DEBUG_SERIAL
    Serial.println(F("[CAL] OFFSET DONE"));
#endif

    return true;
  }

  return false;
}

// ======================================================
// IDLE AUTO RE-ZERO (ANTI-DRIFT SAFE)
// ======================================================

void idleCurrentAutoRezero(uint32_t now)
{
  (void)now;

  // ต้อง throttle idle จริง
  if (abs((int)rcThrottle - 1500) > 40)
    return;

  // 🔴 ต้อง current ต่ำทุกช่อง
  for (uint8_t i = 0; i < 4; i++)
  {
    if (fabs(curA[i]) > 2.0f)
      return;  // มีโหลด → ห้าม re-zero
  }

  // ==================================================
  // APPLY RE-ZERO
  // ==================================================
  for (uint8_t i = 0; i < 4; i++)
  {
    float err = curA[i] - currentOffset[i];

    if (fabs(err) < REZERO_THRESHOLD_A)
    {
      currentOffset[i] =
        currentOffset[i] * (1.0f - REZERO_ALPHA) +
        curA[i] * REZERO_ALPHA;
    }
  }
}

