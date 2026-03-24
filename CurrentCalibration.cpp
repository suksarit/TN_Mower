// ============================================================================
// CurrentCalibration.cpp (FINAL - SAFE CAL + ANTI-DRIFT)
// ============================================================================

#include <Arduino.h>
#include "CurrentCalibration.h"
#include "GlobalState.h"

// ======================================================
// CONFIG
// ======================================================

constexpr uint8_t CAL_SAMPLE_N = 32;
constexpr uint16_t CAL_TIMEOUT_MS = 400;

constexpr float CAL_STABLE_THRESHOLD_A = 1.0f;

// 🔴 sanity limit (กัน offset เพี้ยน)
constexpr float OFFSET_MAX_ABS = 5.0f;

// 🔴 auto re-zero
constexpr float REZERO_THRESHOLD_A = 1.0f;
constexpr float REZERO_ALPHA = 0.005f;

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
// NON-BLOCKING CALIBRATION (SAFE)
// ======================================================

bool calibrateCurrentOffsetNonBlocking(uint32_t now)
{
  if (calDone) return true;

  if (calStart_ms == 0)
    calStart_ms = now;

  // ==================================================
  // 🔴 TIMEOUT = FAIL SAFE (ไม่ใช้ค่าเพี้ยน)
  // ==================================================
  if (now - calStart_ms > CAL_TIMEOUT_MS)
  {
#if DEBUG_SERIAL
    Serial.println(F("[CAL] TIMEOUT -> SKIP OFFSET"));
#endif
    calDone = true;
    return true;
  }

  // ==================================================
  // ต้องนิ่งจริง
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
  // SAMPLE
  // ==================================================
  for (uint8_t i = 0; i < 4; i++)
  {
    calAccum[i] += curA[i];
  }

  calCount++;

  if (calCount >= CAL_SAMPLE_N)
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      float offset = calAccum[i] / CAL_SAMPLE_N;

      // 🔴 sanity check
      if (fabs(offset) < OFFSET_MAX_ABS)
        currentOffset[i] = offset;
      else
        currentOffset[i] = 0;  // reject
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
// IDLE AUTO RE-ZERO (SAFE)
// ======================================================

void idleCurrentAutoRezero(uint32_t now)
{
  // ต้อง throttle idle
  if (abs((int)rcThrottle - 1500) > 40)
    return;

  // 🔴 ต้อง current ต่ำจริง (ไม่ใช้ PWM)
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

