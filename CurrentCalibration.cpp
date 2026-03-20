// ============================================================================
// CurrentCalibration.cpp (HARDENED - STABLE + SAFE OFFSET)
// ============================================================================

#include <Arduino.h>
#include "CurrentCalibration.h"
#include "GlobalState.h"

// ======================================================
// CONFIG
// ======================================================

constexpr uint8_t CAL_SAMPLE_N = 32;
constexpr uint16_t CAL_TIMEOUT_MS = 400;

// ต้องนิ่งจริงก่อนนับ sample
constexpr float CAL_STABLE_THRESHOLD_A = 1.0f;

constexpr float REZERO_THRESHOLD_A = 1.2f;
constexpr float REZERO_ALPHA = 0.01f;   // 🔴 ลดลง กัน drift

// ======================================================
// INTERNAL STATE
// ======================================================

static uint8_t calCount = 0;
static float calAccum[4] = {0,0,0,0};
static uint32_t calStart_ms = 0;
static bool calDone = false;

// ======================================================
// RESET (เรียกตอน boot หรือ fault)
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
// NON-BLOCKING CALIBRATION
// ======================================================

bool calibrateCurrentOffsetNonBlocking(uint32_t now)
{
  if (calDone) return true;

  if (calStart_ms == 0)
    calStart_ms = now;

  // timeout กันค้าง
  if (now - calStart_ms > CAL_TIMEOUT_MS)
  {
    calDone = true;
    return true;
  }

  // ==================================================
  // 🔴 ต้องนิ่งจริงก่อนนับ
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
      currentOffset[i] = calAccum[i] / CAL_SAMPLE_N;
    }

    calDone = true;

#if DEBUG_SERIAL
    Serial.println(F("[CAL] CURRENT OFFSET DONE"));
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
  // ต้อง idle จริง (ทั้ง throttle + output)
  if (abs((int)rcThrottle - 1500) > 40)
    return;

  if (abs(curL) > 20 || abs(curR) > 20)
    return;

  for (uint8_t i = 0; i < 4; i++)
  {
    float err = curA[i] - currentOffset[i];

    // 🔴 ต้องใกล้จริง และไม่ drift
    if (fabs(err) < REZERO_THRESHOLD_A)
    {
      currentOffset[i] =
        currentOffset[i] * (1.0f - REZERO_ALPHA) +
        curA[i] * REZERO_ALPHA;
    }
  }
}

