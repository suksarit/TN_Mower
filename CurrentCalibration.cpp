// CurrentCalibration.cpp

#include <Arduino.h>
#include "CurrentCalibration.h"
#include "GlobalState.h"

// ======================================================
// CONFIG
// ======================================================

constexpr uint8_t CAL_SAMPLE_N = 32;
constexpr uint16_t CAL_TIMEOUT_MS = 400;

constexpr float REZERO_THRESHOLD_A = 1.5f;   // ต่ำกว่า = ถือว่า idle
constexpr float REZERO_ALPHA = 0.02f;        // smoothing

// ======================================================
// INTERNAL STATE
// ======================================================

static uint8_t calCount = 0;
static float calAccum[4] = {0,0,0,0};
static uint32_t calStart_ms = 0;
static bool calDone = false;

// ======================================================
// NON-BLOCKING CALIBRATION
// เรียกจาก updateSystemState()
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

  // อ่านค่าปัจจุบัน (จาก SensorManager)
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
// IDLE AUTO RE-ZERO
// เรียกจาก taskDriveEvents()
// ======================================================
void idleCurrentAutoRezero(uint32_t now)
{
  // ต้องอยู่ในสถานะที่รถไม่วิ่ง
  if (abs((int)rcThrottle - 1500) > 50)
    return;

  for (uint8_t i = 0; i < 4; i++)
  {
    float err = curA[i] - currentOffset[i];

    if (abs(err) < REZERO_THRESHOLD_A)
    {
      currentOffset[i] =
        currentOffset[i] * (1.0f - REZERO_ALPHA) +
        curA[i] * REZERO_ALPHA;
    }
  }
}



