// ============================================================================
// DriveRamp.cpp (CLEAN + FIXED SIGNATURE + SAFE)
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "HardwareConfig.h"
#include "SystemTypes.h"
#include "SensorManager.h"
#include "DriveRamp.h"

// ==================================================
// LOCAL HELPER (RAMP FUNCTION)
// ==================================================
static int16_t ramp(int16_t current, float target, int16_t step)
{
  if (current < target)
    return min((int16_t)(current + step), (int16_t)target);

  if (current > target)
    return max((int16_t)(current - step), (int16_t)target);

  return current;
}

// ============================================================================
// MAIN RAMP FUNCTION
// ============================================================================

void updateDriveRamp(float &finalTargetL,
                     float &finalTargetR)
{
  // ==================================================
  // CLAMP INPUT
  // ==================================================
  finalTargetL = constrain(finalTargetL, -PWM_TOP, PWM_TOP);
  finalTargetR = constrain(finalTargetR, -PWM_TOP, PWM_TOP);

  uint32_t now = millis();

  // ==================================================
  // ERROR SIZE
  // ==================================================
  int16_t errMax =
    max(abs((int16_t)(finalTargetL - curL)),
        abs((int16_t)(finalTargetR - curR)));

  float step;

  if (driveState == DriveState::LIMP)
    step = 2.0f;
  else if (errMax > 700)
    step = 3.0f;
  else if (errMax > 350)
    step = 4.0f;
  else
    step = 6.0f;

  // ==================================================
  // SAFE REVERSE CONTROL
  // ==================================================
  constexpr int16_t REVERSE_SAFE_PWM = 120;

  bool reverseL =
    (curL > 0 && finalTargetL < 0) ||
    (curL < 0 && finalTargetL > 0);

  bool reverseR =
    (curR > 0 && finalTargetR < 0) ||
    (curR < 0 && finalTargetR > 0);

  if (reverseL) {
    if (abs(curL) > REVERSE_SAFE_PWM || now < revBlockUntilL)
      finalTargetL = 0;
    else {
      revBlockUntilL = now + REVERSE_DEADTIME_MS;
      finalTargetL = 0;
    }
  }

  if (reverseR) {
    if (abs(curR) > REVERSE_SAFE_PWM || now < revBlockUntilR)
      finalTargetR = 0;
    else {
      revBlockUntilR = now + REVERSE_DEADTIME_MS;
      finalTargetR = 0;
    }
  }

  // ==================================================
  // CURRENT LOAD SCALE
  // ==================================================
  constexpr float LOAD_CURRENT_HIGH = 40.0f;
  constexpr float LOAD_CURRENT_MAX  = 65.0f;

  float curLoad = max(curLeft(), curRight());

  float loadScale = 1.0f;

  if (curLoad > LOAD_CURRENT_HIGH) {
    loadScale =
      1.0f - ((curLoad - LOAD_CURRENT_HIGH) /
              (LOAD_CURRENT_MAX - LOAD_CURRENT_HIGH));

    loadScale = constrain(loadScale, 0.35f, 1.0f);
  }

  // ==================================================
  // TERRAIN SCALE
  // ==================================================
  constexpr float TERRAIN_DRAG_HIGH = 50.0f;

  float terrainScale = 1.0f;

  if (terrainDragAvg > TERRAIN_DRAG_HIGH) {
    terrainScale =
      1.0f - (terrainDragAvg - TERRAIN_DRAG_HIGH) * 0.02f;

    terrainScale = constrain(terrainScale, 0.45f, 1.0f);
  }

  // ==================================================
  // FINAL STEP
  // ==================================================
  step *= loadScale;
  step *= terrainScale;

  if (step < 1.0f)
    step = 1.0f;

  int16_t stepInt = (int16_t)step;

  // ==================================================
  // APPLY RAMP
  // ==================================================
  curL = ramp(curL, finalTargetL, stepInt);
  curR = ramp(curR, finalTargetR, stepInt);

  // ==================================================
  // LOW SPEED SMOOTH
  // ==================================================
  constexpr int16_t LOW_PWM = 120;

  if (abs(curL) < LOW_PWM)
    curL = (curL * 9) / 10;

  if (abs(curR) < LOW_PWM)
    curR = (curR * 9) / 10;

  // ==================================================
  // TRACTION BALANCE
  // ==================================================
  constexpr int16_t MAX_WHEEL_DIFF = 300;

  int16_t diff = curL - curR;

  if (diff > MAX_WHEEL_DIFF)
    curL = curR + MAX_WHEEL_DIFF;

  if (diff < -MAX_WHEEL_DIFF)
    curR = curL + MAX_WHEEL_DIFF;

  // ==================================================
  // FINAL CLAMP
  // ==================================================
  curL = constrain(curL, -PWM_TOP, PWM_TOP);
  curR = constrain(curR, -PWM_TOP, PWM_TOP);
}