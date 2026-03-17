// ============================================================================
// CurrentController.cpp (CLEAN + SAFE PI LOOP)
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "SensorManager.h"
#include "CurrentController.h"

// ==================================================
// CURRENT LOOP STATE (LOCAL ONLY)
// ==================================================
static float iErrL = 0.0f;
static float iErrR = 0.0f;

// ============================================================================
// APPLY CURRENT LOOP (PI CONTROL)
// แก้ค่า targetL / targetR โดยตรง
// ============================================================================

void applyCurrentLoop(float &targetL, float &targetR)
{
  // --------------------------------------------------
  // CONFIG
  // --------------------------------------------------
  constexpr float CUR_TARGET = 35.0f;   // กระแสเป้าหมาย (Amp)
  constexpr float KP = 0.8f;
  constexpr float KI = 0.05f;

  constexpr float INTEGRATOR_LIMIT = 200.0f;

  // --------------------------------------------------
  // RESET INTEGRATOR (เมื่อไม่มีคำสั่ง)
  // --------------------------------------------------
  if (abs(targetL) < 1 && abs(targetR) < 1) {
    iErrL = 0;
    iErrR = 0;
    return;
  }

  // --------------------------------------------------
  // READ CURRENT
  // --------------------------------------------------
  float curL_now = curLeft();
  float curR_now = curRight();

  // --------------------------------------------------
  // ERROR
  // --------------------------------------------------
  float errL = CUR_TARGET - curL_now;
  float errR = CUR_TARGET - curR_now;

  // --------------------------------------------------
  // INTEGRATOR
  // --------------------------------------------------
  iErrL += errL;
  iErrR += errR;

  // anti-windup
  iErrL = constrain(iErrL, -INTEGRATOR_LIMIT, INTEGRATOR_LIMIT);
  iErrR = constrain(iErrR, -INTEGRATOR_LIMIT, INTEGRATOR_LIMIT);

  // --------------------------------------------------
  // PI OUTPUT
  // --------------------------------------------------
  float outL = KP * errL + KI * iErrL;
  float outR = KP * errR + KI * iErrR;

  // --------------------------------------------------
  // CONVERT TO SCALE
  // --------------------------------------------------
  float scaleL = outL / CUR_TARGET;
  float scaleR = outR / CUR_TARGET;

  // clamp scale (สำคัญมาก)
  scaleL = constrain(scaleL, 0.3f, 1.0f);
  scaleR = constrain(scaleR, 0.3f, 1.0f);

  // --------------------------------------------------
  // APPLY SCALE
  // --------------------------------------------------
  targetL *= scaleL;
  targetR *= scaleR;
}