// ============================================================================
// CurrentController.cpp (FIXED - SINGLE INTEGRATOR + RESET SAFE)
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "SensorManager.h"
#include "CurrentController.h"
#include "HardwareConfig.h"

// ==================================================
// CURRENT LOOP STATE (GLOBAL STATIC - SINGLE SOURCE)
// ==================================================
static float iErrL = 0.0f;
static float iErrR = 0.0f;

// ============================================================================
// APPLY CURRENT LOOP (PI CONTROL)
// แก้ค่า targetL / targetR โดยตรง
// ============================================================================

void applyCurrentLoop(float &targetL, float &targetR)
{
  // ==================================================
  // TIME (dt จาก control loop)
  // ==================================================
  float dt = controlDt_s;

  if (dt <= 0.0001f)
    return;

  // ==================================================
  // MEASURE CURRENT (Amp)
  // ==================================================
  float measL = getMotorCurrentL();
  float measR = getMotorCurrentR();

  // ==================================================
  // ERROR
  // ==================================================
  float errL = targetL - measL;
  float errR = targetR - measR;

  // ==================================================
  // INTEGRATOR (ANTI-WINDUP)
  // ==================================================
  constexpr float I_LIMIT = PWM_TOP * 0.5f;

  iErrL += errL * dt;
  iErrR += errR * dt;

  iErrL = constrain(iErrL, -I_LIMIT, I_LIMIT);
  iErrR = constrain(iErrR, -I_LIMIT, I_LIMIT);

  // ==================================================
  // PI CONTROL
  // ==================================================
  constexpr float KP = 0.6f;
  constexpr float KI = 1.2f;

  float outL = KP * errL + KI * iErrL;
  float outR = KP * errR + KI * iErrR;

  // ==================================================
  // APPLY OUTPUT
  // ==================================================
  targetL += outL;
  targetR += outR;

  // ==================================================
  // FINAL CLAMP
  // ==================================================
  targetL = constrain(targetL, -PWM_TOP, PWM_TOP);
  targetR = constrain(targetR, -PWM_TOP, PWM_TOP);
}

// ============================================================================
// RESET CURRENT LOOP (CRITICAL FOR SAFETY)
// ต้องเรียกเมื่อ:
// - FAULT
// - DRIVER DISABLE
// - SOFT STOP
// ============================================================================

void resetCurrentLoop()
{
  iErrL = 0.0f;
  iErrR = 0.0f;
}