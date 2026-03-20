// ============================================================================
// CurrentController.cpp (TORQUE PID - CLEAN + SAFE + STABLE)
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "SensorManager.h"
#include "CurrentController.h"
#include "HardwareConfig.h"

// ==================================================
// CURRENT LOOP STATE
// ==================================================
static float iErrL = 0.0f;
static float iErrR = 0.0f;

static float prevErrL = 0.0f;
static float prevErrR = 0.0f;

// ============================================================================
// APPLY CURRENT PID (TORQUE CONTROL)
// INPUT: targetCurrent (Amp)
// OUTPUT: curL / curR (PWM)
// ============================================================================

void applyCurrentPID(float targetCurrentL, float targetCurrentR)
{
  // ==================================================
  // TIME
  // ==================================================
  float dt = controlDt_s;

  if (dt <= 0.0001f)
    return;

  // ==================================================
  // MEASURE CURRENT (SAFE)
  // ==================================================
  float measL = getMotorCurrentSafeL();
  float measR = getMotorCurrentSafeR();

  // ==================================================
  // ERROR (Amp)
  // ==================================================
  float errL = targetCurrentL - measL;
  float errR = targetCurrentR - measR;

  // ==================================================
  // PID CONFIG (ต้องจูน)
  // ==================================================
  constexpr float KP = 0.7f;
  constexpr float KI = 0.15f;
  constexpr float KD = 0.01f;

  // ==================================================
  // INTEGRATOR (ANTI-WINDUP)
  // ==================================================
  constexpr float I_LIMIT = 40.0f;   // Amp·s

  iErrL += errL * dt;
  iErrR += errR * dt;

  iErrL = constrain(iErrL, -I_LIMIT, I_LIMIT);
  iErrR = constrain(iErrR, -I_LIMIT, I_LIMIT);

  // ==================================================
  // DERIVATIVE
  // ==================================================
  float dErrL = (errL - prevErrL) / dt;
  float dErrR = (errR - prevErrR) / dt;

  prevErrL = errL;
  prevErrR = errR;

  // ==================================================
  // PID OUTPUT (Amp → PWM)
  // ==================================================
  float outL = KP * errL + KI * iErrL + KD * dErrL;
  float outR = KP * errR + KI * iErrR + KD * dErrR;

  // ==================================================
  // SCALE TO PWM DOMAIN
  // ==================================================
  constexpr float CURRENT_TO_PWM = 40.0f; // 🔴 ต้องจูน

  curL += outL * CURRENT_TO_PWM;
  curR += outR * CURRENT_TO_PWM;

  // ==================================================
  // LIMIT
  // ==================================================
  curL = constrain(curL, -PWM_TOP, PWM_TOP);
  curR = constrain(curR, -PWM_TOP, PWM_TOP);

  // ==================================================
  // HARD CLAMP ANTI-WINDUP (สำคัญ)
  // ==================================================
  if (curL == PWM_TOP || curL == -PWM_TOP)
    iErrL *= 0.9f;

  if (curR == PWM_TOP || curR == -PWM_TOP)
    iErrR *= 0.9f;
}

// ============================================================================
// RESET CURRENT LOOP (CRITICAL SAFETY)
// ============================================================================

void resetCurrentLoop()
{
  iErrL = 0.0f;
  iErrR = 0.0f;

  prevErrL = 0.0f;
  prevErrR = 0.0f;
}

