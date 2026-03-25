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
  // MEASURE CURRENT
  // ==================================================
  float measL = getMotorCurrentSafeL();
  float measR = getMotorCurrentSafeR();

  // ==================================================
  // ERROR
  // ==================================================
  float errL = targetCurrentL - measL;
  float errR = targetCurrentR - measR;

  // ==================================================
  // PID CONFIG (ปรับให้เสถียรขึ้น)
  // ==================================================
  constexpr float KP = 0.6f;
  constexpr float KI = 0.12f;
  constexpr float KD = 0.005f;

  // ==================================================
  // 🔴 DERIVATIVE FILTER (สำคัญมาก)
  // ==================================================
  static float dFiltL = 0;
  static float dFiltR = 0;

  float dRawL = (errL - prevErrL) / dt;
  float dRawR = (errR - prevErrR) / dt;

  dFiltL = dFiltL * 0.7f + dRawL * 0.3f;
  dFiltR = dFiltR * 0.7f + dRawR * 0.3f;

  prevErrL = errL;
  prevErrR = errR;

  // ==================================================
  // 🔴 INTEGRATOR (CONDITIONAL ANTI-WINDUP)
  // ==================================================
  constexpr float I_LIMIT = 30.0f;

  bool satL = (curL >= PWM_TOP || curL <= -PWM_TOP);
  bool satR = (curR >= PWM_TOP || curR <= -PWM_TOP);

  if (!satL)
    iErrL += errL * dt;

  if (!satR)
    iErrR += errR * dt;

  iErrL = constrain(iErrL, -I_LIMIT, I_LIMIT);
  iErrR = constrain(iErrR, -I_LIMIT, I_LIMIT);

  // ==================================================
  // PID OUTPUT
  // ==================================================
  float outL = KP * errL + KI * iErrL + KD * dFiltL;
  float outR = KP * errR + KI * iErrR + KD * dFiltR;

  // ==================================================
  // 🔴 FEEDFORWARD (ช่วย response)
  // ==================================================
  constexpr float FF_GAIN = 20.0f;

  float ffL = targetCurrentL * FF_GAIN;
  float ffR = targetCurrentR * FF_GAIN;

  // ==================================================
  // 🔴 FINAL OUTPUT (ไม่ใช้ += แล้ว)
  // ==================================================
  float pwmL = ffL + outL * 25.0f;
  float pwmR = ffR + outR * 25.0f;

  // ==================================================
  // LIMIT
  // ==================================================
  curL = constrain(pwmL, -PWM_TOP, PWM_TOP);
  curR = constrain(pwmR, -PWM_TOP, PWM_TOP);
}

void resetCurrentLoop()
{
  iErrL = 0.0f;
  iErrR = 0.0f;

  prevErrL = 0.0f;
  prevErrR = 0.0f;
}

