// ============================================================================
// CurrentController.cpp
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
  // TIME (FIXED DT)
  // ==================================================
  const float dt = controlDt_s;
  if (dt <= 0.0001f)
    return;

  // ==================================================
  // MEASURE CURRENT
  // ==================================================
  const float measL = getMotorCurrentSafeL();
  const float measR = getMotorCurrentSafeR();

  // ==================================================
  // ERROR
  // ==================================================
  const float errL = targetCurrentL - measL;
  const float errR = targetCurrentR - measR;

  // ==================================================
  // PID CONFIG (TUNED - STABLE FIELD USE)
  // ==================================================
  constexpr float KP = 0.6f;
  constexpr float KI = 0.10f;   // 🔴 ลดเล็กน้อย กันสะสมไวเกิน
  constexpr float KD = 0.003f;  // 🔴 ลด noise derivative

  // ==================================================
  // 🔴 DERIVATIVE (FILTERED - LOW NOISE)
  // ==================================================
  static float dFiltL = 0.0f;
  static float dFiltR = 0.0f;

  const float dRawL = (errL - prevErrL) / dt;
  const float dRawR = (errR - prevErrR) / dt;

  // low-pass filter (stronger)
  dFiltL = dFiltL * 0.8f + dRawL * 0.2f;
  dFiltR = dFiltR * 0.8f + dRawR * 0.2f;

  prevErrL = errL;
  prevErrR = errR;

  // ==================================================
  // 🔴 INTEGRATOR (ANTI-WINDUP IMPROVED)
  // ==================================================
  constexpr float I_LIMIT = 25.0f;

  const bool satL = (curL >= PWM_TOP || curL <= -PWM_TOP);
  const bool satR = (curR >= PWM_TOP || curR <= -PWM_TOP);

  // integrate only if not saturating OR helping recover
  if (!satL || (satL && ((curL > 0 && errL < 0) || (curL < 0 && errL > 0))))
    iErrL += errL * dt;

  if (!satR || (satR && ((curR > 0 && errR < 0) || (curR < 0 && errR > 0))))
    iErrR += errR * dt;

  iErrL = constrain(iErrL, -I_LIMIT, I_LIMIT);
  iErrR = constrain(iErrR, -I_LIMIT, I_LIMIT);

  // ==================================================
  // PID OUTPUT
  // ==================================================
  const float outL = KP * errL + KI * iErrL + KD * dFiltL;
  const float outR = KP * errR + KI * iErrR + KD * dFiltR;

  // ==================================================
  // 🔴 FEEDFORWARD (BALANCED)
  // ==================================================
  constexpr float FF_GAIN = 8.0f;

  const float ffL = targetCurrentL * FF_GAIN;
  const float ffR = targetCurrentR * FF_GAIN;

  // ==================================================
  // COMBINE
  // ==================================================
  float pwmL = ffL + outL * 15.0f;
  float pwmR = ffR + outR * 15.0f;

  // ==================================================
  // 🔴 OUTPUT SLEW LIMIT (CRITICAL - กัน jerk)
  // ==================================================
  static float lastPwmL = 0.0f;
  static float lastPwmR = 0.0f;

  const float maxStep = 80.0f;  // 🔴 ปรับได้ (ยิ่งน้อยยิ่งนุ่ม)

  float dL = pwmL - lastPwmL;
  float dR = pwmR - lastPwmR;

  if (dL > maxStep) dL = maxStep;
  if (dL < -maxStep) dL = -maxStep;

  if (dR > maxStep) dR = maxStep;
  if (dR < -maxStep) dR = -maxStep;

  pwmL = lastPwmL + dL;
  pwmR = lastPwmR + dR;

  lastPwmL = pwmL;
  lastPwmR = pwmR;

  // ==================================================
  // FINAL LIMIT
  // ==================================================
  curL = constrain(pwmL, -PWM_TOP, PWM_TOP);
  curR = constrain(pwmR, -PWM_TOP, PWM_TOP);
}

void resetCurrentLoop() {
  iErrL = 0.0f;
  iErrR = 0.0f;

  prevErrL = 0.0f;
  prevErrR = 0.0f;
}
