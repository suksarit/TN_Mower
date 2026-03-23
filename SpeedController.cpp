// SpeedController.cpp

#include <Arduino.h>
#include "SpeedController.h"
#include "GlobalState.h"
#include "HardwareConfig.h"

// ======================================================
// INTERNAL STATE
// ======================================================
static float iErrL = 0;
static float iErrR = 0;

// ======================================================
// MAIN SPEED CONTROL
// ======================================================
void applySpeedControl(float &targetL, float &targetR)
{
  // ==================================================
  // TIME
  // ==================================================
  float dt = controlDt_s;
  if (dt <= 0.0001f) return;

  // ==================================================
  // MEASURE SPEED (ต้องมี sensor จริง)
  // ==================================================
  float speedL = getWheelSpeedL();
  float speedR = getWheelSpeedR();

  // ==================================================
  // ERROR
  // ==================================================
  float errL = targetL - speedL;
  float errR = targetR - speedR;

  // ==================================================
  // PI CONTROL
  // ==================================================
  constexpr float KP = 0.8f;
  constexpr float KI = 1.5f;

  iErrL += errL * dt;
  iErrR += errR * dt;

  // anti-windup
  constexpr float I_LIMIT = PWM_TOP * 0.5f;
  iErrL = constrain(iErrL, -I_LIMIT, I_LIMIT);
  iErrR = constrain(iErrR, -I_LIMIT, I_LIMIT);

  float outL = KP * errL + KI * iErrL;
  float outR = KP * errR + KI * iErrR;

  // ==================================================
  // APPLY
  // ==================================================
  targetL += outL;
  targetR += outR;

  targetL = constrain(targetL, -PWM_TOP, PWM_TOP);
  targetR = constrain(targetR, -PWM_TOP, PWM_TOP);
}


