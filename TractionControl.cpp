// TractionControl.cpp

#include <Arduino.h>
#include "TractionControl.h"

// ======================================================
// LOCAL STATE
// ======================================================

static float tractionScaleL = 1.0f;
static float tractionScaleR = 1.0f;

// ======================================================
// MAIN
// ======================================================

void applyTractionControl(
  float &targetCurrentL,
  float &targetCurrentR,
  float curA_L,
  float curA_R)
{
  // ===============================
  // CONFIG (ต้องจูน)
  // ===============================
  const float SLIP_RATIO = 0.35f;     // ต่ำกว่านี้ = น่าสงสัยว่าฟรี
  const float RECOVER_RATE = 0.02f;
  const float REDUCE_RATE  = 0.08f;

  const float MIN_SCALE = 0.4f;

  // ==================================================
  // LEFT
  // ==================================================
  float expectL = abs(targetCurrentL);

  if (expectL > 5.0f)
  {
    float ratio = curA_L / expectL;

    if (ratio < SLIP_RATIO)
    {
      tractionScaleL -= REDUCE_RATE;
    }
    else
    {
      tractionScaleL += RECOVER_RATE;
    }
  }
  else
  {
    tractionScaleL = 1.0f;
  }

  tractionScaleL = constrain(tractionScaleL, MIN_SCALE, 1.0f);

  // ==================================================
  // RIGHT
  // ==================================================
  float expectR = abs(targetCurrentR);

  if (expectR > 5.0f)
  {
    float ratio = curA_R / expectR;

    if (ratio < SLIP_RATIO)
    {
      tractionScaleR -= REDUCE_RATE;
    }
    else
    {
      tractionScaleR += RECOVER_RATE;
    }
  }
  else
  {
    tractionScaleR = 1.0f;
  }

  tractionScaleR = constrain(tractionScaleR, MIN_SCALE, 1.0f);

  // ==================================================
  // APPLY
  // ==================================================
  targetCurrentL *= tractionScaleL;
  targetCurrentR *= tractionScaleR;
}

