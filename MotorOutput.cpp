// ============================================================================
// MotorOutput.cpp (FIXED - SAFE + NO BUG + PID COMPATIBLE)
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "HardwareConfig.h"
#include "MotorDriver.h"

// ======================================================
// H-BRIDGE MACRO
// ======================================================

#define HBRIDGE_L_OFF() (PORTA &= ~0b00000011)
#define HBRIDGE_L_FWD() (PORTA = (PORTA & ~0b00000011) | 0b00000001)
#define HBRIDGE_L_REV() (PORTA = (PORTA & ~0b00000011) | 0b00000010)

#define HBRIDGE_R_OFF() (PORTA &= ~0b00001100)
#define HBRIDGE_R_FWD() (PORTA = (PORTA & ~0b00001100) | 0b00000100)
#define HBRIDGE_R_REV() (PORTA = (PORTA & ~0b00001100) | 0b00001000)

// ============================================================================
// MAIN OUTPUT FUNCTION
// ============================================================================

void outputMotorPWM()
{
  enum MotorState
  {
    RUN,
    DEADTIME,
    SET_DIR,
    PWM_DELAY
  };

  static MotorState stateL = RUN;
  static MotorState stateR = RUN;

  static uint32_t timerL = 0;
  static uint32_t timerR = 0;

  constexpr uint16_t DIR_DEADTIME_US = 1200;
  constexpr uint16_t PWM_RESUME_DELAY_US = 800;

  uint32_t now = micros();

  // ==================================================
  // 🔴 CLAMP INPUT (จาก PID)
  // ==================================================
  curL = constrain(curL, -PWM_TOP, PWM_TOP);
  curR = constrain(curR, -PWM_TOP, PWM_TOP);

  int8_t dirL = (curL > 0) ? 1 : (curL < 0 ? -1 : 0);
  int8_t dirR = (curR > 0) ? 1 : (curR < 0 ? -1 : 0);

  int16_t pwmL = abs(curL);
  int16_t pwmR = abs(curR);

  // ==================================================
  // LEFT MOTOR
  // ==================================================

  switch (stateL)
  {
    case RUN:

      if (dirL != lastDirL)
      {
        setPWM_L(0);
        HBRIDGE_L_OFF();

        timerL = now;
        stateL = DEADTIME;
        break;
      }

      setPWM_L(pwmL);
      break;

    case DEADTIME:

      setPWM_L(0);

      if (now - timerL >= DIR_DEADTIME_US)
        stateL = SET_DIR;

      break;

    case SET_DIR:

      setPWM_L(0);

      if (dirL > 0) HBRIDGE_L_FWD();
      else if (dirL < 0) HBRIDGE_L_REV();
      else HBRIDGE_L_OFF();

      lastDirL = dirL;
      timerL = now;

      stateL = PWM_DELAY;
      break;

    case PWM_DELAY:

      setPWM_L(0);

      if (now - timerL >= PWM_RESUME_DELAY_US)
        stateL = RUN;

      break;
  }

  // ==================================================
  // RIGHT MOTOR
  // ==================================================

  switch (stateR)
  {
    case RUN:

      if (dirR != lastDirR)
      {
        setPWM_R(0);
        HBRIDGE_R_OFF();

        timerR = now;
        stateR = DEADTIME;
        break;
      }

      setPWM_R(pwmR);
      break;

    case DEADTIME:

      setPWM_R(0);

      if (now - timerR >= DIR_DEADTIME_US)
        stateR = SET_DIR;

      break;

    case SET_DIR:

      setPWM_R(0);

      if (dirR > 0) HBRIDGE_R_FWD();
      else if (dirR < 0) HBRIDGE_R_REV();
      else HBRIDGE_R_OFF();

      lastDirR = dirR;
      timerR = now;

      stateR = PWM_DELAY;
      break;

    case PWM_DELAY:

      setPWM_R(0);

      if (now - timerR >= PWM_RESUME_DELAY_US)
        stateR = RUN;

      break;
  }
}

