// ============================================================================
// MotorOutput.cpp (FIXED - DEADTIME SAFE + NO SPIKE)
// ============================================================================

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "HardwareConfig.h"
#include "MotorDriver.h"
#include "SystemDegradation.h"

// H-BRIDGE
#define HBRIDGE_L_OFF() (PORTA &= ~0b00000011)
#define HBRIDGE_L_FWD() (PORTA = (PORTA & ~0b00000011) | 0b00000001)
#define HBRIDGE_L_REV() (PORTA = (PORTA & ~0b00000011) | 0b00000010)

#define HBRIDGE_R_OFF() (PORTA &= ~0b00001100)
#define HBRIDGE_R_FWD() (PORTA = (PORTA & ~0b00001100) | 0b00000100)
#define HBRIDGE_R_REV() (PORTA = (PORTA & ~0b00001100) | 0b00001000)

// ============================================================================
// MAIN OUTPUT
// ============================================================================
void outputMotorPWM()
{
  enum State { RUN, DEADTIME, SET_DIR };

  static State stateL = RUN;
  static State stateR = RUN;

  static uint32_t timerL = 0;
  static uint32_t timerR = 0;

  constexpr uint16_t DEADTIME_US = 1200;

  uint32_t now = micros();

  curL = constrain(curL, -PWM_TOP, PWM_TOP);
  curR = constrain(curR, -PWM_TOP, PWM_TOP);

  int8_t dirL = (curL > 0) ? 1 : (curL < 0 ? -1 : 0);
  int8_t dirR = (curR > 0) ? 1 : (curR < 0 ? -1 : 0);

  float scale = getPowerScale();

uint16_t pwmL = abs(curL) * scale;
uint16_t pwmR = abs(curR) * scale;

  // ================= LEFT =================
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
      if (now - timerL >= DEADTIME_US)
        stateL = SET_DIR;
      break;

    case SET_DIR:
      if (dirL > 0) HBRIDGE_L_FWD();
      else if (dirL < 0) HBRIDGE_L_REV();
      else HBRIDGE_L_OFF();

      lastDirL = dirL;
      stateL = RUN;
      break;
  }

  // ================= RIGHT =================
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
      if (now - timerR >= DEADTIME_US)
        stateR = SET_DIR;
      break;

    case SET_DIR:
      if (dirR > 0) HBRIDGE_R_FWD();
      else if (dirR < 0) HBRIDGE_R_REV();
      else HBRIDGE_R_OFF();

      lastDirR = dirR;
      stateR = RUN;
      break;
  }
}

