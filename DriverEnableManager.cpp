//  DriverEnableManager.cpp

#include "DriverEnableManager.h"
#include "HardwareConfig.h"
#include "MotorDriver.h"
#include "GlobalState.h"

bool neutral(uint16_t v);
bool driveCommandZero();

void updateDriverEnable(DriverEnableContext &ctx)
{

  bool runAllowed =
    (ctx.driveState == DriveState::RUN ||
     ctx.driveState == DriveState::LIMP) &&
    !ctx.driverRearmRequired;

  bool pwmSafe =
    (abs(ctx.curL) < 20) &&
    (abs(ctx.curR) < 20) &&
    (abs(ctx.targetL) < 20) &&
    (abs(ctx.targetR) < 20);

  bool thrNeutral = neutral(ctx.rcThrottle);
  bool strNeutral = neutral(ctx.rcSteer);

  bool rcSafe = thrNeutral && strNeutral;

  static uint32_t targetStableStart_ms = 0;

  bool targetNowZero = driveCommandZero();

  if (targetNowZero)
  {
    if (targetStableStart_ms == 0)
      targetStableStart_ms = ctx.now;
  }
  else
  {
    targetStableStart_ms = 0;
  }

  bool targetSafe =
    targetNowZero &&
    (targetStableStart_ms != 0) &&
    (ctx.now - targetStableStart_ms >= 50);

  bool ibusConfirmed = !ctx.requireIbusConfirm;
  bool autoReverseInactive = !ctx.autoReverseActive;

  bool dirSafe = (PORTA & 0b00001111) == 0;

  bool driverEnableConditions =
    ctx.systemState == SystemState::ACTIVE &&
    !ctx.faultLatched &&
    runAllowed &&
    pwmSafe &&
    rcSafe &&
    targetSafe &&
    ibusConfirmed &&
    autoReverseInactive &&
    dirSafe;

  constexpr uint32_t DRIVER_ARM_MS = 80;
  constexpr uint32_t DRIVER_SETTLE_MS = 40;

  switch (ctx.driverState)
  {

    case DriverState::DISABLED:

      digitalWrite(PIN_DRV_ENABLE, LOW);
      HBRIDGE_ALL_OFF();

      if (driverEnableConditions)
      {
        ctx.driverState = DriverState::ARMING;
        ctx.driverStateStart_ms = ctx.now;
      }

      break;

    case DriverState::ARMING:

      digitalWrite(PIN_DRV_ENABLE, LOW);

      if (!driverEnableConditions)
      {
        ctx.driverState = DriverState::DISABLED;
        break;
      }

      if (ctx.now - ctx.driverStateStart_ms >= DRIVER_ARM_MS)
      {
        setPWM_L(0);
        setPWM_R(0);

        digitalWrite(DIR_L1, LOW);
        digitalWrite(DIR_L2, LOW);
        digitalWrite(DIR_R1, LOW);
        digitalWrite(DIR_R2, LOW);

        delayMicroseconds(5);

        digitalWrite(PIN_DRV_ENABLE, HIGH);

        ctx.driverEnabled_ms = ctx.now;
        ctx.driverState = DriverState::SETTLING;
      }

      break;

    case DriverState::SETTLING:

      setPWM_L(0);
      setPWM_R(0);

      if (ctx.now - ctx.driverEnabled_ms >= DRIVER_SETTLE_MS)
      {
        ctx.driverState = DriverState::ACTIVE;
        ctx.driverActiveStart_ms = ctx.now;
      }

      break;

    case DriverState::ACTIVE:

      constexpr uint32_t DRIVER_ACTIVE_GUARD_MS = 80;

      if (ctx.now - ctx.driverActiveStart_ms < DRIVER_ACTIVE_GUARD_MS)
      {
        ctx.targetL = 0;
        ctx.targetR = 0;
        ctx.curL = 0;
        ctx.curR = 0;
      }

      if (!driverEnableConditions)
      {
        ctx.driverState = DriverState::DISABLED;
        digitalWrite(PIN_DRV_ENABLE, LOW);
      }

      break;
  }
}

