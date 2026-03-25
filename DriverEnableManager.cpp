// ============================================================================
// DriverEnableManager.cpp (FIXED - STABLE ENABLE + NO GLITCH)
// ============================================================================

#include "DriverEnableManager.h"
#include "HardwareConfig.h"
#include "MotorDriver.h"
#include "GlobalState.h"

bool neutral(uint16_t v);
bool driveCommandZero();

// ============================================================================
// MAIN FUNCTION
// ============================================================================
void updateDriverEnable(DriverEnableContext &ctx)
{
  // ==================================================
  // 🔴 BASIC CONDITIONS
  // ==================================================
  bool runAllowed =
    (ctx.driveState == DriveState::RUN ||
     ctx.driveState == DriveState::LIMP) &&
    !ctx.driverRearmRequired;

  // 🔴 FIX: ใช้ scale ถูกต้อง (float -1.0 ถึง 1.0)
  bool pwmSafe =
    (fabs(ctx.curL) < 0.05f) &&
    (fabs(ctx.curR) < 0.05f) &&
    (fabs(ctx.targetL) < 0.05f) &&
    (fabs(ctx.targetR) < 0.05f);

  bool thrNeutral = neutral(ctx.rcThrottle);
  bool strNeutral = neutral(ctx.rcSteer);
  bool rcSafe = thrNeutral && strNeutral;

  // ==================================================
  // 🔴 TARGET ZERO STABLE (กัน jitter)
  // ==================================================
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
    (ctx.now - targetStableStart_ms >= 80);  // 🔴 เพิ่มเป็น 80ms

  // ==================================================
  // 🔴 OTHER CONDITIONS
  // ==================================================
  bool ibusConfirmed = !ctx.requireIbusConfirm;
  bool autoReverseInactive = !ctx.autoReverseActive;

  // 🔴 FIX: อ่านทิศทางให้ชัด
  uint8_t dirState = PORTA & 0b00001111;
  bool dirSafe = (dirState == 0);

  // ==================================================
  // 🔴 FINAL ENABLE CONDITIONS
  // ==================================================
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

  // ==================================================
  // 🔴 DEBOUNCE ENABLE CONDITIONS (สำคัญมาก)
  // ==================================================
  static uint32_t condStableStart_ms = 0;

  if (driverEnableConditions)
  {
    if (condStableStart_ms == 0)
      condStableStart_ms = ctx.now;
  }
  else
  {
    condStableStart_ms = 0;
  }

  bool condStable =
    (condStableStart_ms != 0) &&
    (ctx.now - condStableStart_ms >= 100);  // 🔴 ต้องนิ่ง 100ms

  // ==================================================
  // TIMING CONFIG
  // ==================================================
  constexpr uint32_t DRIVER_ARM_MS = 100;     // เดิม 80 → เพิ่ม
  constexpr uint32_t DRIVER_SETTLE_MS = 80;   // เดิม 40 → เพิ่ม
  constexpr uint32_t DRIVER_ACTIVE_GUARD_MS = 120;

  // ==================================================
  // STATE MACHINE
  // ==================================================
  switch (ctx.driverState)
  {
    // ==================================================
    case DriverState::DISABLED:
    {
      digitalWrite(PIN_DRV_ENABLE, LOW);
      HBRIDGE_ALL_OFF();

      ctx.curL = 0;
      ctx.curR = 0;
      ctx.targetL = 0;
      ctx.targetR = 0;

      if (condStable)
      {
        ctx.driverState = DriverState::ARMING;
        ctx.driverStateStart_ms = ctx.now;
      }

      break;
    }

    // ==================================================
    case DriverState::ARMING:
    {
      digitalWrite(PIN_DRV_ENABLE, LOW);

      if (!driverEnableConditions)
      {
        ctx.driverState = DriverState::DISABLED;
        break;
      }

      if (ctx.now - ctx.driverStateStart_ms >= DRIVER_ARM_MS)
      {
        // 🔴 reset ทุกอย่างก่อน enable
        ctx.curL = 0;
        ctx.curR = 0;
        ctx.targetL = 0;
        ctx.targetR = 0;

        // 🔴 kill direction
        HBRIDGE_ALL_OFF();

        delayMicroseconds(5);

        digitalWrite(PIN_DRV_ENABLE, HIGH);

        ctx.driverEnabled_ms = ctx.now;
        ctx.driverState = DriverState::SETTLING;
      }

      break;
    }

    // ==================================================
    case DriverState::SETTLING:
    {
      // 🔴 freeze output
      ctx.curL = 0;
      ctx.curR = 0;
      ctx.targetL = 0;
      ctx.targetR = 0;

      if (ctx.now - ctx.driverEnabled_ms >= DRIVER_SETTLE_MS)
      {
        ctx.driverState = DriverState::ACTIVE;
        ctx.driverActiveStart_ms = ctx.now;
      }

      break;
    }

    // ==================================================
    case DriverState::ACTIVE:
    {
      // 🔴 guard ช่วงแรกกันกระชาก
      if (ctx.now - ctx.driverActiveStart_ms < DRIVER_ACTIVE_GUARD_MS)
      {
        ctx.targetL = 0;
        ctx.targetR = 0;
        ctx.curL = 0;
        ctx.curR = 0;
      }

      // 🔴 fail-safe
      if (!driverEnableConditions)
      {
        digitalWrite(PIN_DRV_ENABLE, LOW);
        HBRIDGE_ALL_OFF();

        ctx.driverState = DriverState::DISABLED;
      }

      break;
    }
  }
}

