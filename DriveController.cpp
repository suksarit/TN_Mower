//DriveController.cpp

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include "GlobalState.h"
#include "DriveController.h"
#include "MotorDriver.h"
#include "SafetyManager.h"
#include "FaultManager.h"
#include "SensorManager.h"
#include "HardwareConfig.h"
#include "SystemTypes.h"
#include "CommsManager.h"
#include "Storm32Controller.h"
#include "ThermalManager.h"

// ==================================================
// CURRENT LOOP STATE
// ==================================================
static float iErrL = 0;
static float iErrR = 0;

// AUTO REVERSE STATE
uint32_t autoReverseStart_ms = 0;

// AUTO REVERSE RECOVERY WINDOW
bool reverseRecoveryActive = false;

// reverse oscillation lock
uint32_t reverseLockUntil_ms = 0;

// terrain drag estimator
float terrainDragAvg = 0;

// MOTOR STALL ENERGY PROTECTION
float stallEnergy = 0.0f;
uint32_t stallEnergyLast_ms = 0;

#define HBRIDGE_L_OFF() (PORTA &= ~0b00000011)
#define HBRIDGE_L_FWD() \
  { PORTA = (PORTA & ~0b00000011) | 0b00000001; }
#define HBRIDGE_L_REV() \
  { PORTA = (PORTA & ~0b00000011) | 0b00000010; }

#define HBRIDGE_R_OFF() (PORTA &= ~0b00001100)
#define HBRIDGE_R_FWD() \
  { PORTA = (PORTA & ~0b00001100) | 0b00000100; }
#define HBRIDGE_R_REV() \
  { PORTA = (PORTA & ~0b00001100) | 0b00001000; }


void runDrive(uint32_t now) {
  static DriveState lastDriveState = DriveState::IDLE;
  static uint32_t limpSafeStart_ms = 0;

  // ==================================================
  // MOTOR RUNAWAY DETECTION
  // ==================================================

  constexpr int16_t RUNAWAY_PWM_THRESHOLD = 120;
  constexpr uint32_t RUNAWAY_CONFIRM_MS = 200;

  static uint32_t runawayStart_ms = 0;

  bool commandZero = (targetL == 0 && targetR == 0);

  bool motorMoving =
    (abs(curL) > RUNAWAY_PWM_THRESHOLD || abs(curR) > RUNAWAY_PWM_THRESHOLD);

  if (commandZero && motorMoving) {
    if (runawayStart_ms == 0)
      runawayStart_ms = now;

    if (now - runawayStart_ms > RUNAWAY_CONFIRM_MS) {
#if DEBUG_SERIAL
      Serial.println(F("[DRIVE] RUNAWAY DETECTED"));
#endif
      latchFault(FaultCode::DRIVE_TIMEOUT);
    }
  } else {
    runawayStart_ms = 0;
  }

  // ==================================================
  // CACHE SAFETY STATE
  // ==================================================

  SafetyState safety = getDriveSafety();

  // ==================================================
  // DRIVE STATE MACHINE
  // ==================================================

  switch (driveState) {

    case DriveState::IDLE:

      targetL = 0;
      targetR = 0;
      curL = 0;
      curR = 0;

      limpSafeStart_ms = 0;
      driveSoftStopStart_ms = 0;

      if (systemState == SystemState::ACTIVE) {
        driveState = DriveState::RUN;
      }

      break;

      // ==================================================

    case DriveState::RUN:

      if (safety == SafetyState::EMERGENCY) {
        driveState = DriveState::SOFT_STOP;
      } else if (safety == SafetyState::LIMP) {
        driveState = DriveState::LIMP;
        limpSafeStart_ms = 0;
      }

      break;

      // ==================================================

    case DriveState::LIMP:

      updateDriveRamp(targetL, targetR);

      targetL /= 2;
      targetR /= 2;

      if (safety == SafetyState::EMERGENCY) {
        driveState = DriveState::SOFT_STOP;
        break;
      }

      if (safety == SafetyState::SAFE) {
        if (limpSafeStart_ms == 0)
          limpSafeStart_ms = now;

        else if (now - limpSafeStart_ms >= LIMP_RECOVER_MS) {

#if DEBUG_SERIAL
          Serial.println(F("[DRIVE] LIMP RECOVER -> RUN"));
#endif

          driveState = DriveState::RUN;
          limpSafeStart_ms = 0;
        }
      } else {
        limpSafeStart_ms = 0;
      }

      break;

      // ==================================================

    case DriveState::SOFT_STOP:

      targetL = 0;
      targetR = 0;

      if (driveSoftStopStart_ms == 0)
        driveSoftStopStart_ms = now;

      if ((curL == 0 && curR == 0) || (now - driveSoftStopStart_ms >= DRIVE_SOFT_STOP_TIMEOUT_MS)) {
        driveSafe();
        driveState = DriveState::LOCKED;
      }

      break;

      // ==================================================

    case DriveState::LOCKED:

      driveSafe();
      break;

      // ==================================================

    default:

      latchFault(FaultCode::LOGIC_WATCHDOG);

      driveSafe();

      targetL = 0;
      targetR = 0;
      curL = 0;
      curR = 0;

      driveState = DriveState::LOCKED;

      limpSafeStart_ms = 0;
      driveSoftStopStart_ms = 0;

      break;
  }

  // ==================================================
  // STATE CHANGE LOG
  // ==================================================

  if (driveState != lastDriveState) {

#if DEBUG_SERIAL
    Serial.print(F("[DRIVE STATE] "));
    Serial.print((uint8_t)lastDriveState);
    Serial.print(F(" -> "));
    Serial.println((uint8_t)driveState);
#endif

    if (driveState == DriveState::SOFT_STOP)
      driveSoftStopStart_ms = now;

    lastDriveState = driveState;
  }
}

void applyCurrentLoop(float &targetL, float &targetR) {
  // --------------------------------------------------
  // CONFIG
  // --------------------------------------------------
  constexpr float CUR_TARGET = 35.0f;  // กระแสเป้าหมาย
  constexpr float KP = 0.8f;
  constexpr float KI = 0.05f;

  // --------------------------------------------------
  // READ CURRENT
  // --------------------------------------------------
  float curL_now = curLeft();
  float curR_now = curRight();

  // --------------------------------------------------
  // ERROR
  // --------------------------------------------------
  float errL = CUR_TARGET - curL_now;
  float errR = CUR_TARGET - curR_now;

  // --------------------------------------------------
  // INTEGRATOR
  // --------------------------------------------------
  iErrL += errL;
  iErrR += errR;

  // anti windup
  iErrL = constrain(iErrL, -200, 200);
  iErrR = constrain(iErrR, -200, 200);

  // --------------------------------------------------
  // PI OUTPUT
  // --------------------------------------------------
  float outL = KP * errL + KI * iErrL;
  float outR = KP * errR + KI * iErrR;

  // --------------------------------------------------
  // APPLY SCALE (ลด PWM เมื่อ current สูง)
  // --------------------------------------------------
  float scaleL = constrain(outL / CUR_TARGET, 0.3f, 1.0f);
  float scaleR = constrain(outR / CUR_TARGET, 0.3f, 1.0f);

  targetL *= scaleL;
  targetR *= scaleR;
}

void applyDrive(uint32_t now) {

  // --------------------------------------------------
  // DRIVER NOT ENABLED → BLOCK PWM OUTPUT
  // --------------------------------------------------
  if (driverState != DriverState::ACTIVE) {

    setPWM_L(0);
    setPWM_R(0);

    curL = 0;
    curR = 0;

    targetL = 0;
    targetR = 0;

    return;
  }

  // ==================================================
  // HARD STOP
  // ==================================================
  if (systemState == SystemState::FAULT || driveState == DriveState::LOCKED) {

    driveSafe();

    curL = 0;
    curR = 0;

    targetL = 0;
    targetR = 0;

    return;
  }

  // ==================================================
  // DRIVER SETTLING WINDOW
  // ==================================================
  if (driverState == DriverState::SETTLING) {

    setPWM_L(0);
    setPWM_R(0);

    curL = 0;
    curR = 0;

    return;
  }

  // ==================================================
  // SYSTEM SAFETY GUARD
  // ==================================================
  if (systemState != SystemState::ACTIVE) {
    driveSafe();

    curL = 0;
    curR = 0;

    targetL = 0;
    targetR = 0;

    return;
  }

  // ==================================================
  // TARGET COMPUTE
  // ==================================================
  float finalTargetL = targetL;
  float finalTargetR = targetR;

  computeDriveTarget(finalTargetL, finalTargetR, now);

  // ==================================================
  // STALL ENERGY PROTECTION
  // ==================================================
  float stallScale = 1.0f;

  if (!autoReverseActive) {

    stallScale =
      computeStallScale(now, curLeft(), curRight());

    finalTargetL *= stallScale;
    finalTargetR *= stallScale;
  }

  // ==================================================
  // 🔴 CURRENT LOOP  
  // ==================================================
  applyCurrentLoop(finalTargetL, finalTargetR);

  // ==================================================
  // POWER MANAGEMENT (FINAL SCALE)
  // ==================================================
  float scale = getPowerScale();

  if (isThermalEmergency()) {
    finalTargetL = 0;
    finalTargetR = 0;
  } else {
    finalTargetL *= scale;
    finalTargetR *= scale;
  }

  // ==================================================
  // 🔴 FEEDFORWARD LOAD ESTIMATE (ใส่ตรงนี้)
  // ==================================================
  float pwmLoad = (abs(finalTargetL) + abs(finalTargetR)) * 0.5f;

  float ffScale = 1.0f;

  if (pwmLoad > 600)
    ffScale = 0.8f;

  finalTargetL *= ffScale;
  finalTargetR *= ffScale;

  // ==================================================
  // DRIVE LIMITS
  // ==================================================
  applyDriveLimits(
    finalTargetL,
    finalTargetR,
    curLeft(),
    curRight());

  // ==================================================
  // LIMIT TARGET
  // ==================================================
  finalTargetL =
    constrain(finalTargetL, -PWM_TOP, PWM_TOP);

  finalTargetR =
    constrain(finalTargetR, -PWM_TOP, PWM_TOP);

  // ==================================================
  // RAMP CONTROL
  // ==================================================
  updateDriveRamp(finalTargetL, finalTargetR);

  // ==================================================
  // DEADZONE CLEANUP
  // ==================================================
  if (abs(curL) < 6)
    curL = 0;

  if (abs(curR) < 6)
    curR = 0;

  // ==================================================
  // OUTPUT PWM
  // ==================================================
  outputMotorPWM();
}

void computeDriveTarget(float &finalTargetL,
                        float &finalTargetR,
                        uint32_t now) {

  // ==================================================
  // AUTO REVERSE ACTIVE
  // ==================================================

  if (autoRev.active && !ibusCommLost && !requireIbusConfirm && systemState == SystemState::ACTIVE && (abs(curL) > 100 || abs(curR) > 100)) {

    // --------------------------------------------------
    // AUTO REVERSE RUNNING
    // --------------------------------------------------

    if (now - autoRev.start_ms < autoRev.duration_ms) {

      int8_t dirL = 0;
      int8_t dirR = 0;

      // ==================================================
      // DETECT SAFE REVERSE DIRECTION
      // ==================================================

      // --------------------------------------------------
      // LEFT MOTOR
      // --------------------------------------------------

      if (lastDirL > 0) {
        dirL = -1;
      } else if (lastDirL < 0) {
        dirL = 1;
      } else {
        // fallback ใช้ทิศจริงของ PWM
        if (curL > 0)
          dirL = -1;
        else if (curL < 0)
          dirL = 1;
        else {
          // ไม่รู้ทิศจริง → ยกเลิก auto reverse
          autoRev.active = false;
          autoReverseActive = false;
          return;
        }
      }

      // --------------------------------------------------
      // RIGHT MOTOR
      // --------------------------------------------------

      if (lastDirR > 0) {
        dirR = -1;
      } else if (lastDirR < 0) {
        dirR = 1;
      } else {
        if (curR > 0)
          dirR = -1;
        else if (curR < 0)
          dirR = 1;
        else {
          autoRev.active = false;
          autoReverseActive = false;
          return;
        }
      }

      // ==================================================
      // APPLY REVERSE POWER
      // ==================================================

      finalTargetL =
        constrain(dirL * autoRev.pwm, -PWM_TOP, PWM_TOP);

      finalTargetR =
        constrain(dirR * autoRev.pwm, -PWM_TOP, PWM_TOP);

      return;
    }

    // --------------------------------------------------
    // AUTO REVERSE FINISHED
    // --------------------------------------------------

    autoRev.active = false;
    autoReverseActive = false;

    reverseRecoveryActive = true;
    reverseRecoveryStart_ms = now;
  }

  // ==================================================
  // RECOVERY WINDOW
  // ==================================================

  if (reverseRecoveryActive) {
    if (now - reverseRecoveryStart_ms < REVERSE_RECOVERY_MS) {
      finalTargetL *= REVERSE_RECOVERY_LIMIT;
      finalTargetR *= REVERSE_RECOVERY_LIMIT;
      return;
    }

    reverseRecoveryActive = false;
  }
}


void applyDriveLimits(float &finalTargetL,
                      float &finalTargetR,
                      float curA_L,
                      float curA_R) {

  // ==================================================
  // 1️⃣ TRACTION CONTROL
  // ==================================================

  constexpr float TRACTION_SLIP_DIFF = 18.0f;

  if (fabs(curA_L - curA_R) > TRACTION_SLIP_DIFF) {

    if (curA_L < curA_R)
      finalTargetL *= 0.75f;

    else
      finalTargetR *= 0.75f;
  }

  // ==================================================
  // 2️⃣ CURRENT LIMIT
  // ==================================================

  float curUseL = curA_L;
  float curUseR = curA_R;

  if (curUseL > CUR_LIMP_A)
    finalTargetL *= 0.5f;

  if (curUseR > CUR_LIMP_A)
    finalTargetR *= 0.5f;

  if (curA_R > CUR_LIMP_A)
    finalTargetR *= 0.5f;

  // ==================================================
  // 5️⃣ IMBALANCE CORRECTION
  // ==================================================

  finalTargetL += imbalanceCorrL;
  finalTargetR += imbalanceCorrR;

  // ==================================================
  // CLAMP
  // ==================================================

  finalTargetL =
    constrain(finalTargetL, -PWM_TOP, PWM_TOP);

  finalTargetR =
    constrain(finalTargetR, -PWM_TOP, PWM_TOP);
}


void updateDriveRamp(float finalTargetL,
                     float finalTargetR) {
  finalTargetL = constrain(finalTargetL, -PWM_TOP, PWM_TOP);
  finalTargetR = constrain(finalTargetR, -PWM_TOP, PWM_TOP);

  uint32_t now = millis();

  // ==================================================
  // ERROR SIZE
  // ==================================================

  int16_t errMax =
    max(abs((int16_t)(finalTargetL - curL)),
        abs((int16_t)(finalTargetR - curR)));

  float step;

  if (driveState == DriveState::LIMP)
    step = 2.0f;
  else if (errMax > 700)
    step = 3.0f;
  else if (errMax > 350)
    step = 4.0f;
  else
    step = 6.0f;

  // ==================================================
  // SAFE REVERSE CONTROL
  // ==================================================

  constexpr int16_t REVERSE_SAFE_PWM = 120;

  bool reverseL =
    (curL > 0 && finalTargetL < 0) || (curL < 0 && finalTargetL > 0);

  bool reverseR =
    (curR > 0 && finalTargetR < 0) || (curR < 0 && finalTargetR > 0);

  if (reverseL) {

    if (abs(curL) > REVERSE_SAFE_PWM || now < revBlockUntilL)
      finalTargetL = 0;
    else {
      revBlockUntilL = now + REVERSE_DEADTIME_MS;
      finalTargetL = 0;
    }
  }

  if (reverseR) {

    if (abs(curR) > REVERSE_SAFE_PWM || now < revBlockUntilR)
      finalTargetR = 0;
    else {
      revBlockUntilR = now + REVERSE_DEADTIME_MS;
      finalTargetR = 0;
    }
  }

  // ==================================================
  // CURRENT BASED LOAD ADAPTIVE RAMP
  // ==================================================

  constexpr float LOAD_CURRENT_HIGH = 40.0f;
  constexpr float LOAD_CURRENT_MAX = 65.0f;

  float curLoad = max(curLeft(), curRight());

  float loadScale = 1.0f;

  if (curLoad > LOAD_CURRENT_HIGH) {

    loadScale =
      1.0f - ((curLoad - LOAD_CURRENT_HIGH) / (LOAD_CURRENT_MAX - LOAD_CURRENT_HIGH));

    loadScale = constrain(loadScale, 0.35f, 1.0f);
  }

  // ==================================================
  // TERRAIN DRAG ADAPTIVE CONTROL
  // ==================================================

  constexpr float TERRAIN_DRAG_HIGH = 50.0f;

  float terrainScale = 1.0f;

  if (terrainDragAvg > TERRAIN_DRAG_HIGH) {

    terrainScale =
      1.0f - (terrainDragAvg - TERRAIN_DRAG_HIGH) * 0.02f;

    terrainScale = constrain(terrainScale, 0.45f, 1.0f);
  }

  step *= loadScale;
  step *= terrainScale;

  if (step < 1.0f)
    step = 1.0f;

  int16_t stepInt = (int16_t)step;

  // ==================================================
  // APPLY RAMP
  // ==================================================

  curL = ramp(curL, finalTargetL, stepInt);
  curR = ramp(curR, finalTargetR, stepInt);

  // ==================================================
  // LOW SPEED SMOOTHING (ANTI JITTER)
  // ==================================================

  constexpr int16_t LOW_PWM = 120;

  if (abs(curL) < LOW_PWM)
    curL = (curL * 9) / 10;

  if (abs(curR) < LOW_PWM)
    curR = (curR * 9) / 10;

  // ==================================================
  // TRACTION BALANCE
  // ==================================================

  constexpr int16_t MAX_WHEEL_DIFF = 300;

  int16_t diff = curL - curR;

  if (diff > MAX_WHEEL_DIFF)
    curL = curR + MAX_WHEEL_DIFF;

  if (diff < -MAX_WHEEL_DIFF)
    curR = curL + MAX_WHEEL_DIFF;

  // ==================================================
  // FINAL CLAMP
  // ==================================================

  curL = constrain(curL, -PWM_TOP, PWM_TOP);
  curR = constrain(curR, -PWM_TOP, PWM_TOP);
}


void outputMotorPWM() {

  enum MotorState {
    RUN,
    DEADTIME,
    SET_DIR,
    PWM_DELAY
  };

  static MotorState stateL = RUN;
  static MotorState stateR = RUN;

  static uint32_t timerL = 0;
  static uint32_t timerR = 0;

  static int16_t pwmOutL = 0;
  static int16_t pwmOutR = 0;

  constexpr uint16_t DIR_DEADTIME_US = 1200;
  constexpr uint16_t PWM_RESUME_DELAY_US = 800;

  constexpr int16_t PWM_RAMP_STEP = 8;
  constexpr int16_t PWM_SNAP_WINDOW = 10;

  // torque precharge
  constexpr int16_t PWM_PRECHARGE = 40;

  uint32_t now = micros();

  int8_t dirL = (curL > 0) ? 1 : (curL < 0 ? -1 : 0);
  int8_t dirR = (curR > 0) ? 1 : (curR < 0 ? -1 : 0);

  int16_t targetL = abs(curL);
  int16_t targetR = abs(curR);

  // ================= LEFT =================

  // ================= LEFT =================

  switch (stateL) {

    case RUN:

      // --------------------------------------------------
      // Direction change request
      // --------------------------------------------------
      if (dirL != lastDirL) {

        // kill PWM ก่อนเสมอ
        setPWM_L(0);

        // disable H-bridge instantly
        HBRIDGE_L_OFF();

        timerL = now;
        pwmOutL = 0;

        stateL = DEADTIME;

        break;
      }

      // --------------------------------------------------
      // Normal ramp control
      // --------------------------------------------------
      {

        int16_t diff = targetL - pwmOutL;

        if (pwmOutL == 0 && targetL > PWM_PRECHARGE) {

          pwmOutL = PWM_PRECHARGE;

        } else if (abs(diff) <= PWM_SNAP_WINDOW) {

          pwmOutL = targetL;

        } else if (diff > 0) {

          pwmOutL += PWM_RAMP_STEP;

        } else {

          pwmOutL -= PWM_RAMP_STEP;
        }

        pwmOutL = constrain(pwmOutL, 0, PWM_TOP);

        setPWM_L(pwmOutL);
      }

      break;

      // --------------------------------------------------

    case DEADTIME:

      setPWM_L(0);

      if (now - timerL >= DIR_DEADTIME_US) {

        stateL = SET_DIR;
      }

      break;

      // --------------------------------------------------

    case SET_DIR:

      setPWM_L(0);

      // atomic direction update
      if (dirL > 0) {

        HBRIDGE_L_FWD();

      } else if (dirL < 0) {

        HBRIDGE_L_REV();

      } else {

        HBRIDGE_L_OFF();
      }

      timerL = now;

      lastDirL = dirL;

      stateL = PWM_DELAY;

      break;

      // --------------------------------------------------

    case PWM_DELAY:

      setPWM_L(0);

      if (now - timerL >= PWM_RESUME_DELAY_US) {

        stateL = RUN;
      }

      break;
  }

  // ================= RIGHT =================

  switch (stateR) {

    case RUN:

      if (dirR != 0 && dirR != lastDirR) {

        setPWM_R(0);

        PORTA &= ~0b00001100;

        timerR = now;
        pwmOutR = 0;

        stateR = DEADTIME;

      } else {

        int16_t diff = targetR - pwmOutR;

        if (pwmOutR == 0 && targetR > PWM_PRECHARGE) {

          pwmOutR = PWM_PRECHARGE;

        } else if (abs(diff) <= PWM_SNAP_WINDOW) {

          pwmOutR = targetR;

        } else if (diff > 0) {

          pwmOutR += PWM_RAMP_STEP;

        } else {

          pwmOutR -= PWM_RAMP_STEP;
        }

        pwmOutR = constrain(pwmOutR, 0, PWM_TOP);

        setPWM_R(pwmOutR);
      }

      break;

    case DEADTIME:

      setPWM_R(0);

      if (now - timerR >= DIR_DEADTIME_US) {
        stateR = SET_DIR;
      }

      break;

    case SET_DIR:

      setPWM_R(0);

      if (dirR > 0) {

        HBRIDGE_R_FWD();

      } else if (dirR < 0) {

        HBRIDGE_R_REV();

      } else {

        HBRIDGE_R_OFF();
      }

      timerR = now;
      stateR = PWM_DELAY;

      lastDirR = dirR;

      break;

    case PWM_DELAY:

      setPWM_R(0);

      if (now - timerR >= PWM_RESUME_DELAY_US) {
        stateR = RUN;
      }

      break;
  }
}


void detectWheelStuck(uint32_t now) {

  // --------------------------------------------------
  // AUTO REVERSE GUARD
  // --------------------------------------------------
  if (autoReverseActive)
    return;

  // --------------------------------------------------
  // GLOBAL COOLDOWN
  // --------------------------------------------------
  static uint32_t lastStuckTrigger_ms = 0;

  constexpr uint32_t STUCK_COOLDOWN_MS = 1500;

  if (now - lastStuckTrigger_ms < STUCK_COOLDOWN_MS)
    return;

  // --------------------------------------------------
  // TIMER สำหรับตรวจ stuck
  // --------------------------------------------------
  static uint32_t stuckStart_ms = 0;

  // --------------------------------------------------
  // CONFIG
  // --------------------------------------------------
  constexpr int16_t MIN_PWM_FOR_STUCK = 300;
  constexpr int16_t TURNING_DIFF_MAX = 260;

  int16_t pwmMag = max(abs(curL), abs(curR));

  // --------------------------------------------------
  // LOW PWM GUARD
  // --------------------------------------------------
  if (pwmMag < MIN_PWM_FOR_STUCK) {

    // decay terrain drag เมื่อโหลดต่ำ
    terrainDragAvg *= 0.90f;

    stuckStart_ms = 0;
    return;
  }

  // --------------------------------------------------
  // TURNING GUARD
  // --------------------------------------------------
  if (abs(curL - curR) > TURNING_DIFF_MAX) {

    stuckStart_ms = 0;
    return;
  }

  // --------------------------------------------------
  // CURRENT READ
  // --------------------------------------------------
  float cL = curLeft();
  float cR = curRight();

  if (!isfinite(cL) || !isfinite(cR)) {
    stuckStart_ms = 0;
    return;
  }

  float maxCur = max(cL, cR);

  // --------------------------------------------------
  // LOAD LOW → RESET DRAG MODEL
  // --------------------------------------------------
  if (maxCur < CUR_WARN_A) {

    terrainDragAvg *= 0.85f;

    stuckStart_ms = 0;
    return;
  }

  // --------------------------------------------------
  // TERRAIN DRAG ESTIMATOR
  // --------------------------------------------------
  float curAvg = (cL + cR) * 0.5f;

  constexpr float DRAG_ALPHA = 0.06f;

  terrainDragAvg =
    terrainDragAvg * (1.0f - DRAG_ALPHA) + curAvg * DRAG_ALPHA;

  // --------------------------------------------------
  // CLAMP TERRAIN DRAG
  // --------------------------------------------------
  terrainDragAvg =
    constrain(terrainDragAvg, 0.0f, CUR_LIMP_A * 1.6f);

  bool heavyGrass =
    terrainDragAvg > CUR_WARN_A * 1.4f;

  // --------------------------------------------------
  // TORQUE SATURATION
  // --------------------------------------------------
  bool torqueHigh;

  if (heavyGrass)
    torqueHigh = (maxCur > CUR_LIMP_A * 1.2f);
  else
    torqueHigh = (maxCur > CUR_LIMP_A);

  if (!torqueHigh) {

    stuckStart_ms = 0;
    return;
  }

  // --------------------------------------------------
  // CURRENT IMBALANCE
  // --------------------------------------------------
  float diff = fabs(cL - cR);

  float percentDiff =
    diff / (maxCur + 0.001f);

  float imbalanceThreshold;

  if (pwmMag < 400)
    imbalanceThreshold = 0.45f;
  else if (pwmMag < 700)
    imbalanceThreshold = 0.55f;
  else
    imbalanceThreshold = 0.65f;

  if (heavyGrass)
    imbalanceThreshold *= 1.20f;

  if (percentDiff <= imbalanceThreshold) {

    stuckStart_ms = 0;
    return;
  }

  // --------------------------------------------------
  // MOTOR ACCELERATION FAILURE
  // --------------------------------------------------
  static int16_t prevL = 0;
  static int16_t prevR = 0;

  int16_t dL = abs(curL - prevL);
  int16_t dR = abs(curR - prevR);

  prevL = curL;
  prevR = curR;

  constexpr int16_t MIN_ACCEL = 60;

  if (!(dL < MIN_ACCEL && dR < MIN_ACCEL)) {

    stuckStart_ms = 0;
    return;
  }

  // --------------------------------------------------
  // DYNAMIC STUCK TIME
  // --------------------------------------------------
  uint32_t stuckTime;

  if (pwmMag < 400)
    stuckTime = 800;
  else if (pwmMag < 700)
    stuckTime = 600;
  else
    stuckTime = 420;

  if (stuckStart_ms == 0)
    stuckStart_ms = now;

  if (now - stuckStart_ms >= stuckTime) {

    stuckStart_ms = 0;

    lastStuckTrigger_ms = now;

    if (cL > cR)
      lastDriveEvent = DriveEvent::STUCK_LEFT;
    else
      lastDriveEvent = DriveEvent::STUCK_RIGHT;

    startAutoReverse(now);
  }
}


void detectWheelLock() {

  static uint8_t lockCnt = 0;
  static uint32_t lockStart_ms = 0;

  // ==================================================
  // CONFIG
  // ==================================================
  constexpr int16_t MIN_PWM_FOR_LOCK = 350;
  constexpr float CURRENT_BALANCE_RATIO = 0.25f;
  constexpr uint8_t LOCK_CONFIRM_CNT = 4;

  // เพิ่ม time guard
  constexpr uint16_t LOCK_CONFIRM_MS = 400;

  uint32_t now = millis();

  // ==================================================
  // ต้องมีแรงขับจริง
  // ==================================================
  int16_t pwmMag = max(abs(curL), abs(curR));

  if (pwmMag < MIN_PWM_FOR_LOCK) {
    lockCnt = 0;
    lockStart_ms = 0;
    return;
  }

  // ==================================================
  // CURRENT
  // ==================================================
  float cL = curLeft();
  float cR = curRight();

  float maxCur = max(cL, cR);
  float diffCur = fabs(cL - cR);

  // ==================================================
  // CURRENT BALANCE
  // ถ้าต่างกันมาก = หญ้าหนา ไม่ใช่ lock
  // ==================================================
  if (maxCur <= 0.1f) {
    lockCnt = 0;
    lockStart_ms = 0;
    return;
  }

  float balanceRatio = diffCur / maxCur;

  if (balanceRatio > CURRENT_BALANCE_RATIO) {
    lockCnt = 0;
    lockStart_ms = 0;
    return;
  }

  // ==================================================
  // ADAPTIVE CURRENT THRESHOLD
  // ==================================================
  float lockCurrentThreshold;

  if (pwmMag < 500)
    lockCurrentThreshold = CUR_LIMP_A * 0.85f;
  else if (pwmMag < 800)
    lockCurrentThreshold = CUR_LIMP_A;
  else
    lockCurrentThreshold = CUR_LIMP_A * 1.15f;

  // ==================================================
  // BOTH SIDES HIGH CURRENT
  // ==================================================
  if (cL > lockCurrentThreshold && cR > lockCurrentThreshold) {

    // ==================================================
    // RAMP SATURATION CHECK
    // motor output ใกล้ target แล้ว
    // ==================================================
    int16_t errL = abs(targetL - curL);
    int16_t errR = abs(targetR - curR);

    if (errL < 50 && errR < 50) {

      if (lockCnt == 0) {
        lockStart_ms = now;
      }

      lockCnt++;

      // ต้องผ่านทั้ง counter และ time guard
      if (lockCnt >= LOCK_CONFIRM_CNT && (now - lockStart_ms) >= LOCK_CONFIRM_MS) {

        lastDriveEvent = DriveEvent::WHEEL_LOCK;
        latchFault(FaultCode::OVER_CURRENT);
      }

    } else {

      lockCnt = 0;
      lockStart_ms = 0;
    }

  } else {

    lockCnt = 0;
    lockStart_ms = 0;
  }
}


bool detectMotorStall() {

  static float prevCurL = 0;
  static float prevCurR = 0;

  static uint8_t stallCnt = 0;

  float cL = curLeft();
  float cR = curRight();

  if (abs(targetL) < 200 && abs(targetR) < 200) {

    prevCurL = cL;
    prevCurR = cR;

    return false;
  }

  if (cL < 5 && cR < 5)
    return false;

  float dCurL = cL - prevCurL;
  float dCurR = cR - prevCurR;

  constexpr float STALL_CURRENT_STEP = 18.0f;
  constexpr float STALL_CURRENT_MIN = 40.0f;
  constexpr uint8_t STALL_CONFIRM_CNT = 2;

  if (cL > STALL_CURRENT_MIN || cR > STALL_CURRENT_MIN) {

    bool stallDetected = false;

    if (cL > STALL_CURRENT_MIN && dCurL > STALL_CURRENT_STEP)
      stallDetected = true;

    if (cR > STALL_CURRENT_MIN && dCurR > STALL_CURRENT_STEP)
      stallDetected = true;

    if (stallDetected) {

      if (++stallCnt >= STALL_CONFIRM_CNT) {

        stallCnt = 0;

        prevCurL = cL;
        prevCurR = cR;

        return true;
      }

    } else {

      stallCnt = 0;
    }

  } else {

    stallCnt = 0;
  }

  prevCurL = cL;
  prevCurR = cR;

  return false;
}


void detectSideImbalanceAndSteer() {

  // ==================================================
  // RESET CORRECTION EVERY LOOP (TRANSIENT LAYER)
  // ==================================================
  imbalanceCorrL = 0;
  imbalanceCorrR = 0;

  // ==================================================
  // STOP COMMAND GUARD
  // ==================================================
  if (abs(targetL) < 50 && abs(targetR) < 50) {
    return;
  }

  // ==================================================
  // VEHICLE MUST ACTUALLY MOVE
  // ==================================================
  if (abs(curL) < 200 && abs(curR) < 200) {
    return;
  }

  // ==================================================
  // SKIP WHEN OPERATOR INTENTIONALLY TURNING
  // ==================================================
  if (abs(targetL - targetR) > 200) {
    return;
  }

  // ==================================================
  // READ CURRENT
  // ==================================================
  float cL = curLeft();
  float cR = curRight();

  // ==================================================
  // SENSOR PLAUSIBILITY
  // ==================================================
  if (cL < 0 || cR < 0 || cL > 200 || cR > 200) {
    return;
  }

  // ==================================================
  // HIGH LOAD GUARD
  // ถ้ากระแสสูง แปลว่าระบบกำลังเจอโหลดหนัก
  // ให้ WheelStuckDetection จัดการแทน
  // ==================================================
  float maxCur = max(cL, cR);

  if (maxCur > CUR_WARN_A) {
    return;
  }

  // ==================================================
  // HIGH SPEED GUARD
  // imbalance correction ใช้เฉพาะ low speed
  // ==================================================
  int16_t pwmMag = max(abs(curL), abs(curR));

  if (pwmMag > 600) {
    return;
  }

  constexpr float CUR_IMBALANCE_A = 25.0f;
  constexpr int16_t STEER_COMP = 140;

  // ==================================================
  // DEBOUNCE
  // ==================================================
  static uint8_t imbCnt = 0;

  if (abs(cL - cR) > CUR_IMBALANCE_A) {

    if (++imbCnt < 2)
      return;

  } else {

    imbCnt = 0;
    return;
  }

  // ==================================================
  // APPLY CORRECTION
  // ==================================================
  lastDriveEvent = DriveEvent::IMBALANCE;

  if (cL > cR) {

    imbalanceCorrL = -STEER_COMP;
    imbalanceCorrR = +STEER_COMP;

  } else {

    imbalanceCorrL = +STEER_COMP;
    imbalanceCorrR = -STEER_COMP;
  }
}


void startAutoReverse(uint32_t now) {

  // --------------------------------------------------
  // SAFETY GUARD
  // --------------------------------------------------
  if (getDriveSafety() != SafetyState::SAFE)
    return;

  // --------------------------------------------------
  // RECOVERY WINDOW GUARD
  // ห้าม reverse ซ้ำในช่วง recovery
  // --------------------------------------------------
  if (reverseRecoveryActive)
    return;

  // --------------------------------------------------
  // RC COMMAND GUARD
  // ต้องมีคำสั่งขับจริง
  // --------------------------------------------------
  if (abs(targetL) < 200 && abs(targetR) < 200)
    return;


  // --------------------------------------------------
  // DRIVER ENABLE GUARD
  // --------------------------------------------------
  if (driverState != DriverState::ACTIVE)
    return;

  // --------------------------------------------------
  // ANTI OSCILLATION LOCK
  // --------------------------------------------------
  if (now < reverseLockUntil_ms)
    return;

  reverseLockUntil_ms = now + 1500;

  // --------------------------------------------------
  // COOLDOWN
  // --------------------------------------------------
  static uint32_t lastReverse_ms = 0;

  constexpr uint32_t REVERSE_COOLDOWN_MS = 1200;

  if (lastReverse_ms != 0 && now - lastReverse_ms < REVERSE_COOLDOWN_MS)
    return;

  // --------------------------------------------------
  // MAX REVERSE ATTEMPT
  // --------------------------------------------------
  if (autoReverseCount >= MAX_AUTO_REVERSE) {

#if DEBUG_SERIAL
    Serial.println(F("[AUTO REV] ESCALATE -> FAULT"));
#endif

    latchFault(FaultCode::OVER_CURRENT);
    return;
  }

  // --------------------------------------------------
  // VALIDATE MOTOR LOAD
  // --------------------------------------------------
  if (abs(curL) < 200 && abs(curR) < 200)
    return;

  // --------------------------------------------------
  // REVERSE TIME ESCALATION
  // --------------------------------------------------
  uint16_t reverseTime;

  switch (autoReverseCount) {
    case 0: reverseTime = 320; break;
    case 1: reverseTime = 450; break;
    case 2: reverseTime = 650; break;
    default: reverseTime = 700; break;
  }

  // --------------------------------------------------
  // REVERSE POWER ESCALATION
  // --------------------------------------------------
  int16_t reversePWM;

  switch (autoReverseCount) {
    case 0: reversePWM = 260; break;
    case 1: reversePWM = 320; break;
    case 2: reversePWM = 380; break;
    default: reversePWM = 420; break;
  }

  reversePWM = constrain(reversePWM, 200, PWM_TOP / 2);

  // --------------------------------------------------
  // START AUTO REVERSE
  // --------------------------------------------------
  autoRev.active = true;
  autoRev.start_ms = now;
  autoRev.duration_ms = reverseTime;
  autoRev.pwm = reversePWM;

  autoReverseActive = true;
  autoReverseStart_ms = now;

  autoReverseCount++;

  lastReverse_ms = now;

  lastDriveEvent = DriveEvent::AUTO_REVERSE;

#if DEBUG_SERIAL
  Serial.print(F("[AUTO REV] START #"));
  Serial.print(autoReverseCount);
  Serial.print(F(" PWM="));
  Serial.print(reversePWM);
  Serial.print(F(" TIME="));
  Serial.println(reverseTime);
#endif
}


void forceDriveSoftStop(uint32_t now) {
  if (driveState != DriveState::SOFT_STOP) {
    driveState = DriveState::SOFT_STOP;
    driveSoftStopStart_ms = now;
  }
}


// ============================================================================
// MOTOR STALL ENERGY PROTECTION
// จำกัดพลังงานเมื่อมอเตอร์ติดหรือล้อไม่หมุน
// ============================================================================

float computeStallScale(uint32_t now, float curL_A, float curR_A) {

  float curMax = max(curL_A, curR_A);

  // guard first loop
  if (stallEnergyLast_ms == 0) {
    stallEnergyLast_ms = now;
    return 1.0f;
  }

  uint32_t dt = now - stallEnergyLast_ms;
  stallEnergyLast_ms = now;

  if (dt > 200) dt = 200;

  float dtSec = dt * 0.001f;

  // --------------------------------------------------
  // ENERGY INTEGRATION
  // --------------------------------------------------

  if (curMax > STALL_CURRENT_A) {

    float excess = curMax - STALL_CURRENT_A;

    stallEnergy += excess * dtSec * 0.02f;

  } else {

    stallEnergy *= STALL_DECAY;
  }

  if (stallEnergy < 0)
    stallEnergy = 0;

  // --------------------------------------------------
  // SCALE COMPUTATION
  // --------------------------------------------------

  if (stallEnergy < STALL_POWER_LIMIT)
    return 1.0f;

  float scale = STALL_POWER_LIMIT / stallEnergy;

  if (scale < 0.25f)
    scale = 0.25f;

  return scale;
}


void updateDriveTarget() {

  // ==================================================
  // SYSTEM SAFETY GUARD
  // ==================================================
  if (ibusCommLost || getDriveSafety() == SafetyState::EMERGENCY || systemState != SystemState::ACTIVE) {

    targetL = 0;
    targetR = 0;
    return;
  }

  // ==================================================
  // READ IBUS ONCE
  // ==================================================
  uint16_t rawThr = rcThrottle;
  uint16_t rawStr = rcSteer;

  // ==================================================
  // IBUS PLAUSIBILITY
  // ==================================================
  if (rawThr < 900 || rawThr > 2100 || rawStr < 900 || rawStr > 2100) {

    targetL = 0;
    targetR = 0;
    return;
  }

  // ==================================================
  // IBUS RECOVERY CONFIRM
  // ==================================================
  if (requireIbusConfirm) {

    bool thrNeutral = neutral(rawThr);
    bool strNeutral = neutral(rawStr);

    if (thrNeutral && strNeutral) {

      requireIbusConfirm = false;

    } else {

      targetL = 0;
      targetR = 0;
      return;
    }
  }

  // ==================================================
  // RC SPIKE + RATE FILTER
  // ==================================================
  static uint16_t lastThr = 1500;
  static uint16_t lastStr = 1500;

  constexpr int16_t RC_SPIKE_LIMIT = 300;
  constexpr int16_t RC_RATE_LIMIT = 80;

  // spike guard
  if (abs((int)rawThr - (int)lastThr) > RC_SPIKE_LIMIT)
    rawThr = lastThr;

  if (abs((int)rawStr - (int)lastStr) > RC_SPIKE_LIMIT)
    rawStr = lastStr;

  // rate limiter
  if (rawThr > lastThr + RC_RATE_LIMIT)
    rawThr = lastThr + RC_RATE_LIMIT;

  if (rawThr < lastThr - RC_RATE_LIMIT)
    rawThr = lastThr - RC_RATE_LIMIT;

  if (rawStr > lastStr + RC_RATE_LIMIT)
    rawStr = lastStr + RC_RATE_LIMIT;

  if (rawStr < lastStr - RC_RATE_LIMIT)
    rawStr = lastStr - RC_RATE_LIMIT;

  lastThr = rawThr;
  lastStr = rawStr;

  // ==================================================
  // AXIS MAPPING
  // ==================================================
  auto mapAxis = [](int16_t v) -> int16_t {
    constexpr int16_t IN_MIN = 1000;
    constexpr int16_t IN_MAX = 2000;

    constexpr int16_t DB_MIN = 1450;
    constexpr int16_t DB_MAX = 1550;

    constexpr int16_t OUT_MAX = PWM_TOP;

    if (v >= DB_MIN && v <= DB_MAX)
      return 0;

    if (v < DB_MIN) {

      long m = map(v, IN_MIN, DB_MIN, -OUT_MAX, 0);
      return constrain((int16_t)m, -OUT_MAX, 0);
    }

    long m = map(v, DB_MAX, IN_MAX, 0, OUT_MAX);
    return constrain((int16_t)m, 0, OUT_MAX);
  };

  int16_t thr = mapAxis(rawThr);
  int16_t str = mapAxis(rawStr);

  // ==================================================
  // ZERO THROTTLE SAFETY
  // ==================================================
  constexpr int16_t THR_STOP_BAND = 50;

  if (abs(thr) < THR_STOP_BAND) {

    targetL = 0;
    targetR = 0;
    return;
  }

  // ==================================================
  // LOW SPEED STEER DAMP
  // ==================================================
  int16_t absThr = abs(thr);

  if (absThr < 200)
    str = str * 0.6;

  // ==================================================
  // FIXED POINT MIXING
  // ==================================================
  int16_t blend_x1000;

  if (absThr <= 120)
    blend_x1000 = 150;

  else if (absThr >= 380)
    blend_x1000 = 1000;

  else
    blend_x1000 = ((absThr - 120) * 1000) / 260;

  int16_t arcL = constrain(thr + str, -PWM_TOP, PWM_TOP);
  int16_t arcR = constrain(thr - str, -PWM_TOP, PWM_TOP);

  int16_t k_x1000 = (abs(str) * 1000) / PWM_TOP;

  if (str < 0)
    k_x1000 = -k_x1000;

  int32_t diffL = (int32_t)thr * (1000 + k_x1000);
  int32_t diffR = (int32_t)thr * (1000 - k_x1000);

  diffL /= 1000;
  diffR /= 1000;

  int32_t outL =
    ((int32_t)arcL * (1000 - blend_x1000) + diffL * blend_x1000) / 1000;

  int32_t outR =
    ((int32_t)arcR * (1000 - blend_x1000) + diffR * blend_x1000) / 1000;

  // ==================================================
  // NORMALIZE
  // ==================================================
  int32_t maxMag = max(abs(outL), abs(outR));

  if (maxMag > PWM_TOP) {

    outL = (outL * PWM_TOP) / maxMag;
    outR = (outR * PWM_TOP) / maxMag;
  }

  targetL = constrain(outL, -PWM_TOP, PWM_TOP);
  targetR = constrain(outR, -PWM_TOP, PWM_TOP);
}

bool driveCommandZero() {
  return (targetL == 0 && targetR == 0);
}
