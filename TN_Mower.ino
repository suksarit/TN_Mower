// ========================================================================================
// TN_Mower.ino  - By TN MOWER
// ควบคุมรถบังคับตัดหญ้า Mega2560 + FlySky IBUS + HC160AS2 + Storm32BGC
// ========================================================================================

#define DEBUG_SERIAL 1  // 1=เปิด 0 =ปิด : DEBUG_SERIAL
#if DEBUG_SERIAL
#define DBG(x) Serial.print(x)
#define DBGL(x) Serial.println(x)
#else
#define DBG(x)
#define DBGL(x)
#endif
#define TELEMETRY_CSV 1  // 1=เปิด 0=ปิด : CSV telemetry
#define TELEMETRY_PERIOD_MS 200
#define TEST_MODE 0  // 1=ทดสอบ ไม่มีเซ็นเซฮร์ , 0=สนามจริง มีเซ็นเซอร์

#include <IBusBM.h>
#include <Servo.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_ADS1X15.h>

#include "Storm32Controller.h"
#include "BluetoothManager.h"
#include "DriveController.h"
#include "MotorDriver.h"
#include "SensorManager.h"
#include "SystemTypes.h"
#include "SafetyManager.h"
#include "GlobalState.h"
#include "HardwareConfig.h"
#include "EngineManager.h"
#include "TelemetryManager.h"
#include "FaultManager.h"
#include "CommsManager.h"
#include "DriverEnableManager.h"
#include "FanManager.h"
#include "VoltageManager.h"
#include "ThermalManager.h"
#include "DriveProtection.h"
#include "BluetoothProtocol.h"
#include "BluetoothTelemetry.h "

// ======================================================
// FUNCTION PROTOTYPES
// ======================================================

bool neutral(uint16_t v);

void updateIgnition();
void updateEngineState(uint32_t now);
void updateEngineThrottle();
void updateStarter(uint32_t now);
void runBlade(uint32_t now);
void telemetryCSV(uint32_t now, uint32_t loopStart_us);
void monitorSubsystemWatchdogs(uint32_t now);
void backgroundFaultEEPROMTask(uint32_t now);
void processFaultReset(uint32_t now);
bool driveCommandZero();
void i2cBusClear();

// FUNCTION PROTOTYPES (FINAL / MATCH ARDUINO ABI)
bool readDriverTempsPT100(int &tL, int &tR);
bool updateSensors(void);
void requestFault(FaultCode code);

// DRIVE PIPELINE
bool driveSafetyGuard();
void computeDriveTarget(float &finalTargetL, float &finalTargetR, uint32_t now);
void applyDriveLimits(float &finalTargetL, float &finalTargetR, float curA_L, float curA_R);
void updateDriveRamp(float finalTargetL, float finalTargetR);
void outputMotorPWM();

// ----- MAX31865 (PT100) -----
constexpr uint8_t MAX_CS_L = 49;
constexpr uint8_t MAX_CS_R = 48;

Adafruit_MAX31865 maxL(MAX_CS_L);
Adafruit_MAX31865 maxR(MAX_CS_R);

// PT100 constants
constexpr float RTD_RNOMINAL = 100.0f;
constexpr float RTD_RREF = 430.0f;

// FAST H-BRIDGE CONTROL (PORTA DIRECT)
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

#define HBRIDGE_ALL_OFF() (PORTA &= ~0b00001111)

// ===== Time Budget (ms) =====
#define BUDGET_SENSORS_MS 5
#define BUDGET_COMMS_MS 3
#define BUDGET_DRIVE_MS 2
#define BUDGET_BLADE_MS 2
#define BUDGET_LOOP_MS 20

// SENSOR / CALIBRATION
constexpr uint32_t SENSOR_WARMUP_MS = 2000;

bool currentOffsetCalibrated = false;

constexpr uint8_t ACS_CAL_SAMPLE_N = 32;
constexpr uint16_t ACS_CAL_TIMEOUT_MS = 400;

// ============================================================================
// TIMER CONTROL LOOP (LEVEL 4)
// ============================================================================
volatile bool controlFlag = false;
volatile uint8_t tick = 0;
volatile uint8_t controlTicks = 0;

ISR(TIMER2_COMPA_vect) {
  tick++;

  if (tick >= 20) {
    tick = 0;

    if (controlTicks < 255)  // กัน overflow
      controlTicks++;
  }
}

// ============================================================================
// CONTROL LOOP CONFIG (DETERMINISTIC)
// ============================================================================
constexpr uint32_t CONTROL_PERIOD_US = 20000;  // 20ms = 50Hz
static uint32_t lastControl_us = 0;

// ============================================================================
// WATCHDOG DOMAINS (DUAL-LAYER ARCH)
// ============================================================================
constexpr uint16_t WD_SENSOR_TIMEOUT_MS = 120;
constexpr uint8_t TEMP_TIMEOUT_MULTIPLIER = 3;  // ≥2 เท่า WD
constexpr uint16_t TEMP_SENSOR_TIMEOUT_MS =
  WD_SENSOR_TIMEOUT_MS * TEMP_TIMEOUT_MULTIPLIER;

// ============================================================================
// INDUSTRIAL LOOP TIMING SUPERVISOR
// ============================================================================
static int32_t loopOverrunAccum_us = 0;

// ============================================================================
// PHASE BUDGET CONFIRM (GLOBAL)
// ============================================================================
uint8_t commsBudgetCnt = 0;
uint8_t driveBudgetCnt = 0;
uint8_t bladeBudgetCnt = 0;

constexpr uint8_t PHASE_BUDGET_CONFIRM = 3;

constexpr int32_t LOOP_OVERRUN_FAULT_US = 40000;   // sustained overload
constexpr int32_t LOOP_OVERRUN_RECOVER_US = 2000;  // decay per healthy loop
constexpr uint32_t LOOP_HARD_LIMIT_US =
  BUDGET_LOOP_MS * 2000UL;  // 2x budget immediate kill

// ============================================================================
// PWM CONFIG
// ============================================================================
#define PWM_TOP 1067  // 15 kHz

// ============================================================================
// FAN CONTROL (DRIVER COOLING)
// ============================================================================

// LEFT DRIVER
constexpr int16_t FAN_L_START_C = 55;  // เริ่มหมุน
constexpr int16_t FAN_L_FULL_C = 85;   // เต็มรอบ

// RIGHT DRIVER
constexpr int16_t FAN_R_START_C = 60;
constexpr int16_t FAN_R_FULL_C = 88;

// PWM behavior
constexpr uint8_t FAN_MIN_PWM = 80;   // ต่ำกว่านี้พัดลมไม่หมุน
constexpr uint8_t FAN_PWM_HYST = 8;   // hysteresis กันแกว่ง
constexpr uint8_t FAN_IDLE_PWM = 50;  // ~20% idle spin

constexpr uint32_t VOLT_SENSOR_TIMEOUT_MS = 1500;  // เดิม 500 → 1.5 วินาที
constexpr uint8_t VOLT_SENSOR_FAIL_COUNT = 3;      // ต้อง fail 3 รอบติดก่อน latch

// ============================================================================
// UTIL
// ============================================================================

// ============================================================================
// BUFFER COPY (ATOMIC - ISR SAFE)
// ============================================================================
inline void copyDriveBuffer() {

  float tL1, tR1, cL1, cR1;
  float tL2, tR2, cL2, cR2;

  do {
    cli();

    tL1 = driveBufISR.targetL;
    tR1 = driveBufISR.targetR;
    cL1 = driveBufISR.curL;
    cR1 = driveBufISR.curR;

    sei();

    cli();

    tL2 = driveBufISR.targetL;
    tR2 = driveBufISR.targetR;
    cL2 = driveBufISR.curL;
    cR2 = driveBufISR.curR;

    sei();

  } while (tL1 != tL2 || tR1 != tR2 || cL1 != cL2 || cR1 != cR2);

  driveBufMain.targetL = tL1;
  driveBufMain.targetR = tR1;
  driveBufMain.curL = cL1;
  driveBufMain.curR = cR1;
}

uint8_t crc8_update(uint8_t crc, uint8_t data) {
  crc ^= data;

  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x80)
      crc = (crc << 1) ^ 0x07;
    else
      crc <<= 1;
  }

  return crc;
}

// ======================================================
// FREE RAM (SRAM MONITOR)
// AVR SRAM calculation
//
// วิธีคำนวณ
// free RAM = stack_top - heap_top
//
// heap_top = __brkval
// ถ้า heap ยังไม่ถูก allocate (__brkval == 0)
// ให้ใช้ __heap_start แทน
//
// NOTE:
// __heap_start และ __brkval เป็น symbol ภายในของ AVR libc
// บาง toolchain อาจไม่ได้ประกาศใน header
// จึงต้อง declare extern เอง
// ======================================================

#if defined(__AVR__)

extern unsigned int __heap_start;
extern void *__brkval;

int freeRam() {
  int stackTop;

  void *heapTop = (__brkval == 0)
                    ? (void *)&__heap_start
                    : __brkval;

  return (int)&stackTop - (int)heapTop;
}

#else

int freeRam() {
  return -1;  // not supported on non-AVR platforms
}

#endif


void setupControlTimer() {
  cli();

  TCCR2A = 0;
  TCCR2B = 0;

  // CTC mode
  TCCR2A |= (1 << WGM21);

  // prescaler 1024 → 16MHz / 1024 = 15625 Hz
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);

  // 🔴 20ms → 312 ticks (15625 * 0.02)
  OCR2A = 312 - 1;

  TIMSK2 |= (1 << OCIE2A);

  sei();
}


bool driveSafetyGuard() {
  if (systemState == SystemState::FAULT || driveState == DriveState::LOCKED) {
    // 🔴 ห้ามแตะ PWM
    return false;
  }

  bool thrNeutral = neutral(rcThrottle);
  bool strNeutral = neutral(rcSteer);

  if (driverRearmRequired) {
    if (thrNeutral && strNeutral)
      driverRearmRequired = false;

    return false;
  }

  return true;
}

void handleFaultImmediateCut() {
#if DEBUG_SERIAL
  static bool printed = false;
  if (!printed) {
    Serial.println(F("[FAULT] HARD KILL"));
    printed = true;
  }
#endif

  // --------------------------------------------------
  // 🔴 1) ตัดไฟไดรเวอร์ (ของจริง)
  // --------------------------------------------------
  digitalWrite(PIN_DRV_ENABLE, LOW);

  // --------------------------------------------------
  // 🔴 2) ห้ามแตะ curL / PWM
  // (ปล่อยให้ applyDrive จัดการ)
  // --------------------------------------------------

  // --------------------------------------------------
  // 🔴 3) ตัดใบมีด (อันนี้ถูกแล้ว)
  // --------------------------------------------------
  bladeServo.writeMicroseconds(1000);

  // --------------------------------------------------
  // 🔴 4) ตัด starter
  // --------------------------------------------------
  starterActive = false;
  digitalWrite(RELAY_STARTER, LOW);
}

void updateSystemState(uint32_t now) {
  static uint32_t ibusStableStart_ms = 0;

  bool ignitionOn = ignitionActive;

  // --------------------------------------------------
  // AUTO REVERSE COUNTER RESET ON FAULT / DISARM
  // --------------------------------------------------

  if (systemState != SystemState::ACTIVE) {

    autoReverseCount = 0;
    autoReverseActive = false;
  }

  // ------------------------------------------------
  // HARD FAULT PRIORITY
  // ------------------------------------------------
  if (faultLatched) {
    systemState = SystemState::FAULT;
    return;
  }

  // ------------------------------------------------
  // IGNITION OFF = DISARM SYSTEM
  // ------------------------------------------------
  if (!ignitionOn) {
    systemState = SystemState::INIT;
    ibusStableStart_ms = 0;
    return;
  }

  switch (systemState) {
    // --------------------------------------------
    case SystemState::INIT:
      {
        // --- RESET DRIVER REARM ---
        extern bool driverRearmRequired;
        driverRearmRequired = true;

        if (ibusCommLost) {
          ibusStableStart_ms = 0;
          return;
        }

        if (ibusStableStart_ms == 0) {
          ibusStableStart_ms = now;
          return;
        }

        if (now - ibusStableStart_ms < 1000)
          return;

        bool thrNeutral = neutral(rcThrottle);
        bool strNeutral = neutral(rcSteer);

        // ต้องผ่าน neutral ก่อน
        if (!rcNeutralConfirmed) {

          if (thrNeutral && strNeutral) {
            rcNeutralConfirmed = true;
          }

          return;
        }

        if (!calibrateCurrentOffsetNonBlocking(now))
          return;

        systemState = SystemState::ACTIVE;
        systemActiveStart_ms = now;

        break;
      }

    // --------------------------------------------
    case SystemState::ACTIVE:
      {
        static uint32_t ibusLostStart = 0;

        if (ibusCommLost) {
          if (ibusLostStart == 0)
            ibusLostStart = now;

          // 🔴 ต้องหลุดต่อเนื่อง >300ms ถึง reset
          if (now - ibusLostStart > 300) {
            systemState = SystemState::INIT;
            ibusStableStart_ms = 0;
            ibusLostStart = 0;
          }
        } else {
          ibusLostStart = 0;
        }

        break;
      }

    // --------------------------------------------
    case SystemState::FAULT:
      break;

    default:
      requestFault(FaultCode::LOGIC_WATCHDOG);
      systemState = SystemState::FAULT;
      break;
  }
}

void taskComms(uint32_t now) {
  uint32_t tComms_us = micros();

  updateComms(now);
  updateRcCache();
  updateIgnition();
  updateSystemState(now);

  uint32_t dtComms = micros() - tComms_us;

  if (dtComms > BUDGET_COMMS_MS * 1000UL) {
    if (++commsBudgetCnt >= PHASE_BUDGET_CONFIRM)
      requestFault(FaultCode::COMMS_TIMEOUT);
  } else {
    commsBudgetCnt = 0;
  }
}

void taskDriveEvents(uint32_t now) {
  cli();  // 🔴 lock interrupt

  curA_snapshot[0] = getMotorCurrentL();
  curA_snapshot[1] = getMotorCurrentR();

  sei();  // 🔴 unlock

  curA_snapshot[2] = curA[2];
  curA_snapshot[3] = curA[3];

  updateEngineState(now);
  idleCurrentAutoRezero(now);
}

void taskSafety(uint32_t now) {
  SafetyInput sin;

  sin.curA[0] = curA[0];
  sin.curA[1] = curA[1];
  sin.curA[2] = curA[2];
  sin.curA[3] = curA[3];

  sin.tempDriverL = tempDriverL;
  sin.tempDriverR = tempDriverR;
  sin.faultLatched = faultLatched;
  sin.driveEvent = lastDriveEvent;

  SafetyThresholds sth = {
    CUR_WARN_A,
    CUR_LIMP_A,
    TEMP_WARN_C,
    TEMP_LIMP_C
  };

  SafetyState rawSafety = evaluateSafetyRaw(sin, sth);

  updateSafetyStability(
    rawSafety,
    now,
    autoReverseCount,
    autoReverseActive,
    lastDriveEvent);
}

bool taskSystemGate(uint32_t now, uint32_t loopStart_us) {
  // 🔴 HARD KILL PRIORITY สูงสุด
  if (killRequest == KillType::HARD) {
    handleFaultImmediateCut();

#if TELEMETRY_CSV
    telemetryCSV(now, loopStart_us);
#endif

    digitalWrite(PIN_HW_WD_HB,
                 !digitalRead(PIN_HW_WD_HB));

    return false;
  }

  if (systemState != SystemState::ACTIVE) {
    digitalWrite(PIN_DRV_ENABLE, LOW);

#if TELEMETRY_CSV
    telemetryCSV(now, loopStart_us);
#endif

    digitalWrite(PIN_HW_WD_HB,
                 !digitalRead(PIN_HW_WD_HB));

    return false;
  }

  return true;
}

void taskBlade(uint32_t now) {
  uint32_t tBlade_us = micros();

  runBlade(now);

  uint32_t dtBlade = micros() - tBlade_us;

  if (dtBlade > BUDGET_BLADE_MS * 1000UL) {
    if (++bladeBudgetCnt >= PHASE_BUDGET_CONFIRM)
      requestFault(FaultCode::BLADE_TIMEOUT);
  } else {
    bladeBudgetCnt = 0;
    wdBlade.lastUpdate_ms = now;
  }
}

void taskAux(uint32_t now) {
  // --------------------------------------------------
  // AUX LOGIC (COMPUTE PHASE)
  // --------------------------------------------------
  updateThermalManager(now);
  updateDriverFans();
  updateVoltageWarning(now);

  updateEngineThrottle();
  updateStarter(now);

  // --------------------------------------------------
  // GIMBAL
  // --------------------------------------------------
  getGimbal().setSystemEnabled(
    systemState == SystemState::ACTIVE && !requireIbusConfirm);

  getGimbal().update(now);
}

void taskDriverEnable(uint32_t now) {
  DriverEnableContext ctx;

  ctx.systemState = systemState;
  ctx.driveState = driveState;
  ctx.driverState = driverState;

  ctx.faultLatched = faultLatched;
  ctx.driverRearmRequired = driverRearmRequired;
  ctx.requireIbusConfirm = requireIbusConfirm;
  ctx.autoReverseActive = autoReverseActive;

  ctx.rcThrottle = rcThrottle;
  ctx.rcSteer = rcSteer;

  ctx.curL = driveBufMain.curL;
  ctx.curR = driveBufMain.curR;
  ctx.targetL = driveBufMain.targetL;
  ctx.targetR = driveBufMain.targetR;

  ctx.now = now;
  ctx.systemActiveStart_ms = systemActiveStart_ms;
  ctx.driverStateStart_ms = driverStateStart_ms;
  ctx.driverEnabled_ms = driverEnabled_ms;
  ctx.driverActiveStart_ms = driverActiveStart_ms;

  updateDriverEnable(ctx);

  driverState = ctx.driverState;
  driverStateStart_ms = ctx.driverStateStart_ms;
  driverEnabled_ms = ctx.driverEnabled_ms;
  driverActiveStart_ms = ctx.driverActiveStart_ms;

  curL = ctx.curL;
  curR = ctx.curR;
  targetL = ctx.targetL;
  targetR = ctx.targetR;
}

void taskLoopSupervisor(uint32_t loopStart_us) {
  uint32_t loopTime_us = micros() - loopStart_us;
  uint32_t loopBudget_us = BUDGET_LOOP_MS * 1000UL;

  if (loopTime_us > LOOP_HARD_LIMIT_US)
    requestFault(FaultCode::LOOP_OVERRUN);

  if (loopTime_us > loopBudget_us)
    loopOverrunAccum_us += (loopTime_us - loopBudget_us);
  else {
    loopOverrunAccum_us -= LOOP_OVERRUN_RECOVER_US;
    if (loopOverrunAccum_us < 0)
      loopOverrunAccum_us = 0;
  }

  if (loopOverrunAccum_us > LOOP_OVERRUN_FAULT_US)
    requestFault(FaultCode::LOOP_OVERRUN);
}

void taskWatchdog() {
  int freeMem = freeRam();

  if (freeMem < 600) {
    requestFault(FaultCode::LOGIC_WATCHDOG);
  }

  bool wdHealthy =
    !wdSensor.faulted && !wdComms.faulted && !wdDrive.faulted && !wdBlade.faulted && !faultLatched;

  // 🔴 เพิ่มเงื่อนไข: system ต้อง ACTIVE เท่านั้น
  if (wdHealthy && systemState == SystemState::ACTIVE) {
    wdt_reset();

    digitalWrite(PIN_HW_WD_HB,
                 !digitalRead(PIN_HW_WD_HB));
  } else {
    // 🔴 ไม่ feed → MCU reset
    digitalWrite(PIN_HW_WD_HB, LOW);
  }
}

void taskBackground(uint32_t now) {
  // WATCHDOG (central)
  monitorSubsystemWatchdogs(now);

  // EEPROM (central)
  backgroundFaultEEPROMTask(now);
}

void runControlLoop(uint32_t now, uint32_t loopStart_us) {
  // ==================================================
  // 🔴 HARD KILL (สูงสุด - หยุดทันที)
  // ==================================================
  if (killRequest == KillType::HARD) {
    driveBufISR.targetL = 0;
    driveBufISR.targetR = 0;
    driveBufISR.curL = 0;
    driveBufISR.curR = 0;

    // 🔴 reset ramp state ด้วย (กันเด้งตอนกลับมา)
    static float lastTargetL = 0;
    static float lastTargetR = 0;
    lastTargetL = 0;
    lastTargetR = 0;

    return;
  }

  // ==================================================
  // FIX 1: FIXED DT (DETERMINISTIC)
  // ==================================================
  controlDt_s = 0.02f;

  // ==================================================
  // FIX 2: SAFETY GUARD (ก่อนทุกอย่าง)
  // ==================================================
  if (!driveSafetyGuard()) {
    driveBufISR.targetL = 0;
    driveBufISR.targetR = 0;
    driveBufISR.curL = 0;
    driveBufISR.curR = 0;
    return;
  }

  // ==================================================
  // STAGE 1: READ INPUT → TARGET
  // ==================================================
  updateDriveTarget();

  // ==================================================
  // 🔴 FIX 3: RAMP LIMIT (กันกระชากจริง)
  // ==================================================
  static float lastTargetL = 0.0f;
  static float lastTargetR = 0.0f;

  const float maxStep = 0.06f;  // 🔴 ปรับได้ (ยิ่งน้อยยิ่งนุ่ม)

  float dL = targetL - lastTargetL;
  float dR = targetR - lastTargetR;

  if (dL > maxStep) dL = maxStep;
  if (dL < -maxStep) dL = -maxStep;

  if (dR > maxStep) dR = maxStep;
  if (dR < -maxStep) dR = -maxStep;

  targetL = lastTargetL + dL;
  targetR = lastTargetR + dR;

  lastTargetL = targetL;
  lastTargetR = targetR;

  // ==================================================
  // BUFFER TARGET (หลัง ramp เท่านั้น)
  // ==================================================
  driveBufISR.targetL = targetL;
  driveBufISR.targetR = targetR;

  // ==================================================
  // FIX 4: SYSTEM STATE GUARD
  // ==================================================
  if (systemState != SystemState::ACTIVE) {
    driveBufISR.curL = 0;
    driveBufISR.curR = 0;

    // 🔴 reset ramp กันค้าง
    lastTargetL = 0;
    lastTargetR = 0;

    return;
  }

  // ==================================================
  // STAGE 2: DRIVE LOGIC (event / protection)
  // ==================================================
  runDrive(now);

  // ==================================================
  // 🔴 FIX 5: FINAL GUARD (กันหลุดหลัง runDrive)
  // ==================================================
  if (systemState != SystemState::ACTIVE || driveState == DriveState::LOCKED) {
    driveBufISR.curL = 0;
    driveBufISR.curR = 0;

    // 🔴 reset ramp กันเด้งตอน unlock
    lastTargetL = 0;
    lastTargetR = 0;

    return;
  }

  // ==================================================
  // 🔴 FIX 6: SANITY CLAMP (กันค่าเพี้ยน)
  // ==================================================
  if (targetL > 1.0f) targetL = 1.0f;
  if (targetL < -1.0f) targetL = -1.0f;

  if (targetR > 1.0f) targetR = 1.0f;
  if (targetR < -1.0f) targetR = -1.0f;

  // ==================================================
  // STAGE 3: APPLY DRIVE (OWNER OF PWM)
  // ==================================================
  applyDrive(now);

  // ==================================================
  // FIX 7: WATCHDOG UPDATE
  // ==================================================
  wdDrive.lastUpdate_ms = now;

  // ==================================================
  // FEEDBACK BUFFER
  // ==================================================
  driveBufISR.curL = curL;
  driveBufISR.curR = curR;
}


void setup() {

  // ==================================================
  // 🔴 HARD SAFE FIRST (กันพุ่งก่อนทุกอย่าง)
  // ==================================================
  cli();

  pinMode(PIN_DRV_ENABLE, OUTPUT);
  digitalWrite(PIN_DRV_ENABLE, LOW);

  // kill PWM hardware
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR4A = 0;
  TCCR4B = 0;

  OCR3A = 0;
  OCR4A = 0;

  // kill H-bridge
  HBRIDGE_ALL_OFF();

  // reset control state
  curL = 0;
  curR = 0;
  targetL = 0;
  targetR = 0;

  sei();

  // ==================================================
  // 🔴 DETECT WDT RESET
  // ==================================================
  if (MCUSR & (1 << WDRF)) {
#if DEBUG_SERIAL
    Serial.println(F("[BOOT] WDT RESET"));
#endif
    systemState = SystemState::INIT;

    requireIbusConfirm = true;
    rcNeutralConfirmed = false;
  }

  MCUSR = 0;

  // ==================================================
  // TIMER CONTROL LOOP
  // ==================================================
  setupControlTimer();

  // ==================================================
  // SERIAL
  // ==================================================
#if DEBUG_SERIAL || TELEMETRY_CSV
  Serial.begin(115200);
#endif

  Serial1.begin(115200);  // iBUS
  Serial2.begin(115200);  // 🔴 Bluetooth HC-05

  // ==================================================
  // 🔴 INIT BLUETOOTH PROTOCOL (สำคัญ)
  // ==================================================
  btProtocolInit();   // ✅ ต้องอยู่หลัง Serial2.begin เท่านั้น

  ibus.begin(Serial1);
  getGimbal().begin();
  getGimbal().forceOff();

  // ==================================================
  // I2C
  // ==================================================
  i2cBusClear();
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(6000, true);

  // ==================================================
  // ADS INIT
  // ==================================================
  adsCurPresent = adsCur.begin(0x48);
  if (adsCurPresent) {
    adsCur.setGain(GAIN_ONE);
    adsCur.setDataRate(RATE_ADS1115_250SPS);
  }

  adsVoltPresent = adsVolt.begin(0x49);
  if (adsVoltPresent) {
    adsVolt.setGain(GAIN_ONE);
    adsVolt.setDataRate(RATE_ADS1115_250SPS);
  }

  // ==================================================
  // GPIO
  // ==================================================
  pinMode(PIN_HW_WD_HB, OUTPUT);
  digitalWrite(PIN_HW_WD_HB, LOW);

  pinMode(DIR_L1, OUTPUT);
  pinMode(DIR_L2, OUTPUT);
  pinMode(DIR_R1, OUTPUT);
  pinMode(DIR_R2, OUTPUT);

  pinMode(PIN_CUR_TRIP, INPUT_PULLUP);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  pinMode(RELAY_WARN, OUTPUT);
  digitalWrite(RELAY_WARN, LOW);

  pinMode(RELAY_STARTER, OUTPUT);
  digitalWrite(RELAY_STARTER, LOW);

  pinMode(RELAY_IGNITION, OUTPUT);
  digitalWrite(RELAY_IGNITION, LOW);

  pinMode(FAN_L, OUTPUT);
  digitalWrite(FAN_L, LOW);

  pinMode(FAN_R, OUTPUT);
  digitalWrite(FAN_R, LOW);

  // ==================================================
  // HARD CUT AGAIN (double safety)
  // ==================================================
  driveSafe();

  // ==================================================
  // SPI TEMP
  // ==================================================
#if !TEST_MODE
  SPI.begin();
  maxL.begin(MAX31865_3WIRE);
  maxR.begin(MAX31865_3WIRE);
#endif

  // ==================================================
  // PWM INIT
  // ==================================================
  setPWM_L(0);
  setPWM_R(0);

  setupPWM15K();
  setupFanPWM15K();

  bladeServo.attach(SERVO_ENGINE_PIN);
  bladeServo.writeMicroseconds(1000);

  // ==================================================
  // INIT STATE
  // ==================================================
  for (uint8_t i = 0; i < 4; i++) {
    curA[i] = 0.0f;
    overCurCnt[i] = 0;
  }

  tempDriverL = 0;
  tempDriverR = 0;
  engineVolt = 0.0f;

  systemState = SystemState::INIT;
  driveState = DriveState::IDLE;
  bladeState = BladeState::IDLE;

  faultLatched = false;
  activeFault = FaultCode::NONE;

  revBlockUntilL = 0;
  revBlockUntilR = 0;

  driveSoftStopStart_ms = 0;
  bladeSoftStopStart_ms = 0;

  engineStopped_ms = 0;
  starterActive = false;

  lastIbusByte_ms = millis();
  ibusCommLost = true;

  // ==================================================
  // WATCHDOG INIT
  // ==================================================
  {
    uint32_t now = millis();

    wdSensor.lastUpdate_ms = now;
    wdComms.lastUpdate_ms = now;
    wdDrive.lastUpdate_ms = now;
    wdBlade.lastUpdate_ms = now;

    wdSensor.faulted = false;
    wdComms.faulted = false;
    wdDrive.faulted = false;
    wdBlade.faulted = false;
  }

  // ==================================================
  // 🔴 START WDT (ท้ายสุดเท่านั้น)
  // ==================================================
  wdt_reset();
  wdt_enable(WDTO_1S);
}

void loop() {

  uint32_t loopStart_us = micros();

  // ==================================================
  // REAL-TIME CONTROL
  // ==================================================
  uint8_t ticksToRun;

  cli();
  ticksToRun = controlTicks;
  controlTicks = 0;
  sei();

  if (ticksToRun > 1) ticksToRun = 1;

  if (ticksToRun == 1) {
    uint32_t ctrlStart = micros();
    uint32_t ctrlNow = millis();

    runControlLoop(ctrlNow, loopStart_us);

    uint32_t ctrlTime = micros() - ctrlStart;

    if (ctrlTime > CONTROL_PERIOD_US) {
      requestFault(FaultCode::LOOP_OVERRUN);
    }
  }

  // ==================================================
  // COPY BUFFER
  // ==================================================
  copyDriveBuffer();

  uint32_t now = millis();

  // ==================================================
  // 🔴 BLUETOOTH (PRIORITY INPUT)
  // ==================================================
  btProtocolUpdate();       // รับคำสั่ง (STOP / HB / RESET)
  btTelemetryUpdate(now);   // ส่งข้อมูล

  // ==================================================
  // BACKGROUND (RC มาก่อนเสมอ)
  // ==================================================
  taskComms(now);           // RC = priority สูงสุด

  sensorTask(now);
  processFaultReset(now);

  taskDriveEvents(now);

  // ==================================================
  // 🔴 SAFETY
  // ==================================================
  taskSafety(now);

  // 🔴 🔥 จุดสำคัญที่สุด (คุณยังไม่มี)
  applyKillRequest(now);

  // ==================================================
  // 🔴 FAULT DEBUG
  // ==================================================
  static FaultCode lastFault = FaultCode::NONE;

  FaultCode currentFault = getActiveFault();

  if (currentFault != lastFault) {
#if DEBUG_SERIAL
    Serial.print(F("[FAULT] "));
    Serial.println(faultCodeToString(currentFault));
#endif
    lastFault = currentFault;
  }

  // ==================================================
  // SYSTEM GATE
  // ==================================================
  if (!taskSystemGate(now, loopStart_us))
    return;

  // ==================================================
  // OTHER TASKS
  // ==================================================
  taskBlade(now);
  taskAux(now);
  taskDriverEnable(now);

  // ==================================================
  // SUPERVISOR
  // ==================================================
  taskBackground(now);
  taskLoopSupervisor(loopStart_us);
  taskWatchdog();
}



