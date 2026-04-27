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
#include "BluetoothTelemetry.h "
#include "BluetoothCommand.h"

// ======================================================
// FUNCTION PROTOTYPES
// ======================================================

// ตรวจว่า RC อยู่ในตำแหน่ง Neutral (ใช้เป็นเงื่อนไข Arm ระบบ)
bool neutral(uint16_t v);

// ควบคุมสถานะ ignition relay (เปิด/ปิดเครื่องยนต์)
void updateIgnition();

// State machine ของเครื่องยนต์ (Idle / Starting / Running / Fault)
void updateEngineState(uint32_t now);

// ควบคุม throttle (servo) ตาม command และ state เครื่องยนต์
void updateEngineThrottle();

// ควบคุม starter relay พร้อม timeout protection
void updateStarter(uint32_t now);

// ควบคุมใบมีด (blade) พร้อม logic ความปลอดภัย
void runBlade(uint32_t now);

// ส่งข้อมูล telemetry แบบ CSV (debug / log / วิเคราะห์)
void telemetryCSV(uint32_t now, uint32_t loopStart_us);

// ตรวจ watchdog ของ subsystem (Sensor / Comms / Drive / Blade)
void monitorSubsystemWatchdogs(uint32_t now);

// งาน background สำหรับจัดการ fault และ EEPROM (non-blocking)
void backgroundFaultEEPROMTask(uint32_t now);

// จัดการ reset fault (manual / Bluetooth)
void processFaultReset(uint32_t now);

// ตรวจว่าไม่มีคำสั่งขับเคลื่อน (ใช้เป็นเงื่อนไข safety)
bool driveCommandZero();

// เคลียร์ I2C bus กรณี bus hang (industrial recovery)
void i2cBusClear();

// อ่านอุณหภูมิ driver ผ่าน PT100 (MAX31865)
bool readDriverTempsPT100(int &tL, int &tR);

// อัปเดต sensor ทั้งระบบ (current / voltage / temp)
bool updateSensors(void);

// เรียก fault → เข้าสู่ FAULT state และ latch
void requestFault(FaultCode code);

// ======================================================
// DRIVE PIPELINE (Modular Control Flow)
// ======================================================

// ตรวจ safety ก่อนอนุญาตให้ระบบ drive ทำงาน
bool driveSafetyGuard();

// คำนวณ target ความเร็วซ้าย/ขวาจาก RC input
void computeDriveTarget(float &finalTargetL, float &finalTargetR, uint32_t now);

// จำกัดค่าตาม current protection / thermal / logic constraint
void applyDriveLimits(float &finalTargetL, float &finalTargetR, float curA_L, float curA_R);

// ทำ ramp (soft start / stop / smoothing) เพื่อลด jerk
void updateDriveRamp(float finalTargetL, float finalTargetR);

// ส่ง PWM + direction ไปยัง H-Bridge จริง
void outputMotorPWM();

// ======================================================
// PT100 TEMPERATURE SENSOR (MAX31865)
// ======================================================
// Chip Select สำหรับ driver ซ้าย/ขวา
constexpr uint8_t MAX_CS_L = 49;
constexpr uint8_t MAX_CS_R = 48;

// ค่ามาตรฐาน PT100 (ใช้คำนวณอุณหภูมิ)
constexpr float RTD_RNOMINAL = 100.0f;
constexpr float RTD_RREF = 430.0f;

// ======================================================
// FAST H-BRIDGE CONTROL (DIRECT PORT MANIPULATION)
// ======================================================
// ใช้ PORTA โดยตรง → latency ต่ำสุด (ระดับ register)

// ปิดมอเตอร์ซ้าย
#define HBRIDGE_L_OFF() (PORTA &= ~0b00000011)

// เดินหน้ามอเตอร์ซ้าย
#define HBRIDGE_L_FWD() \
  { PORTA = (PORTA & ~0b00000011) | 0b00000001; }

// ถอยหลังมอเตอร์ซ้าย
#define HBRIDGE_L_REV() \
  { PORTA = (PORTA & ~0b00000011) | 0b00000010; }

// ปิดมอเตอร์ขวา
#define HBRIDGE_R_OFF() (PORTA &= ~0b00001100)

// เดินหน้ามอเตอร์ขวา
#define HBRIDGE_R_FWD() \
  { PORTA = (PORTA & ~0b00001100) | 0b00000100; }

// ถอยหลังมอเตอร์ขวา
#define HBRIDGE_R_REV() \
  { PORTA = (PORTA & ~0b00001100) | 0b00001000; }

// ปิดมอเตอร์ทั้งหมดทันที (ใช้ใน fault / emergency)
#define HBRIDGE_ALL_OFF() (PORTA &= ~0b00001111)

// ======================================================
// TIME BUDGET (REAL-TIME CONSTRAINT)
// ======================================================
// ใช้ตรวจว่าแต่ละ phase ใช้เวลานานเกินหรือไม่
#define BUDGET_SENSORS_MS 5  // sensor processing
#define BUDGET_COMMS_MS 3    // RC + Bluetooth
#define BUDGET_DRIVE_MS 2    // drive control
#define BUDGET_BLADE_MS 2    // blade control
#define BUDGET_LOOP_MS 20    // total loop (50Hz)

// ======================================================
// SENSOR CALIBRATION
// ======================================================

constexpr uint32_t SENSOR_WARMUP_MS = 2000;  // warmup ก่อนใช้งานจริง

bool currentOffsetCalibrated = false;  // flag การ calibrate ACS758

constexpr uint8_t ACS_CAL_SAMPLE_N = 32;      // จำนวน sample
constexpr uint16_t ACS_CAL_TIMEOUT_MS = 400;  // timeout calibration

// ======================================================
// TIMER CONTROL LOOP (ISR DRIVEN - 50Hz)
// ======================================================

volatile bool controlFlag = false;  // reserved (ยังไม่ใช้จริง)
volatile uint8_t tick = 0;          // base tick
volatile uint8_t controlTicks = 0;  // จำนวนรอบ control ที่ต้อง execute

// Timer2 ISR → generate control tick
ISR(TIMER2_COMPA_vect) {

  tick++;

  // ทุก 20 tick → 20ms → 50Hz
  if (tick >= 20) {

    tick = 0;

    // ป้องกัน overflow
    if (controlTicks < 255)
      controlTicks++;
  }
}

// ======================================================
// CONTROL LOOP CONFIG (DETERMINISTIC TIMING)
// ======================================================

constexpr uint32_t CONTROL_PERIOD_US = 20000;  // 20ms fixed loop
static uint32_t lastControl_us = 0;

// ======================================================
// WATCHDOG DOMAINS (MULTI-LAYER SAFETY)
// ======================================================

constexpr uint16_t WD_SENSOR_TIMEOUT_MS = 120;

// temp sensor ต้อง tolerant มากกว่า
constexpr uint8_t TEMP_TIMEOUT_MULTIPLIER = 3;

constexpr uint16_t TEMP_SENSOR_TIMEOUT_MS =
  WD_SENSOR_TIMEOUT_MS * TEMP_TIMEOUT_MULTIPLIER;


// ======================================================
// LOOP TIMING SUPERVISOR
// ======================================================
// ใช้ตรวจ overload แบบสะสม (industrial approach)

static int32_t loopOverrunAccum_us = 0;

// ======================================================
// PHASE FAULT CONFIRMATION (ANTI-FALSE TRIGGER)
// ======================================================

uint8_t commsBudgetCnt = 0;
uint8_t driveBudgetCnt = 0;
uint8_t bladeBudgetCnt = 0;

// ต้อง fail ต่อเนื่อง 3 ครั้ง
constexpr uint8_t PHASE_BUDGET_CONFIRM = 3;

constexpr int32_t LOOP_OVERRUN_FAULT_US = 40000;
constexpr int32_t LOOP_OVERRUN_RECOVER_US = 2000;

// hard kill ถ้าเกิน 2x budget
constexpr uint32_t LOOP_HARD_LIMIT_US =
  BUDGET_LOOP_MS * 2000UL;

// ======================================================
// PWM CONFIGURATION
// ======================================================

#define PWM_TOP 1067  // สำหรับ 15kHz (Timer config)

// ======================================================
// FAN CONTROL (THERMAL MANAGEMENT)
// ======================================================

// Threshold เปิด/ปิดพัดลม
constexpr int16_t FAN_L_START_C = 55;
constexpr int16_t FAN_L_FULL_C = 85;

constexpr int16_t FAN_R_START_C = 60;
constexpr int16_t FAN_R_FULL_C = 88;

// PWM constraints
constexpr uint8_t FAN_MIN_PWM = 80;   // ต่ำสุดที่หมุนได้
constexpr uint8_t FAN_PWM_HYST = 8;   // hysteresis กัน oscillation
constexpr uint8_t FAN_IDLE_PWM = 50;  // idle spin


// ======================================================
// VOLTAGE SENSOR FAULT LOGIC
// ======================================================

constexpr uint32_t VOLT_SENSOR_TIMEOUT_MS = 1500;
constexpr uint8_t VOLT_SENSOR_FAIL_COUNT = 3;


// ======================================================
// BUFFER COPY (ISR → MAIN SAFE TRANSFER)
// ======================================================
// ป้องกัน race condition ด้วย double-read verification

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

  // commit data → main context
  driveBufMain.targetL = tL1;
  driveBufMain.targetR = tR1;
  driveBufMain.curL = cL1;
  driveBufMain.curR = cR1;
}

// ======================================================
// FREE RAM MONITOR (WATCHDOG SUPPORT)
// ======================================================

#if defined(__AVR__)

extern unsigned int __heap_start;
extern void *__brkval;

int freeRam() {

  int stackTop;

  void *heapTop = (__brkval == 0)
                    ? (void *)&__heap_start
                    : __brkval;

  // stack - heap = available RAM
  return (int)&stackTop - (int)heapTop;
}

#else

int freeRam() {
  return -1;  // unsupported platform
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
  // AUX LOGIC
  // --------------------------------------------------
  updateThermalManager(now);
  updateDriverFans();
  updateVoltageWarning(now);

  updateEngineThrottle();
  updateStarter(now);

  // --------------------------------------------------
  // GIMBAL ARM / DISARM
  // --------------------------------------------------
  bool gimbalAllow =
    (systemState == SystemState::ACTIVE) && !requireIbusConfirm && !faultLatched && !killLatched;

  getGimbal().setSystemEnabled(gimbalAllow);

  if (gimbalAllow) {
    getGimbal().hardDisable(false);
    getGimbal().clearEmergency();
  } else {
    getGimbal().hardDisable(true);
  }

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

  // 🔴 เงื่อนไข: system ต้อง ACTIVE เท่านั้น
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
  // 🔴 CONTROL WATCHDOG UPDATE (สำคัญมาก)
  // ==================================================
  lastControlExec_us = micros();

  // ==================================================
  // 🔴 HARD KILL (สูงสุด - หยุดทันที)
  // ==================================================
  if (killRequest == KillType::HARD) {

    driveBufISR.targetL = 0;
    driveBufISR.targetR = 0;
    driveBufISR.curL = 0;
    driveBufISR.curR = 0;

    // 🔴 reset ramp + filter
    static float lastTargetL = 0;
    static float lastTargetR = 0;

    lastTargetL = 0;
    lastTargetR = 0;

    targetL_filtered = 0;
    targetR_filtered = 0;

    return;
  }

  // ==================================================
  // FIX 1: FIXED DT (DETERMINISTIC)
  // ==================================================
  controlDt_s = 0.02f;

  // ==================================================
  // FIX 2: SAFETY GUARD
  // ==================================================
  if (!driveSafetyGuard()) {

    driveBufISR.targetL = 0;
    driveBufISR.targetR = 0;
    driveBufISR.curL = 0;
    driveBufISR.curR = 0;

    targetL_filtered = 0;
    targetR_filtered = 0;

    return;
  }

  // ==================================================
  // STAGE 1: READ INPUT
  // ==================================================
  updateDriveTarget();

  // ==================================================
  // 🔴 FIX 3: ANTI-GLITCH FILTER (ใหม่)
  // ==================================================
  const float MAX_STEP_INPUT = 0.12f;

  float errL = targetL - targetL_filtered;
  float errR = targetR - targetR_filtered;

  if (errL > MAX_STEP_INPUT) errL = MAX_STEP_INPUT;
  if (errL < -MAX_STEP_INPUT) errL = -MAX_STEP_INPUT;

  if (errR > MAX_STEP_INPUT) errR = MAX_STEP_INPUT;
  if (errR < -MAX_STEP_INPUT) errR = -MAX_STEP_INPUT;

  targetL_filtered += errL;
  targetR_filtered += errR;

  targetL = targetL_filtered;
  targetR = targetR_filtered;

  // ==================================================
  // 🔴 FIX 4: RAMP LIMIT (S-CURVE ชั้นแรก)
  // ==================================================
  static float lastTargetL = 0.0f;
  static float lastTargetR = 0.0f;

  const float maxStep = 0.06f;

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
  // BUFFER TARGET
  // ==================================================
  driveBufISR.targetL = targetL;
  driveBufISR.targetR = targetR;

  // ==================================================
  // FIX 5: SYSTEM STATE GUARD
  // ==================================================
  if (systemState != SystemState::ACTIVE) {

    driveBufISR.curL = 0;
    driveBufISR.curR = 0;

    lastTargetL = 0;
    lastTargetR = 0;

    targetL_filtered = 0;
    targetR_filtered = 0;

    return;
  }

  // ==================================================
  // STAGE 2: DRIVE LOGIC
  // ==================================================
  runDrive(now);

  // ==================================================
  // 🔴 FIX 6: FINAL GUARD
  // ==================================================
  if (systemState != SystemState::ACTIVE || driveState == DriveState::LOCKED) {

    driveBufISR.curL = 0;
    driveBufISR.curR = 0;

    lastTargetL = 0;
    lastTargetR = 0;

    targetL_filtered = 0;
    targetR_filtered = 0;

    return;
  }

  // ==================================================
  // 🔴 FIX 7: SANITY CLAMP
  // ==================================================
  if (targetL > 1.0f) targetL = 1.0f;
  if (targetL < -1.0f) targetL = -1.0f;

  if (targetR > 1.0f) targetR = 1.0f;
  if (targetR < -1.0f) targetR = -1.0f;

  // ==================================================
  // 🔴 SOFT STOP (รองรับ S-CURVE จาก kill)
  // ==================================================
  if (killRequest == KillType::SOFT) {
    targetL *= 0.85f;
    targetR *= 0.85f;
  }

  // ==================================================
  // STAGE 3: APPLY DRIVE
  // ==================================================
  applyDrive(now);

  // ==================================================
  // WATCHDOG UPDATE
  // ==================================================
  wdDrive.lastUpdate_ms = now;

  // ==================================================
  // FEEDBACK BUFFER
  // ==================================================
  driveBufISR.curL = curL;
  driveBufISR.curR = curR;
}

void updateLightControl() {
  // 🔴 กด = ON
  if (rcLight > 1500) {
    digitalWrite(PIN_LIGHT, HIGH);
  } else {
    digitalWrite(PIN_LIGHT, LOW);
  }
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

  // 🔴 reset kill system (สำคัญมาก)
  killRequest = KillType::NONE;
  killLatched = false;
  killISRFlag = false;

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
  Serial2.begin(115200);  // Bluetooth

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

  delay(50);  // 🔴 ให้ bus stable

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

  // 🔴 ตรวจว่าทั้ง 2 ตัวต้องพร้อม
  if (!adsCurPresent || !adsVoltPresent) {
#if DEBUG_SERIAL
    Serial.println(F("[FAULT] ADS1115 NOT DETECTED"));
#endif
    requestFault(FaultCode::SENSOR_TIMEOUT);
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
  // 🔴 PWM INIT (ลำดับต้องถูก)
  // ==================================================
  setupPWM15K();     // 🔴 ต้องมาก่อน setPWM
  setupFanPWM15K();  // 🔴 init fan timer ก่อนใช้

  setPWM_L(0);
  setPWM_R(0);
  setFanPWM_L(0);
  setFanPWM_R(0);

  // ==================================================
  // 🔴 ไฟส่องสว่าง
  // ==================================================
  pinMode(PIN_LIGHT, OUTPUT);
  digitalWrite(PIN_LIGHT, LOW);

  // ==================================================
  // HARD CUT AGAIN
  // ==================================================
  driveSafe();

  // ==================================================
  // SPI TEMP
  // ==================================================
#if !TEST_MODE
  SPI.begin();
#endif

  // ==================================================
  // SERVO
  // ==================================================
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
  uint32_t now = millis();

  // ==================================================
  // 🔴 1. INPUT (เร็วสุด)
  // ==================================================

  // 🔴 HANDSHAKE + COMMAND RX (แก้ใหม่)
  static bool handshakeDone = false;
  static uint32_t lastBtRx = 0;
  static uint8_t state = 0;
  static uint8_t b1 = 0;

  // 🔴 reset handshake ถ้าเงียบเกิน 2 วิ
  if (millis() - lastBtRx > 2000) {
    handshakeDone = false;
    state = 0;
  }

  if (!handshakeDone) {

    while (Serial2.available()) {

      uint8_t b = Serial2.read();
      lastBtRx = millis();

      if (state == 0) {

        if (b == 0x55) {
          b1 = b;
          state = 1;
        }

      } else if (state == 1) {

        if (b == 0xAA) {

          // 🔴 ตอบกลับ
          Serial2.write(0xAA);
          Serial2.write(0x55);

          handshakeDone = true;

#if DEBUG_SERIAL
          Serial.println(F("[BT] HANDSHAKE OK"));
#endif
        }

        state = 0;
      }
    }

  } else {

    // 🔴 ทำงานปกติ
    btReceiveCommand();

    if (Serial2.available()) {
      lastBtRx = millis();
    }
  }

  taskComms(now);  // RC priority สูงสุด

  // ==================================================
  // 🔴 2. SENSOR UPDATE
  // ==================================================
  sensorTask(now);

  // ==================================================
  // 🔴 3. SAFETY EVALUATION
  // ==================================================
  taskSafety(now);
  btSafetyCheck();

  // ==================================================
  // 🔴 4. APPLY KILL (central decision)
  // ==================================================
  applyKillRequest(now);

  // ==================================================
  // 🔴 5. CONTROL FREEZE DETECT (industrial)
  // ==================================================
  uint32_t dtControl = micros() - lastControlExec_us;

  if (dtControl > 100000UL) {

    requestFault(FaultCode::LOGIC_WATCHDOG);

    killRequest = KillType::HARD;
    killLatched = true;
    killISRFlag = true;
  }

  // ==================================================
  // 🔴 HARD BLOCK (หยุดจริงทั้งระบบ)
  // ==================================================
  if (killLatched) {

    driveSafe();

    driveBufISR.targetL = 0;
    driveBufISR.targetR = 0;
    driveBufISR.curL = 0;
    driveBufISR.curR = 0;

    btTelemetryUpdate(now);
    processFaultReset(now);

    taskBackground(now);
    taskWatchdog();

    return;
  }

  // ==================================================
  // 🔴 7. FAULT RESET
  // ==================================================
  processFaultReset(now);

  // ==================================================
  // 🔴 8. DRIVE EVENTS
  // ==================================================
  taskDriveEvents(now);

  // ==================================================
  // 🔴 9. REAL-TIME CONTROL
  // ==================================================
  uint8_t ticksToRun;

  cli();
  ticksToRun = controlTicks;
  controlTicks = 0;
  sei();

  if (ticksToRun > 1) ticksToRun = 1;

  if (ticksToRun == 1) {

    uint32_t ctrlStart = micros();

    runControlLoop(now, loopStart_us);

    uint32_t ctrlTime = micros() - ctrlStart;

    if (ctrlTime > CONTROL_PERIOD_US) {
      requestFault(FaultCode::LOOP_OVERRUN);
    }
  }

  // ==================================================
  // 🔴 10. COPY BUFFER
  // ==================================================
  copyDriveBuffer();

  // ==================================================
  // 🔴 11. SYSTEM GATE
  // ==================================================
  if (!taskSystemGate(now, loopStart_us))
    return;

  // ==================================================
  // 🔴 12. OUTPUT
  // ==================================================
  taskBlade(now);
  taskAux(now);
  taskDriverEnable(now);

  // ==================================================
  // 🔴 13. TELEMETRY
  // ==================================================
  btTelemetryUpdate(now);

  // ==================================================
  // 🔴 14. SUPERVISOR
  // ==================================================
  taskBackground(now);
  taskLoopSupervisor(loopStart_us);
  taskWatchdog();

  // ==================================================
  // 🔴 15. ไฟส่องสว่าง
  // ==================================================
  updateLightControl();
}
