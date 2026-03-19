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
#include "Storm32Controller.h"
#include <Adafruit_ADS1X15.h>

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
void latchFault(FaultCode code);

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

  TCCR2A |= (1 << WGM21);

  // prescaler 64 → 250kHz
  TCCR2B |= (1 << CS22);

  // 1ms → 250 ticks
  OCR2A = 249;

  TIMSK2 |= (1 << OCIE2A);

  sei();
}


bool driveSafetyGuard() {

  if (systemState == SystemState::FAULT || driveState == DriveState::LOCKED) {

    driveSafe();

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
    Serial.println(F("[FAULT] IMMEDIATE CUT"));
    printed = true;
  }
#endif

  // --------------------------------------------------
  // 1) DISABLE DRIVER POWER
  // --------------------------------------------------
  digitalWrite(PIN_DRV_ENABLE, LOW);

  // --------------------------------------------------
  // 2) CUT DRIVE OUTPUT
  // --------------------------------------------------
  curL = 0;
  curR = 0;
  targetL = 0;
  targetR = 0;

  // ใช้ driveSafe เฉพาะ kill จริง
  driveSafe();

  // --------------------------------------------------
  // 3) CUT BLADE THROTTLE
  // --------------------------------------------------
  bladeServo.writeMicroseconds(1000);

  // --------------------------------------------------
  // 4) CUT STARTER
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
        if (ibusCommLost) {
          systemState = SystemState::INIT;
          ibusStableStart_ms = 0;
        }
        break;
      }

    // --------------------------------------------
    case SystemState::FAULT:
      break;

    default:
      latchFault(FaultCode::LOGIC_WATCHDOG);
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
      latchFault(FaultCode::COMMS_TIMEOUT);
  } else {
    commsBudgetCnt = 0;
  }
}

void taskDriveEvents(uint32_t now)
{
  // ==================================================
  // SNAPSHOT CURRENT (ใช้ค่าเดียวกับ CONTROL)
  // ==================================================
  // 🔴 ต้องใช้ filtered current เท่านั้น
  curA_snapshot[0] = getMotorCurrentL();
  curA_snapshot[1] = getMotorCurrentR();

  // ช่องอื่น (ถ้ามีจริง)
  curA_snapshot[2] = curA[2];
  curA_snapshot[3] = curA[3];

  // ==================================================
  // ENGINE STATE / AUTO ZERO
  // ==================================================
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

  if (faultLatched)
    systemState = SystemState::FAULT;

  processFaultReset(now);
}

bool taskSystemGate(uint32_t now, uint32_t loopStart_us) {
  bool emergencyActive =
    (systemState == SystemState::FAULT) || (getDriveSafety() == SafetyState::EMERGENCY);

  if (systemState != SystemState::ACTIVE || emergencyActive) {
    handleFaultImmediateCut();

    digitalWrite(PIN_DRV_ENABLE, LOW);

    backgroundFaultEEPROMTask(now);
    monitorSubsystemWatchdogs(now);

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
      latchFault(FaultCode::BLADE_TIMEOUT);
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
    latchFault(FaultCode::LOOP_OVERRUN);

  if (loopTime_us > loopBudget_us)
    loopOverrunAccum_us += (loopTime_us - loopBudget_us);
  else {
    loopOverrunAccum_us -= LOOP_OVERRUN_RECOVER_US;
    if (loopOverrunAccum_us < 0)
      loopOverrunAccum_us = 0;
  }

  if (loopOverrunAccum_us > LOOP_OVERRUN_FAULT_US)
    latchFault(FaultCode::LOOP_OVERRUN);
}

void taskWatchdog() {
  int freeMem = freeRam();

  if (freeMem < 600) {
    latchFault(FaultCode::LOGIC_WATCHDOG);
  }

  bool wdHealthy =
    !wdSensor.faulted && !wdComms.faulted && !wdDrive.faulted && !wdBlade.faulted && !faultLatched;

  if (wdHealthy) {
    wdt_reset();

    digitalWrite(PIN_HW_WD_HB,
                 !digitalRead(PIN_HW_WD_HB));
  } else {
    digitalWrite(PIN_HW_WD_HB, LOW);
  }
}

void taskBackground(uint32_t now) {
  monitorSubsystemWatchdogs(now);
  backgroundFaultEEPROMTask(now);
}

void runControlLoop(uint32_t now, uint32_t loopStart_us) {
  // ==================================================
  // 🔴 FIX 1: USE FIXED DT (DETERMINISTIC CONTROL)
  // ==================================================
  // CONTROL LOOP = 20ms (50Hz)
  controlDt_s = 0.02f;

  // ==================================================
  // 🔴 FIX 2: HARD SAFETY GUARD (BEFORE ANY CONTROL)
  // ==================================================
  // กันกรณี FAULT / LOCK / ยังไม่ rearm
  if (!driveSafetyGuard()) {

    // force zero output (ISR buffer)
    driveBufISR.targetL = 0;
    driveBufISR.targetR = 0;
    driveBufISR.curL = 0;
    driveBufISR.curR = 0;

    return;
  }

  // ==================================================
  // STAGE 1: UPDATE TARGET FROM RC
  // ==================================================
  updateDriveTarget();

  driveBufISR.targetL = targetL;
  driveBufISR.targetR = targetR;

  // ==================================================
  // 🔴 FIX 3: SYSTEM STATE GUARD (DOUBLE LAYER SAFETY)
  // ==================================================
  if (systemState != SystemState::ACTIVE) {

    driveSafe();  // ตัด PWM ทันที

    driveBufISR.curL = 0;
    driveBufISR.curR = 0;

    return;
  }

  // ==================================================
  // STAGE 2: DRIVE STATE + CONTROL
  // ==================================================
  runDrive(now);

  // ==================================================
  // 🔴 FIX 4: FINAL GUARD BEFORE OUTPUT (CRITICAL)
  // ==================================================
  if (systemState != SystemState::ACTIVE || driveState == DriveState::LOCKED) {

    driveSafe();

    driveBufISR.curL = 0;
    driveBufISR.curR = 0;

    return;
  }

  applyDrive(now);

  // ==================================================
  // 🔴 FIX 5: WATCHDOG FEED (REAL-TIME DOMAIN)
  // ==================================================
  wdDrive.lastUpdate_ms = now;

  // ==================================================
  // UPDATE FEEDBACK BUFFER
  // ==================================================
  driveBufISR.curL = curL;
  driveBufISR.curR = curR;
}

void setup() {

  setupControlTimer();
// --------------------------------------------------
// SERIAL (DEBUG)
// --------------------------------------------------
#if DEBUG_SERIAL || TELEMETRY_CSV
  Serial.begin(115200);
#endif
  // --------------------------------------------------
  // SERIAL COMMUNICATION
  // --------------------------------------------------
  Serial1.begin(115200);  // iBUS
  Serial2.begin(115200);  // Storm32

  ibus.begin(Serial1);  // iBUS = INPUT ONLY
  getGimbal().begin();  // Storm32 uses Seria

  getGimbal().forceOff();  // <<< HARD LOCK getGimbal() at boot

  // --------------------------------------------------
  // I2C
  // --------------------------------------------------
  i2cBusClear();
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(6000, true);

  // --------------------------------------------------
  // ADS1115 DETECT (ROBUST INIT)
  // --------------------------------------------------

  adsCurPresent = adsCur.begin(0x48);
  if (adsCurPresent) {
    adsCur.setGain(GAIN_ONE);
    adsCur.setDataRate(RATE_ADS1115_250SPS);
#if DEBUG_SERIAL
    Serial.println(F("[BOOT] ADS CUR OK (0x48)"));
#endif
  } else {
#if DEBUG_SERIAL
    Serial.println(F("[BOOT] ADS CUR NOT FOUND (0x48)"));
#endif
  }

  adsVoltPresent = adsVolt.begin(0x49);
  if (adsVoltPresent) {
    adsVolt.setGain(GAIN_ONE);
    adsVolt.setDataRate(RATE_ADS1115_250SPS);
#if DEBUG_SERIAL
    Serial.println(F("[BOOT] ADS VOLT OK (0x49)"));
#endif
  } else {
#if DEBUG_SERIAL
    Serial.println(F("[BOOT] ADS VOLT NOT FOUND (0x49)"));
#endif
  }

  // --------------------------------------------------
  // READ LAST FAULT FROM EEPROM (FAULT HISTORY)
  // --------------------------------------------------
  uint8_t raw;
  EEPROM.get(100, raw);

  if (!isValidEnum<FaultCode>(raw)) {
    raw = 0;  // หรือ FaultCode::NONE
  }

  FaultCode lastFault = static_cast<FaultCode>(raw);

#if DEBUG_SERIAL
  Serial.print(F("[BOOT] LAST FAULT = "));
  Serial.println((uint8_t)lastFault);
#endif

#if TEST_MODE && DEBUG_SERIAL
  Serial.println();
  Serial.println(F("==================================="));
  Serial.println(F("!!! TEST MODE ACTIVE !!!"));
  Serial.println(F("NO REAL SENSORS / BENCH TEST ONLY"));
  Serial.println(F("DO NOT USE IN FIELD OPERATION"));
  Serial.println(F("==================================="));
#endif

  // --------------------------------------------------
  // GPIO OUTPUT
  // --------------------------------------------------
  pinMode(PIN_HW_WD_HB, OUTPUT);
  digitalWrite(PIN_HW_WD_HB, LOW);  // ค่าเริ่มต้น = silent

  pinMode(PIN_DRV_ENABLE, OUTPUT);
  digitalWrite(PIN_DRV_ENABLE, LOW);  // ❗ default = disable driver
  pinMode(DIR_L1, OUTPUT);
  pinMode(DIR_L2, OUTPUT);
  pinMode(DIR_R1, OUTPUT);
  pinMode(DIR_R2, OUTPUT);

  pinMode(MAX_CS_L, OUTPUT);
  pinMode(MAX_CS_R, OUTPUT);
  digitalWrite(MAX_CS_L, HIGH);
  digitalWrite(MAX_CS_R, HIGH);
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

  driveSafe();  // <<< HARD motor cut at boot

  // --------------------------------------------------
  // SPI / TEMPERATURE
  // --------------------------------------------------
  // SPI / TEMPERATURE
#if !TEST_MODE
  SPI.begin();
  maxL.begin(MAX31865_3WIRE);
  maxR.begin(MAX31865_3WIRE);
#else
#if DEBUG_SERIAL
  Serial.println(F("[BOOT] TEST MODE - PT100 DISABLED"));
#endif
#endif

  // --------------------------------------------------
  // PWM / SERVO
  // --------------------------------------------------
  driveSafe();
  setPWM_L(0);
  setPWM_R(0);
  setupPWM15K();
  setupFanPWM15K();

  bladeServo.attach(SERVO_ENGINE_PIN);
  bladeServo.writeMicroseconds(1000);

  // --------------------------------------------------
  // INIT SENSOR / STATE
  // --------------------------------------------------
  for (uint8_t i = 0; i < 4; i++) {
    curA[i] = 0.0f;
    overCurCnt[i] = 0;
  }

  tempDriverL = 0;
  tempDriverR = 0;
  engineVolt = 0.0f;

  systemState = SystemState::INIT;
  if (!isValidEnum<SystemState>(static_cast<uint8_t>(systemState))) {
    systemState = SystemState::FAULT;
  }
  driveState = DriveState::IDLE;
  bladeState = BladeState::IDLE;

  faultLatched = false;
  activeFault = FaultCode::NONE;


  revBlockUntilL = 0;
  revBlockUntilR = 0;

  driveSoftStopStart_ms = 0;
  bladeSoftStopStart_ms = 0;

  // --------------------------------------------------
  // MISSING RUNTIME RESET (CRITICAL)
  // --------------------------------------------------
  engineStopped_ms = 0;
  starterActive = false;

  lastIbusByte_ms = millis();
  ibusCommLost = true;

  // --------------------------------------------------
  // INIT WATCHDOG DOMAINS (CRITICAL: PREVENT BOOT FALSE TRIP)
  // --------------------------------------------------
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
  // --------------------------------------------------
  // HW WATCHDOG (LAST)
  // --------------------------------------------------
  wdt_reset();
  wdt_enable(WDTO_1S);
  MCUSR &= ~(1 << WDRF);
  wdt_enable(WDTO_1S);
}

void loop() {
  uint32_t now = millis();
  uint32_t loopStart_us = micros();

  // --------------------------------------------------
  // 🔴 REAL-TIME CONTROL (NO MISS + NO RACE)
  // --------------------------------------------------
  uint8_t ticksToRun;

  // -----------------------------
  // ATOMIC SNAPSHOT (CRITICAL)
  // -----------------------------
  cli();
  ticksToRun = controlTicks;
  controlTicks = 0;
  sei();

  // -----------------------------
  // RUN CONTROL LOOP
  // -----------------------------
  uint8_t maxCatchup = 3;  // จำกัด backlog

  while (ticksToRun > 0 && maxCatchup--) {
    ticksToRun--;
   
    uint32_t ctrlStart = micros();

    // -------------------------
    // CONTROL LOOP (REAL-TIME)
    // -------------------------
    uint32_t ctrlNow = millis();
    runControlLoop(ctrlNow, loopStart_us);

    // -------------------------
    // OVERRUN PROTECTION
    // -------------------------
    uint32_t ctrlTime = micros() - ctrlStart;

    if (ctrlTime > CONTROL_PERIOD_US) {
      latchFault(FaultCode::LOOP_OVERRUN);
      break;
    }
  }

  // --------------------------------------------------
  // 🔴 COPY BUFFER (ทำครั้งเดียวหลัง control)
  // --------------------------------------------------
  copyDriveBuffer();

  // --------------------------------------------------
  // BACKGROUND TASK (NON REAL-TIME)
  // --------------------------------------------------
  taskComms(now);
  sensorTask(now);
  taskDriveEvents(now);
  taskSafety(now);

  // --------------------------------------------------
  // SYSTEM GATE (FAULT / EMERGENCY)
  // --------------------------------------------------
  if (!taskSystemGate(now, loopStart_us))
    return;

  // --------------------------------------------------
  // OTHER TASKS
  // --------------------------------------------------
  taskBlade(now);
  taskAux(now);
  taskDriverEnable(now);

  // --------------------------------------------------
  // BACKGROUND / SUPERVISOR
  // --------------------------------------------------
  taskBackground(now);
  taskLoopSupervisor(loopStart_us);
  taskWatchdog();
}

