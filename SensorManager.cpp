// ============================================================================
// SensorManager.cpp 
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MAX31865.h>

#include "SensorManager.h"
#include "GlobalState.h"
#include "HardwareConfig.h"
#include "FaultManager.h"
#include "SystemTypes.h"

#ifndef BUDGET_SENSORS_MS
#define BUDGET_SENSORS_MS 5
#endif

#ifndef PHASE_BUDGET_CONFIRM
#define PHASE_BUDGET_CONFIRM 3
#endif

#define CUR_CH_LEFT   0
#define CUR_CH_RIGHT  1

// ======================================================
// FILTER STATE
// ======================================================
static float curPrev[4]   = {0};
static float slopeLPF[4]  = {0};
static float curFilt[4]   = {0};

// ======================================================
// SPEED STATE
// ======================================================
static float speedEstL = 0;
static float speedEstR = 0;

// ======================================================
static volatile uint8_t adcSyncCounter = 0;

// ======================================================
static uint8_t sensorFailCnt = 0;
static bool sensorDegraded = false;
static uint32_t failStart_ms = 0;

// 🔴 NEW: timeout tracking
static uint32_t lastGoodRead_ms = 0;

// ======================================================
static uint32_t i2cTimer = 0;

// ======================================================
void i2cBusClear()
{
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);

  for (uint8_t i = 0; i < 9; i++)
  {
    pinMode(SCL, OUTPUT);
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);

    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(5);
  }
}

// ======================================================
static void resetSensorState()
{
  for (uint8_t i = 0; i < 4; i++)
  {
    curPrev[i] = 0;
    slopeLPF[i] = 0;
    curFilt[i] = 0;
    curA[i] = 0;
  }

  speedEstL = 0;
  speedEstR = 0;
}

// ======================================================
static void updateI2CRecovery(uint32_t now)
{
  switch (i2cState)
  {
    case I2CRecoverState::IDLE:
      return;

    case I2CRecoverState::END_BUS:
      i2cBusClear();
      i2cTimer = now;
      i2cState = I2CRecoverState::BEGIN_BUS;
      break;

    case I2CRecoverState::BEGIN_BUS:
      if (now - i2cTimer < 10) return;

      Wire.begin();
      Wire.setClock(100000);
      Wire.setWireTimeout(6000, true);

      i2cState = I2CRecoverState::REINIT_ADS;
      break;

    case I2CRecoverState::REINIT_ADS:
      adsCurPresent  = adsCur.begin(0x48);
      adsVoltPresent = adsVolt.begin(0x49);

      resetSensorState();

#if DEBUG_SERIAL
      Serial.println(F("[I2C RECOVERY DONE]"));
#endif

      i2cState = I2CRecoverState::DONE;
      break;

    case I2CRecoverState::DONE:
      i2cState = I2CRecoverState::IDLE;
      break;
  }
}

// ======================================================
void sensorAdcTrigger()
{
  if (adcSyncCounter < 255)
    adcSyncCounter++;
}

// ======================================================
static bool updateADSCurrent()
{
  static uint8_t ch = 0;
  static bool convRunning = false;
  static uint32_t convStart_us = 0;

  uint32_t now_us = micros();
  constexpr uint16_t CONV_TIMEOUT_US = 12000;

  if (!convRunning)
  {
    uint16_t mux =
      ADS1X15_REG_CONFIG_MUX_SINGLE_0 +
      (ADS_CUR_CH_MAP[ch] << 12);

    adsCur.startADCReading(mux, false);

    convStart_us = now_us;
    convRunning = true;
    return false;
  }

  if (!adsCur.conversionComplete())
  {
    if (now_us - convStart_us > CONV_TIMEOUT_US)
    {
      i2cState = I2CRecoverState::END_BUS;
      convRunning = false;
    }
    return false;
  }

  int16_t raw = adsCur.getLastConversionResults();
  convRunning = false;

  float v = raw * ADS1115_LSB_V;

  float a =
    ((v - g_acsOffsetV[ch]) / ACS_SENS_V_PER_A) -
    currentOffset[ch];

  // ==================================================
  // 🔴 HARD CLAMP (กันค่าเพี้ยน)
  // ==================================================
  a = constrain(a, -CUR_MAX_PLAUSIBLE, CUR_MAX_PLAUSIBLE);

  // ==================================================
  // SPIKE LIMIT
  // ==================================================
  constexpr float MAX_DELTA = 15.0f;

  float deltaRaw = a - curPrev[ch];

  if (fabs(deltaRaw) > MAX_DELTA)
    a = curPrev[ch] + constrain(deltaRaw, -MAX_DELTA, MAX_DELTA);

  // ==================================================
  // FILTER
  // ==================================================
  slopeLPF[ch] += 0.25f * ((a - curPrev[ch]) - slopeLPF[ch]);

  float pred = a + slopeLPF[ch] * 0.3f;

  curPrev[ch] = a;

  float curUse =
    constrain((a * 0.75f + pred * 0.25f),
              -CUR_MAX_PLAUSIBLE,
              CUR_MAX_PLAUSIBLE);

  // ==================================================
  // VALID
  // ==================================================
  curA[ch] += 0.15f * (curUse - curA[ch]);
  curFilt[ch] = curFilt[ch] * 0.9f + curA[ch] * 0.1f;

  if (fabs(curUse) > CUR_TRIP_A_CH[ch])
  {
    if (++overCurCnt[ch] >= 3)
    {
      requestFault(FaultCode::OVER_CURRENT);
      return false;
    }
  }
  else
  {
    overCurCnt[ch] = 0;
  }

  ch = (ch + 1) % 4;

  return true;
}

// ======================================================
bool updateSensors()
{
  if (adcSyncCounter == 0)
    return false;

  adcSyncCounter--;

  if (!adsCurPresent)
    return false;

  if (digitalRead(PIN_CUR_TRIP) == LOW)
  {
    requestFault(FaultCode::OVER_CURRENT);
    return false;
  }

  bool ok = updateADSCurrent();

  // ==================================================
  // 🔴 SPEED ESTIMATION (stable)
  // ==================================================
  speedEstL = speedEstL * 0.9f + curFilt[CUR_CH_LEFT] * 0.1f;
  speedEstR = speedEstR * 0.9f + curFilt[CUR_CH_RIGHT] * 0.1f;

  uint32_t now = millis();

  if (ok)
  {
    lastGoodRead_ms = now;
    wdSensor.lastUpdate_ms = now;
  }

  // ==================================================
  // 🔴 SENSOR TIMEOUT (ของจริง)
  // ==================================================
  if (now - lastGoodRead_ms > 300)
  {
    sensorDegraded = true;
  }

  if (now - lastGoodRead_ms > 800)
  {
    requestFault(FaultCode::SENSOR_TIMEOUT);
  }

  return ok;
}

// ======================================================
float getMotorCurrentL(void)
{
  return curFilt[CUR_CH_LEFT];
}

float getMotorCurrentR(void)
{
  return curFilt[CUR_CH_RIGHT];
}

float getMotorCurrentSafeL(void)
{
  float c = getMotorCurrentL();
  if (sensorDegraded) c *= 0.6f;
  return c;
}

float getMotorCurrentSafeR(void)
{
  float c = getMotorCurrentR();
  if (sensorDegraded) c *= 0.6f;
  return c;
}

// ======================================================
float getSpeedL(void)
{
  return speedEstL * 0.02f;
}

float getSpeedR(void)
{
  return speedEstR * 0.02f;
}

// ======================================================
void sensorTask(uint32_t now)
{
  uint32_t tStart = micros();

  bool ok = updateSensors();

  uint32_t dt = micros() - tStart;

  static uint8_t budgetCnt = 0;

  if (dt > (BUDGET_SENSORS_MS * 1000UL))
  {
    if (++budgetCnt >= PHASE_BUDGET_CONFIRM)
      requestFault(FaultCode::SENSOR_TIMEOUT);
  }
  else
  {
    budgetCnt = 0;
  }

  if (i2cState != I2CRecoverState::IDLE)
  {
    updateI2CRecovery(now);
    return;
  }

  if (ok)
  {
    failStart_ms = 0;
    sensorFailCnt = 0;

    if (sensorDegraded && millis() - lastGoodRead_ms < 200)
      sensorDegraded = false;

    return;
  }

  if (failStart_ms == 0)
    failStart_ms = now;

  uint32_t failTime = now - failStart_ms;

  if (failTime > 1200)
    sensorDegraded = true;

  if (failTime > 2000)
  {
    i2cState = I2CRecoverState::END_BUS;
    return;
  }

  if (failTime > 4000)
  {
    if (++sensorFailCnt >= 2)
      requestFault(FaultCode::SENSOR_TIMEOUT);
  }
}



