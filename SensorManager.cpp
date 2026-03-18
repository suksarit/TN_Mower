// ============================================================================
// SensorManager.cpp  (FIXED - ADD CURRENT GETTERS + CHANNEL MAP)
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

// ======================================================
// 🔴 CHANNEL MAP (ต้องตรงกับการต่อจริง)
// ======================================================
#define CUR_CH_LEFT   0
#define CUR_CH_RIGHT  1

// ======================================================
// LOCAL STATE (FILE ONLY)
// ======================================================
static float curPrev[4]   = {0,0,0,0};
static float slopeLPF[4]  = {0,0,0,0};

// ======================================================
// I2C BUS CLEAR PROTOTYPE
// ======================================================
void i2cBusClear();

// ======================================================
// SYNC COUNTER
// ======================================================
static volatile uint8_t adcSyncCounter = 0;

void sensorAdcTrigger()
{
  if (adcSyncCounter < 255)
    adcSyncCounter++;
}

// ======================================================
// SENSOR UPDATE
// ======================================================
bool updateSensors()
{
  if (adcSyncCounter == 0)
    return false;

  adcSyncCounter--;

  uint32_t now_ms = millis();
  uint32_t now_us = micros();

  if (!adsCurPresent)
    return false;

  // ==================================================
  // HARDWARE OVERCURRENT
  // ==================================================
  if (digitalRead(PIN_CUR_TRIP) == LOW)
  {
    latchFault(FaultCode::OVER_CURRENT);
    return false;
  }

  // ==================================================
  // PIPELINE STATE
  // ==================================================
  static uint8_t ch = 0;
  static bool convRunning = false;
  static uint32_t convStart_us = 0;

  constexpr uint16_t CONV_TIMEOUT_US = 12000;

  // ==================================================
  // START CONVERSION
  // ==================================================
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

  // ==================================================
  // WAIT COMPLETE
  // ==================================================
  if (!adsCur.conversionComplete())
  {
    if (now_us - convStart_us > CONV_TIMEOUT_US)
    {
#if DEBUG_SERIAL
      Serial.println(F("[ADS TIMEOUT]"));
#endif
      convRunning = false;
    }
    return false;
  }

  // ==================================================
  // READ VALUE
  // ==================================================
  int16_t raw = adsCur.getLastConversionResults();
  convRunning = false;

  float v = raw * ADS1115_LSB_V;

  float rawA =
    (v - g_acsOffsetV[ch]) /
    ACS_SENS_V_PER_A;

  float a = rawA - currentOffset[ch];

  // ==================================================
  // SPIKE REJECTION
  // ==================================================
  constexpr float MAX_DELTA = 25.0f;

  if (fabs(a - curPrev[ch]) > MAX_DELTA)
  {
    a = curPrev[ch];
  }

  // ==================================================
  // PREDICTIVE
  // ==================================================
  float delta = a - curPrev[ch];

  constexpr float slopeAlpha = 0.4f;
  slopeLPF[ch] += slopeAlpha * (delta - slopeLPF[ch]);

  float pred = a + slopeLPF[ch] * 0.8f;

  if (pred < 0) pred = 0;
  if (pred > CUR_MAX_PLAUSIBLE) pred = CUR_MAX_PLAUSIBLE;

  curPrev[ch] = a;

  // ==================================================
  // PLAUSIBILITY + LPF
  // ==================================================
  if (a >= CUR_MIN_PLAUSIBLE && a <= CUR_MAX_PLAUSIBLE)
  {
    constexpr float alpha = 0.2f;
    curA[ch] += alpha * (a - curA[ch]);

    float curUse = max(a, pred);

    if (curUse > CUR_TRIP_A_CH[ch])
    {
      if (++overCurCnt[ch] >= 2)
      {
        latchFault(FaultCode::OVER_CURRENT);
        return false;
      }
    }
    else
    {
      overCurCnt[ch] = 0;
    }
  }

  // ==================================================
  // NEXT CHANNEL
  // ==================================================
  ch++;
  if (ch >= 4) ch = 0;

  uint16_t mux =
    ADS1X15_REG_CONFIG_MUX_SINGLE_0 +
    (ADS_CUR_CH_MAP[ch] << 12);

  adsCur.startADCReading(mux, false);
  convStart_us = now_us;
  convRunning = true;

  // ==================================================
  // WATCHDOG
  // ==================================================
  wdSensor.lastUpdate_ms = now_ms;

  return true;
}

// ======================================================
// 🔴 CURRENT GETTERS (FIX ERROR)
// ======================================================
float getMotorCurrentL(void)
{
  return curA[CUR_CH_LEFT];   // ใช้ค่าที่ผ่าน LPF แล้ว
}

float getMotorCurrentR(void)
{
  return curA[CUR_CH_RIGHT];
}

// ======================================================
// I2C BUS CLEAR
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

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
}

