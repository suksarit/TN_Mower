//SensorManager.cpp

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MAX31865.h>

#include "SensorManager.h"
#include "GlobalState.h"
#include "HardwareConfig.h"
#include "FaultManager.h"
#include "SystemTypes.h"
#include "DriveController.h" 

// ======================================================
// I2C BUS CLEAR PROTOTYPE
// ======================================================
void i2cBusClear();

// ======================================================
// SENSOR FSM (DETERMINISTIC SAMPLING)
// ======================================================

enum class SensorState : uint8_t {
  IDLE,
  START_CONV,
  WAIT_CONV,
  READ_VALUE,
  NEXT_CH,
  COMPLETE
};

static SensorState sensorState = SensorState::IDLE;

static uint8_t sensorCh = 0;
static uint32_t sensorTimer_ms = 0;

// sample rate (สำคัญมาก)
constexpr uint16_t SENSOR_SAMPLE_PERIOD_MS = 10;  // 100Hz



bool updateSensors()
{
  uint32_t now = millis();

  // ==================================================
  // SENSOR PRESENCE GUARD
  // ==================================================
  if (!adsCurPresent && !adsVoltPresent)
  {
#if DEBUG_SERIAL
    Serial.println(F("[SENSOR] ADS NOT PRESENT"));
#endif

    if (i2cState == I2CRecoverState::IDLE)
      i2cState = I2CRecoverState::END_BUS;

    return false;
  }

#if TEST_MODE
  curA[0] = 5.0f;
  curA[1] = 5.0f;
  curA[2] = 5.0f;
  curA[3] = 5.0f;

  tempDriverL = 45;
  tempDriverR = 47;

  engineVolt = 26.0f;

  return true;
#endif

  // ==================================================
  // I2C RECOVERY (ของเดิม - ไม่แตะ)
  // ==================================================

  constexpr uint8_t I2C_MAX_RECOVER = 5;

  static uint32_t i2cStateStart_ms = 0;
  static uint8_t  i2cResetCount = 0;
  static uint32_t lastRecover_ms = 0;

  if (Wire.getWireTimeoutFlag() &&
      i2cState == I2CRecoverState::IDLE &&
      (now - lastRecover_ms > 2000))
  {
    Wire.clearWireTimeoutFlag();
    i2cState = I2CRecoverState::END_BUS;
    i2cStateStart_ms = now;
  }

  switch (i2cState)
  {
    case I2CRecoverState::END_BUS:
      Wire.end();
      wdSensor.lastUpdate_ms = now;
      i2cState = I2CRecoverState::BEGIN_BUS;
      i2cStateStart_ms = now;
      return false;

    case I2CRecoverState::BEGIN_BUS:
      if (now - i2cStateStart_ms < 1) return false;

      i2cBusClear();

      Wire.begin();
      Wire.setClock(100000);
      Wire.setWireTimeout(6000, true);

      wdSensor.lastUpdate_ms = now;
      i2cState = I2CRecoverState::REINIT_ADS;
      return false;

    case I2CRecoverState::REINIT_ADS:
    {
      static uint8_t adsStage = 0;

      switch (adsStage)
      {
        case 0:
          adsCurPresent = adsCur.begin(0x48);
          if (adsCurPresent)
          {
            adsCur.setGain(GAIN_ONE);
            adsCur.setDataRate(RATE_ADS1115_250SPS);
          }
          wdSensor.lastUpdate_ms = now;
          adsStage = 1;
          return false;

        case 1:
          adsVoltPresent = adsVolt.begin(0x49);
          if (adsVoltPresent)
          {
            adsVolt.setGain(GAIN_ONE);
            adsVolt.setDataRate(RATE_ADS1115_250SPS);
          }
          wdSensor.lastUpdate_ms = now;
          adsStage = 2;
          return false;

        case 2:
          adsStage = 0;
          i2cResetCount++;
          i2cState = I2CRecoverState::DONE;
          return false;
      }
      break;
    }

    case I2CRecoverState::DONE:
      if (i2cResetCount >= I2C_MAX_RECOVER)
      {
#if DEBUG_SERIAL
        Serial.println(F("[I2C] MAX RECOVERY EXCEEDED"));
#endif
        latchFault(FaultCode::VOLT_SENSOR_FAULT);
        return false;
      }

      lastRecover_ms = now;
      i2cResetCount = 0;
      i2cState = I2CRecoverState::IDLE;
      wdSensor.lastUpdate_ms = now;
      return false;

    default:
      break;
  }

  // ==================================================
  // HARDWARE OVERCURRENT
  // ==================================================
  if (digitalRead(PIN_CUR_TRIP) == LOW)
  {
    latchFault(FaultCode::OVER_CURRENT);
    return false;
  }

  // ==================================================
  // 🔴 NEW: DETERMINISTIC SENSOR FSM
  // ==================================================

  enum class SensorState : uint8_t {
    IDLE,
    START,
    WAIT,
    READ,
    NEXT,
    COMPLETE
  };

  static SensorState state = SensorState::IDLE;
  static uint8_t ch = 0;
  static uint32_t sampleStart_ms = 0;

  constexpr uint16_t SAMPLE_PERIOD_MS = 15;   // realistic for ADS1115

  static uint32_t convStart_ms = 0;

  switch (state)
  {
    // --------------------------------------------------
    case SensorState::IDLE:

      if (now - sampleStart_ms < SAMPLE_PERIOD_MS)
        return false;

      sampleStart_ms = now;
      ch = 0;
      state = SensorState::START;
      return false;

    // --------------------------------------------------
    case SensorState::START:
    {
      uint16_t mux =
        ADS1X15_REG_CONFIG_MUX_SINGLE_0 +
        (ADS_CUR_CH_MAP[ch] << 12);

      adsCur.startADCReading(mux, false);

      convStart_ms = now;
      state = SensorState::WAIT;
      return false;
    }

    // --------------------------------------------------
    case SensorState::WAIT:

      if (!adsCur.conversionComplete())
      {
        if (now - convStart_ms > 25)
        {
          state = SensorState::NEXT;
        }
        return false;
      }

      state = SensorState::READ;
      return false;

    // --------------------------------------------------
    case SensorState::READ:
    {
      int16_t raw = adsCur.getLastConversionResults();

      float v = raw * ADS1115_LSB_V;

      float rawA =
        (v - g_acsOffsetV[ch]) /
        ACS_SENS_V_PER_A;

      float a = rawA - currentOffset[ch];

      // LPF
      curA[ch] +=
        CUR_LPF_ALPHA * (a - curA[ch]);

      // OVERCURRENT
      if (a > CUR_TRIP_A_CH[ch])
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

      state = SensorState::NEXT;
      return false;
    }

    // --------------------------------------------------
    case SensorState::NEXT:

      ch++;

      if (ch >= 4)
        state = SensorState::COMPLETE;
      else
        state = SensorState::START;

      return false;

    // --------------------------------------------------
    case SensorState::COMPLETE:

      wdSensor.lastUpdate_ms = now;

      state = SensorState::IDLE;

      return true;
  }

  return false;
}


// ======================================================
// I2C BUS CLEAR IMPLEMENTATION
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

