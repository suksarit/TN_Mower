// ========================================================================================
// GlobalState.h  (FINAL - INDUSTRIAL KILL + WATCHDOG + ANTI-GLITCH)
// ========================================================================================

#pragma once
#ifndef GLOBAL_STATE_H
#define GLOBAL_STATE_H

#include <Arduino.h>
#include <stdint.h>

#include <IBusBM.h>
#include <Servo.h>
#include <Adafruit_ADS1X15.h>

#include "SystemTypes.h"

// ======================================================
// 🔴 KILL SYSTEM (CRITICAL)
// ======================================================
extern volatile bool killISRFlag;
extern bool killLatched;
extern KillType killRequest;

// ======================================================
// 🔴 CONTROL WATCHDOG (FREEZE DETECT)
// ======================================================
extern volatile uint32_t lastControlExec_us;

// ======================================================
// 🔴 ANTI-GLITCH FILTER
// ======================================================
extern float targetL_filtered;
extern float targetR_filtered;

// ======================================================
// GLOBAL VARIABLES
// ======================================================
extern float terrainDragAvg;
extern float controlDt_s;
extern uint32_t rcLastFrame_ms;
extern uint16_t rcLight;

// ======================================================
// DRIVER STATE ENUM
// ======================================================
enum class DriverState : uint8_t {
  DISABLED = 0,
  ARMING,
  SETTLING,
  ACTIVE
};

// ======================================================
// SYSTEM STATE
// ======================================================
inline SystemState systemState = SystemState::INIT;
inline DriveState driveState   = DriveState::IDLE;
inline BladeState bladeState   = BladeState::IDLE;

inline DriverState driverState = DriverState::DISABLED;

// ======================================================
// DRIVER TIMERS
// ======================================================
inline uint32_t driverStateStart_ms  = 0;
inline uint32_t driverEnabled_ms     = 0;
inline uint32_t driverActiveStart_ms = 0;
inline uint32_t systemActiveStart_ms = 0;

inline bool driverRearmRequired = true;

// ======================================================
// RC CACHE
// ======================================================
inline uint16_t rcThrottle = 1500;
inline uint16_t rcSteer    = 1500;

inline uint16_t rcEngine   = 1000;
inline uint16_t rcIgnition = 1000;
inline uint16_t rcStarter  = 1000;

// ======================================================
// IBUS STATE
// ======================================================
inline uint32_t lastIbusByte_ms     = 0;
inline bool ibusCommLost            = false;
inline bool requireIbusConfirm      = false;
inline uint32_t ibusRecoverStart_ms = 0;

// ======================================================
// ENGINE STATE
// ======================================================
inline float engineVolt = 0.0f;
inline uint32_t engineStopped_ms = 0;
inline uint32_t starterStart_ms  = 0;

inline bool ignitionActive = false;
inline bool engineRunning  = false;
inline bool starterActive  = false;

// ======================================================
// SENSOR VALUES
// ======================================================
inline float curA[4]          = {0.0f,0.0f,0.0f,0.0f};
inline float curA_snapshot[4] = {0.0f,0.0f,0.0f,0.0f};

inline int16_t tempDriverL = 0;
inline int16_t tempDriverR = 0;

// ======================================================
// CURRENT SENSOR OFFSET
// ======================================================
inline float g_acsOffsetV[4] = {2.5f,2.5f,2.5f,2.5f};
inline float currentOffset[4] = {0.0f,0.0f,0.0f,0.0f};

inline uint8_t overCurCnt[4] = {0,0,0,0};

// ======================================================
// DRIVE BUFFER (ISR SAFE)
// ======================================================
struct DriveBuffer {
  int16_t targetL;
  int16_t targetR;
  int16_t curL;
  int16_t curR;
};

extern volatile DriveBuffer driveBufISR;
extern DriveBuffer driveBufMain;

// ======================================================
// AUTO REVERSE
// ======================================================
struct AutoReverseState {
  bool active;
  uint32_t start_ms;
  uint16_t duration_ms;
  int16_t pwm;
};

inline AutoReverseState autoRev = {false,0,0,0};

inline bool autoReverseActive = false;
inline uint8_t autoReverseCount = 0;

// ======================================================
// FAULT STATE
// ======================================================
inline FaultCode activeFault = FaultCode::NONE;
inline bool faultLatched = false;
inline bool rcNeutralConfirmed = false;

// ======================================================
// DRIVE TIMERS
// ======================================================
inline uint32_t bladeSoftStopStart_ms = 0;
inline uint32_t driveSoftStopStart_ms = 0;

// ======================================================
// COMM OBJECT
// ======================================================
inline IBusBM ibus;

// ======================================================
// DRIVE VARIABLES
// ======================================================
inline int16_t curL = 0;
inline int16_t curR = 0;

inline int16_t targetL = 0;
inline int16_t targetR = 0;

inline int8_t lastDirL = 0;
inline int8_t lastDirR = 0;

inline uint32_t revBlockUntilL = 0;
inline uint32_t revBlockUntilR = 0;

inline uint32_t reverseRecoveryStart_ms = 0;
inline DriveEvent lastDriveEvent = DriveEvent::NONE;

// ======================================================
// ADS1115
// ======================================================
inline Adafruit_ADS1115 adsCur;
inline Adafruit_ADS1115 adsVolt;

inline bool adsCurPresent  = false;
inline bool adsVoltPresent = false;

// ======================================================
// 🔴 I2C RECOVERY (แก้ error ที่คุณเจอ)
// ======================================================
inline I2CRecoverState i2cState = I2CRecoverState::IDLE;
inline uint32_t i2cRecoverStart_ms = 0;

// ======================================================
// WATCHDOG DOMAINS
// ======================================================
struct WatchdogDomain {
  uint32_t lastUpdate_ms;
  uint32_t timeout_ms;
  bool faulted;
};

inline WatchdogDomain wdSensor = {0,120,false};
inline WatchdogDomain wdComms  = {0,150,false};
inline WatchdogDomain wdDrive  = {0,100,false};
inline WatchdogDomain wdBlade  = {0,100,false};

// ======================================================
// HELPER FUNCTIONS
// ======================================================
inline bool neutral(uint16_t v)
{
  return (v > 1470 && v < 1530);
}

inline float curLeft()
{
  return curA[0] + curA[1];
}

inline float curRight()
{
  return curA[2] + curA[3];
}

// ======================================================
// SERVO
// ======================================================
inline Servo bladeServo;

#endif

