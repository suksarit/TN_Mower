// ========================================================================================
// SystemTypes.h  
// ========================================================================================

#ifndef SYSTEM_TYPES_H
#define SYSTEM_TYPES_H

#include <stdint.h>

// ============================================================================
// SAFETY STATE
// ============================================================================
enum class SafetyState : uint8_t {
  SAFE = 0,
  WARN = 1,
  LIMP = 2,
  EMERGENCY = 3,
  _COUNT
};

// ============================================================================
// DRIVE EVENT
// ============================================================================
enum class DriveEvent : uint8_t {
  NONE = 0,
  IMBALANCE = 1,
  STUCK_LEFT = 2,
  STUCK_RIGHT = 3,
  WHEEL_LOCK = 4,
  AUTO_REVERSE = 5,

  WHEEL_STUCK = 6,
  TRACTION_LOSS = 7,

  STALL = 8,
  STUCK = 9,
  LOAD_HIGH = 10,
  LOAD_SPIKE = 11,
  CURRENT_LIMIT = 12,
  SAFETY_LIMIT = 13,
  THERMAL_LIMIT = 14,

  RESERVED_15 = 15,
  RESERVED_16 = 16,

  _COUNT
};

// ============================================================================
// SYSTEM STATE
// ============================================================================
enum class SystemState : uint8_t {
  INIT = 0,
  ACTIVE = 1,
  FAULT = 2,
  _COUNT
};

// ============================================================================
// DRIVE STATE
// ============================================================================
enum class DriveState : uint8_t {
  IDLE = 0,
  RUN = 1,
  LIMP = 2,
  SOFT_STOP = 3,
  LOCKED = 4,
  _COUNT
};

// ============================================================================
// BLADE STATE
// ============================================================================
enum class BladeState : uint8_t {
  IDLE = 0,
  RUN = 1,
  SOFT_STOP = 2,
  LOCKED = 3,
  _COUNT
};

// ============================================================================
// FAULT CODE (EEPROM SAFE - DO NOT REORDER)
// ============================================================================
enum class FaultCode : uint8_t {
  NONE = 0,

  IBUS_LOST,
  COMMS_TIMEOUT,

  SENSOR_TIMEOUT,

  LOGIC_WATCHDOG,

  CUR_SENSOR_FAULT,
  VOLT_SENSOR_FAULT,
  TEMP_SENSOR_FAULT,

  OVER_CURRENT,
  OVER_TEMP,

  DRIVE_TIMEOUT,
  BLADE_TIMEOUT,

  LOOP_OVERRUN,
  LOW_VOLTAGE_CRITICAL,

  LOW_VOLTAGE_WARN,

  RESERVED_1,
  RESERVED_2,

  // 🔴 เพิ่มใหม่ (ต้องต่อท้ายเท่านั้น)
  RC_INVALID,
  ENGINE_START_FAIL,

  _COUNT
};

// ============================================================================
// OTHER ENUMS
// ============================================================================
enum class I2CRecoverState : uint8_t {
  IDLE,
  END_BUS,
  BEGIN_BUS,
  REINIT_ADS,
  DONE
};

enum class ACSCalState : uint8_t {
  IDLE,
  START_CH,
  WAIT_CONV,
  NEXT_SAMPLE,
  NEXT_CH,
  DONE,
  FAIL
};

// ============================================================================
// 🔴 KILL TYPE (ใช้กับ BT + SAFETY)
// ============================================================================
enum class KillType : uint8_t {
  NONE = 0,
  SOFT = 1,   // ผ่าน DriveRamp
  HARD = 2,   // ตัดทันที
  _COUNT
};

// ============================================================================
// ENUM VALIDATION
// ============================================================================
template<typename E>
inline bool isValidEnum(uint8_t raw) {
  return raw < static_cast<uint8_t>(E::_COUNT);
}

// ============================================================================
// DEBUG STRING
// ============================================================================

inline const char* safetyStateToString(SafetyState s)
{
  switch (s)
  {
    case SafetyState::SAFE: return "SAFE";
    case SafetyState::WARN: return "WARN";
    case SafetyState::LIMP: return "LIMP";
    case SafetyState::EMERGENCY: return "EMERGENCY";
    default: return "UNKNOWN";
  }
}

inline const char* driveStateToString(DriveState s)
{
  switch (s)
  {
    case DriveState::IDLE: return "IDLE";
    case DriveState::RUN: return "RUN";
    case DriveState::LIMP: return "LIMP";
    case DriveState::SOFT_STOP: return "SOFT_STOP";
    case DriveState::LOCKED: return "LOCKED";
    default: return "UNKNOWN";
  }
}

inline const char* bladeStateToString(BladeState s)
{
  switch (s)
  {
    case BladeState::IDLE: return "IDLE";
    case BladeState::RUN: return "RUN";
    case BladeState::SOFT_STOP: return "SOFT_STOP";
    case BladeState::LOCKED: return "LOCKED";
    default: return "UNKNOWN";
  }
}

inline const char* killTypeToString(KillType k)
{
  switch (k)
  {
    case KillType::NONE: return "NONE";
    case KillType::SOFT: return "SOFT";
    case KillType::HARD: return "HARD";
    default: return "UNKNOWN";
  }
}

inline const char* faultCodeToString(FaultCode f)
{
  switch (f)
  {
    case FaultCode::NONE: return "NONE";
    case FaultCode::IBUS_LOST: return "IBUS_LOST";
    case FaultCode::COMMS_TIMEOUT: return "COMMS_TIMEOUT";
    case FaultCode::SENSOR_TIMEOUT: return "SENSOR_TIMEOUT";
    case FaultCode::LOGIC_WATCHDOG: return "LOGIC_WATCHDOG";
    case FaultCode::CUR_SENSOR_FAULT: return "CUR_SENSOR_FAULT";
    case FaultCode::VOLT_SENSOR_FAULT: return "VOLT_SENSOR_FAULT";
    case FaultCode::TEMP_SENSOR_FAULT: return "TEMP_SENSOR_FAULT";
    case FaultCode::OVER_CURRENT: return "OVER_CURRENT";
    case FaultCode::OVER_TEMP: return "OVER_TEMP";
    case FaultCode::DRIVE_TIMEOUT: return "DRIVE_TIMEOUT";
    case FaultCode::BLADE_TIMEOUT: return "BLADE_TIMEOUT";
    case FaultCode::LOOP_OVERRUN: return "LOOP_OVERRUN";
    case FaultCode::LOW_VOLTAGE_CRITICAL: return "LOW_VOLTAGE_CRITICAL";
    case FaultCode::LOW_VOLTAGE_WARN: return "LOW_VOLTAGE_WARN";
    case FaultCode::RC_INVALID: return "RC_INVALID";
    case FaultCode::ENGINE_START_FAIL: return "ENGINE_START_FAIL";
    default: return "UNKNOWN";
  }
}

#endif

